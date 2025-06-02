import bpy
import open3d as o3d
import numpy as np


def get_min_volume_box(points):
    # Convert to Open3D format
    np_points = np.array(points)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np_points)

    # Compute the oriented bounding box
    obb = pcd.get_oriented_bounding_box()

    return obb


def get_ordered_box_corners(obb):
    # Get the center, rotation matrix, and half-extents
    center = obb.center
    R = obb.R  # Rotation matrix
    extents = obb.extent / 2  # extents is full size, we want half-size

    # Local corner offsets (from center), axis-aligned in local frame
    signs = [
        [-1, -1, -1],  # 0
        [ 1, -1, -1],  # 1
        [ 1,  1, -1],  # 2
        [-1,  1, -1],  # 3
        [-1, -1,  1],  # 4
        [ 1, -1,  1],  # 5
        [ 1,  1,  1],  # 6
        [-1,  1,  1]   # 7
    ]

    corners = []
    for s in signs:
        local_offset = R @ (s * extents)
        world_corner = center + local_offset
        corners.append(world_corner)

    return np.array(corners)


def rotation_matrix_around_axis(axis, angle):
    axis = axis / np.linalg.norm(axis)  # Normalize the axis
    x, y, z = axis
    c = np.cos(angle)
    s = np.sin(angle)
    t = 1 - c

    # Rodrigues' rotation formula
    rotation_matrix = np.array([
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c]
    ])
    return rotation_matrix


# Computes the similarity between an obb and a pcd. Similarity is 1 if all points from the pcd are in the obb, and closest to 0 the more numerous and further away they are.
def compute_similarity(obb, pcd):
    # Extract points from the point cloud
    points = np.asarray(pcd.points)

    # Get indices of points inside the OBB
    inside_indices = obb.get_point_indices_within_bounding_box(o3d.utility.Vector3dVector(points))

    # Mask for points outside the OBB
    outside_mask = np.ones(len(points), dtype=bool)
    outside_mask[inside_indices] = False

    # Extract points outside the OBB
    outside_points = points[outside_mask]

    # Compute squared distances from outside points to the OBB
    distances = []
    for point in outside_points:
        # Transform the point to the OBB's local coordinate system
        local_point = np.dot(obb.R.T, point - obb.center)

        # Clamp the local point to the OBB's extents
        clamped_point = np.clip(local_point, -obb.extent / 2, obb.extent / 2)

        # Transform the clamped point back to the world coordinate system
        closest_point = np.dot(obb.R, clamped_point) + obb.center

        # Compute the squared distance
        distances.append(np.linalg.norm(point - closest_point) ** 2)

    # Return the inverse of the sum of squared distances
    return 1 / np.sum(distances) if distances else float('inf')


# Computes and returns the obb in the input list that matches the input pcd best
def get_best_obb(possible_obbs, pcd):
    similarities = []
    for obb in possible_obbs:
        similarities.append(compute_similarity(obb, pcd))

    return possible_obbs[np.argmax(similarities)]


def get_obbs_from_planes(planes, planes_labels, major_angles):
    print("Reconstructing the planes to oriented bounding boxes...")

    planes_obbs = []
    ma1, ma2 = major_angles
    for i, plane_cloud in enumerate(planes):
        obb = get_min_volume_box(plane_cloud.points)
        obb.color = [0, 0, 0]  # Set color to black for better visualization

        # Compute the rotation matrixes with major angles around Z
        R_ma1 = rotation_matrix_around_axis([0, 0, 1], ma1)
        R_ma2 = rotation_matrix_around_axis([0, 0, 1], ma2)
        # Compute new obbs for later testing
        obb_ma1 = o3d.geometry.OrientedBoundingBox(center=obb.center, R=R_ma1, extent=obb.extent)
        obb_ma2 = o3d.geometry.OrientedBoundingBox(center=obb.center, R=R_ma2, extent=obb.extent)

        if planes_labels[i] == "floor":
            obb.R = get_best_obb([obb_ma1, obb_ma2], plane_cloud).R  # Choose the best fitting rotation

        elif planes_labels[i] == "wall_mv1":
            R_ma1 @= rotation_matrix_around_axis([1, 0, 0], np.radians(90)) @ rotation_matrix_around_axis([0, 1, 0], np.radians(90))
            R_ma2 @= rotation_matrix_around_axis([1, 0, 0], np.radians(90))
            obb_ma1.R = R_ma1
            obb_ma2.R = R_ma2

            obb.R = get_best_obb([obb_ma1, obb_ma2], plane_cloud).R  # Choose the best fitting Z rotation

        else:  # wall_mv2
            R_ma1 @= rotation_matrix_around_axis([0, 1, 0], np.radians(90)) @ rotation_matrix_around_axis([1, 0, 0], np.radians(90))
            R_ma2 @= rotation_matrix_around_axis([0, 1, 0], np.radians(90))
            obb_ma1.R = R_ma1
            obb_ma2.R = R_ma2

            obb.R = get_best_obb([obb_ma1, obb_ma2], plane_cloud).R  # Choose the best fitting Z rotation

        planes_obbs.append(obb)

    return planes_obbs


def export_objects(obbs, output="my_mesh.obj"):
    corners_list = [get_ordered_box_corners(obb) for obb in obbs]

    # Define the faces using the indices of the vertices
    faces = [
        (0, 1, 2, 3),  # Bottom face
        (4, 5, 6, 7),  # Top face
        (0, 1, 5, 4),  # Front face
        (2, 3, 7, 6),  # Back face
        (1, 2, 6, 5),  # Right face
        (0, 3, 7, 4)   # Left face
    ]

    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)

    for k in range(len(corners_list)):
        corners = corners_list[k]

        # Create a new mesh and object
        mesh = bpy.data.meshes.new(name="CustomCube")
        obj = bpy.data.objects.new(name="CustomCube", object_data=mesh)

        # Link the object to the active collection
        bpy.context.collection.objects.link(obj)

        # Create the mesh using the defined vertices and faces
        mesh.from_pydata(corners, [], faces)

        # Update the mesh with new data
        mesh.update()

    bpy.ops.wm.obj_export(filepath=output)


if __name__ == "__main__":
    point_cloud = [
        [1, 2, 3],
        [4, 5, 6],
        [7, 6, 5],
        [3, 4, 2],
        [2, 1, 0]
    ]

    obb = get_min_volume_box(point_cloud)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(point_cloud))
    obb.color = (1, 0, 0)

    o3d.visualization.draw_geometries([pcd, obb])

    export_objects([obb])

