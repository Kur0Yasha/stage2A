import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import os


def removeobb(pcd, obb_list):
    """
    Removes all points from the point cloud that are inside any of the given oriented bounding boxes.

    Parameters:
        pcd (o3d.geometry.PointCloud): Input point cloud.
        obb_list (list of o3d.geometry.OrientedBoundingBox): List of OBBs to remove points from.

    Returns:
        o3d.geometry.PointCloud: New point cloud with points outside all OBBs.
    """
    # Convert point cloud to numpy array for faster processing
    points = np.asarray(pcd.points)
    
    # Boolean mask: True if point is OUTSIDE all OBBs
    mask = np.ones(len(points), dtype=bool)
    
    for obb in obb_list:
        inside_mask = obb.get_point_indices_within_bounding_box(o3d.utility.Vector3dVector(points))
        mask[inside_mask] = False  # Mark points inside any OBB as False
    
    # Create new filtered point cloud
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(points[mask])
    
    # Preserve colors if available
    if pcd.has_colors():
        colors = np.asarray(pcd.colors)
        filtered_pcd.colors = o3d.utility.Vector3dVector(colors[mask])
    
    # Preserve normals if available
    if pcd.has_normals():
        normals = np.asarray(pcd.normals)
        filtered_pcd.normals = o3d.utility.Vector3dVector(normals[mask])
    
    return filtered_pcd




def rotation_matrix(axis, theta):
    """
    Returns the rotation matrix for rotating `theta` radians around `axis` ('x', 'y', or 'z')
    """
    c, s = np.cos(theta), np.sin(theta)
    if axis == 'x':
        return np.array([[1, 0, 0],
                         [0, c, -s],
                         [0, s, c]])
    elif axis == 'y':
        return np.array([[c, 0, s],
                         [0, 1, 0],
                         [-s, 0, c]])
    elif axis == 'z':
        return np.array([[c, -s, 0],
                         [s, c, 0],
                         [0, 0, 1]])
    else:
        raise ValueError("Axis must be 'x', 'y', or 'z'.")

def project_to_image(points, colors=None, img_size=512, padding=0.05):
    """
    Projects 3D points to 2D image coordinates and returns a numpy image array.
    Colors must be Nx3 array of RGB values in range [0, 1] or [0, 255].
    If colors is None or empty, defaults to white.
    """
    xy = points[:, :2]
    if xy.size == 0:
        # No points at all
        return np.zeros((img_size, img_size, 3), dtype=np.uint8)

    min_xy = xy.min(axis=0)
    max_xy = xy.max(axis=0)
    scale = (1 - 2 * padding) * img_size / (max_xy - min_xy).max()

    coords = (xy - min_xy) * scale + padding * img_size
    coords = coords.astype(int)

    # If colors missing, set all to white
    if colors is None or len(colors) == 0:
        colors = np.full((len(points), 3), 255, dtype=np.uint8)
    else:
        if colors.max() <= 1.0:
            colors = (colors * 255).astype(np.uint8)
        else:
            colors = colors.astype(np.uint8)

    img = np.zeros((img_size, img_size, 3), dtype=np.uint8)
    for (x, y), color in zip(coords, colors):
        if 0 <= x < img_size and 0 <= y < img_size:
            img[img_size - 1 - y, x] = color

    return img


def save_pointcloud_views(pcd, index, output_dir="output_images"):
    os.makedirs(output_dir, exist_ok=True)
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)  # Nx3 array, values in [0,1]
    

    # View 1: Top-down (looking along -Z)
    img_top = project_to_image(points, colors)
    plt.imsave(os.path.join(output_dir, f"cluster{index}T.png"), img_top)

    # View 2: 45° from +X axis
    rot_y = rotation_matrix('y', np.radians(45))
    view_x45 = points @ rot_y.T
    img_x45 = project_to_image(view_x45, colors)
    plt.imsave(os.path.join(output_dir, f"cluster{index}Y.png"), img_x45)

    # View 3: 45° from +Y axis
    rot_x = rotation_matrix('x', np.radians(-45))
    view_y45 = points @ rot_x.T
    img_y45 = project_to_image(view_y45, colors)
    plt.imsave(os.path.join(output_dir, f"cluster{index}X.png"), img_y45)