import numpy as np
import open3d as o3d
from sklearn.cluster import DBSCAN
from sklearn.decomposition import PCA

def findholes(pcd, obb, voxel_size=0.15, eps=1.5, min_samples=5):
    """
    Args:
        pcd (open3d.geometry.PointCloud): Input point cloud
        obb (open3d.geometry.OrientedBoundingBox): The bounding box to operate within
        voxel_size (float): The 2D voxel size for filtering
        eps (float): DBSCAN epsilon for clustering
        min_samples (int): Minimum samples for a core point in DBSCAN

    Returns:
        List[List[np.ndarray]]: Each list contains the original 3D coordinates of one cluster
    """

    cropped_pcd = pcd.crop(obb)
    points = np.asarray(cropped_pcd.points)

    if len(points) == 0:
        return []

    obb_center = obb.center
    obb_axes = np.asarray(obb.R)
    normal = obb_axes[:, 2]
    u_axis = obb_axes[:, 0]
    v_axis = obb_axes[:, 1]

    local_frame = np.stack([u_axis, v_axis, normal], axis=1)
    relative_points = points - obb_center
    local_coords = relative_points @ local_frame
    projected_2d = local_coords[:, :2]


    min_xy = projected_2d.min(axis=0)
    max_xy = projected_2d.max(axis=0)
    grid_shape = np.ceil((max_xy - min_xy) / voxel_size).astype(int)
    voxel_grid = np.zeros(grid_shape, dtype=np.uint8)

    indices_2d = np.floor((projected_2d - min_xy) / voxel_size).astype(int)
    for idx in indices_2d:
        voxel_grid[tuple(idx)] = 1

    reversed_grid = 1 - voxel_grid

    # Only keep reversed points inside convex hull (masking outer frame)
    reversed_coords = []
    for i in range(grid_shape[0]):
        for j in range(grid_shape[1]):
            if reversed_grid[i, j] == 1:
                coord = np.array([i + 0.5, j + 0.5]) * voxel_size + min_xy
                reversed_coords.append(coord)

    reversed_coords = np.array(reversed_coords)
    if len(reversed_coords) == 0:
        return []

    clustering = DBSCAN(eps=eps * voxel_size, min_samples=min_samples).fit(reversed_coords)
    labels = clustering.labels_
    unique_labels = set(labels) - {-1}

    clusters_3d = []
    for label in unique_labels:
        cluster_points_2d = reversed_coords[labels == label]
        cluster_local_coords = np.concatenate([cluster_points_2d, np.zeros((len(cluster_points_2d), 1))], axis=1)
        cluster_world = cluster_local_coords @ local_frame.T + obb_center
        clusters_3d.append(cluster_world)

    final_cluster = []
    for k in clusters_3d:
        if len(k) >= 10:
            final_cluster.append(k)

    return final_cluster


def holeobb(point_clouds, default_thickness=0.1):
    """
    For each point cloud, find the best-fitting oriented bounding box within the same plane.
    
    Args:
        point_clouds (List[List[List[float]]]): List of point clouds (each a list of 3D points).
        default_thickness (float): Value to assign along the normal direction for OBB thickness.

    Returns:
        List[np.ndarray]: List of OBBs (each 4 corners as 3D coordinates).
    """
    obb_list = []

    for points in point_clouds:
        points = np.array(points)
        centroid = np.mean(points, axis=0)

        # PCA to get the plane basis and normal
        pca = PCA(n_components=3)
        pca.fit(points - centroid)
        basis = pca.components_  # rows: direction vectors

        plane_axes = basis[:2]         # X and Y in the plane
        normal = basis[2]              # normal vector of the plane

        # Project points into 2D plane
        local_2d = (points - centroid) @ plane_axes.T  # Shape: (N, 2)



        # Rotating calipers to find minimum area rectangle
        def min_area_rect(pts):
            min_area = np.inf
            best_rect = None

            for i in range(len(pts)):
                p1 = pts[i]
                p2 = pts[(i + 1) % len(pts)]
                edge_dir = p2 - p1
                edge_dir /= np.linalg.norm(edge_dir)
                perp_dir = np.array([-edge_dir[1], edge_dir[0]])

                proj = np.dot(pts, np.vstack([edge_dir, perp_dir]).T)
                min_x, max_x = proj[:, 0].min(), proj[:, 0].max()
                min_y, max_y = proj[:, 1].min(), proj[:, 1].max()

                area = (max_x - min_x) * (max_y - min_y)
                if area < min_area:
                    min_area = area
                    best_rect = (edge_dir, perp_dir, min_x, max_x, min_y, max_y)

            return best_rect

        dir1, dir2, min_x, max_x, min_y, max_y = min_area_rect(local_2d)

        # Generate rectangle corners in 2D
        rect_corners_2d = np.array([
            [min_x, min_y],
            [max_x, min_y],
            [max_x, max_y],
            [min_x, max_y]
        ])

        # --- Area of rectangle ---
        rect_width = max_x - min_x
        rect_height = max_y - min_y
        rect_area = rect_width * rect_height

        # --- Estimated surface covered by points ---
        point_area = 0.15 ** 2  # from VGF spacing
        point_covered_area = len(points) * point_area

        # --- Ratio (density of real surface inside OBB) ---
        coverage_ratio = point_covered_area / rect_area

        # --- Filtering based on coverage threshold ---
        threshold = 0.95  # you can tune this value (0.0 - 1.0)

        if coverage_ratio < threshold:
            continue  # Skip this point cloud (not dense enough)

        # Rotate corners back into local 2D
        transform = np.vstack([dir1, dir2]).T
        obb_2d = rect_corners_2d @ transform.T

        # Lift 2D corners back into 3D
        obb_3d = obb_2d @ plane_axes + centroid

        obb_list.append(obb_3d)
    
    obb_o3d_list = []

    for obb_corners in obb_list:
        # Compute center
        center = np.mean(obb_corners, axis=0)

        # Compute local axes from corners (plane axes from earlier step)
        x_axis = obb_corners[1] - obb_corners[0]
        y_axis = obb_corners[3] - obb_corners[0]
        z_axis = np.cross(x_axis, y_axis)
        
        # Normalize
        x_axis /= np.linalg.norm(x_axis)
        y_axis /= np.linalg.norm(y_axis)
        z_axis /= np.linalg.norm(z_axis)

        R = np.stack([x_axis, y_axis, z_axis], axis=1)  # 3x3 rotation matrix

        # Project corners into local frame to get extent
        local_corners = (obb_corners - center) @ R
        extent = np.array([rect_width, rect_height, default_thickness])

        # Create OBB
        obb = o3d.geometry.OrientedBoundingBox(center, R, extent)
        obb_o3d_list.append(obb)

    return obb_o3d_list