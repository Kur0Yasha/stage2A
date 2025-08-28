import numpy as np
import open3d as o3d
from sklearn.cluster import DBSCAN

def cluster_large_objects(point_cloud, eps=0.1, min_samples=4, min_points_per_cluster=1000):
    """
    Applies DBSCAN clustering to a point cloud and returns a list of point clouds,
    each representing a large detected object.

    Args:
        point_cloud (o3d.geometry.PointCloud): The input point cloud.
        eps (float): The maximum distance between two samples for them to be considered as in the same neighborhood.
        min_samples (int): The number of samples in a neighborhood for a point to be considered as a core point.
        min_points_per_cluster (int): Minimum number of points to consider a cluster as a large object.

    Returns:
        list[o3d.geometry.PointCloud]: List of point clouds, each corresponding to a large object.
    """
    # Convert Open3D point cloud to numpy array
    points = np.asarray(point_cloud.points)

    # Apply DBSCAN clustering
    dbscan = DBSCAN(eps=eps, min_samples=min_samples)
    labels = dbscan.fit_predict(points)

    # Prepare result
    unique_labels = set(labels)
    clustered_point_clouds = []

    for label in unique_labels:
        if label == -1:
            # -1 is noise
            continue
        indices = np.where(labels == label)[0]
        if len(indices) < min_points_per_cluster:
            # Ignore small clusters
            continue

        # Create a new point cloud for this cluster
        cluster_pcd = o3d.geometry.PointCloud()
        cluster_pcd.points = o3d.utility.Vector3dVector(points[indices])

        # Optionally copy colors if present
        if point_cloud.has_colors():
            colors = np.asarray(point_cloud.colors)
            cluster_pcd.colors = o3d.utility.Vector3dVector(colors[indices])

        # Optionally copy normals if present
        if point_cloud.has_normals():
            normals = np.asarray(point_cloud.normals)
            cluster_pcd.normals = o3d.utility.Vector3dVector(normals[indices])

        clustered_point_clouds.append(cluster_pcd)

    return clustered_point_clouds