# the objective of this file is to take similar objects close to each others and merge them as one.


import open3d as o3d
import numpy as np
from sklearn.cluster import AgglomerativeClustering

def cluster_point_clouds(point_clouds, normal_threshold=0.99, max_gap=0.5):
    """
    Clusters planar point clouds if they are parallel and within a gap distance.
    Returns merged point clouds per cluster.
    
    Args:
        point_clouds (list of o3d.geometry.PointCloud): Input planes.
        normal_threshold (float): Min cosine similarity for parallelism (0.99 ≈ ~8°).
        max_gap (float): Max orthogonal separation between parallel planes.
    
    Returns:
        list of o3d.geometry.PointCloud: Merged point clouds (one per cluster).
    """
    if not point_clouds:
        return []
    
    # Estimate normals (skip orientation for planar clouds)
    normals = []
    centroids = []
    for pcd in point_clouds:
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
        )
        normals.append(np.asarray(pcd.normals)[0])  # Use first normal (planar assumption)
        centroids.append(np.mean(np.asarray(pcd.points), axis=0))
    
    # Pairwise distance matrix
    n = len(point_clouds)
    distance_matrix = 1e10 * np.ones((n, n))
    
    for i in range(n):
        for j in range(i + 1, n):
            # Check normal alignment
            cos_sim = np.abs(np.dot(normals[i], normals[j]))
            if cos_sim < normal_threshold:
                continue  # Not parallel
            
            # Orthogonal gap distance
            gap = np.abs(np.dot(centroids[j] - centroids[i], normals[i]))
            if gap <= max_gap:
                distance_matrix[i, j] = gap
                distance_matrix[j, i] = gap
    
    # Cluster (agglomerative, stops when gaps > max_gap)
    clustering = AgglomerativeClustering(
        n_clusters=None,
        metric="precomputed",
        linkage="complete",
        distance_threshold=max_gap
    )
    cluster_labels = clustering.fit_predict(distance_matrix)
    
    # Merge point clouds per cluster
    clustered_pcds = []
    for label in np.unique(cluster_labels):
        indices = np.where(cluster_labels == label)[0]
        merged_points = np.vstack([np.asarray(point_clouds[i].points) for i in indices])
        merged_pcd = o3d.geometry.PointCloud()
        merged_pcd.points = o3d.utility.Vector3dVector(merged_points)
        clustered_pcds.append(merged_pcd)
    
    return clustered_pcds

# Example usage
if __name__ == "__main__":
    # Create 4 planes (2 clusters)
    plane1 = o3d.geometry.PointCloud()
    plane1.points = o3d.utility.Vector3dVector(np.random.rand(100, 3) * 0.5)
    plane1.points = o3d.utility.Vector3dVector(np.asarray(plane1.points) * [1, 1, 0])  # Flatten to z=0
    
    plane2 = o3d.geometry.PointCloud()
    plane2.points = o3d.utility.Vector3dVector(np.asarray(plane1.points) + [0, 0, 0.8])  # Shift along z
    
    plane3 = o3d.geometry.PointCloud()
    plane3.points = o3d.utility.Vector3dVector(np.random.rand(100, 3) * 0.5)
    plane3.points = o3d.utility.Vector3dVector(np.asarray(plane3.points) * [1, 0, 1])  # Flatten to y=0
    
    plane4 = o3d.geometry.PointCloud()
    plane4.points = o3d.utility.Vector3dVector(np.asarray(plane3.points) + [0, 0.7, 0])  # Shift along y
    
    # Cluster
    clustered_planes = cluster_point_clouds([plane1, plane2, plane3, plane4])
    print(f"Clustered into {len(clustered_planes)} groups")  # Expected: 2 merged point clouds