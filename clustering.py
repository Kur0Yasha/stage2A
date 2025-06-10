# the objective of this file is to take similar objects close to each others and merge them as one.


import numpy as np
import open3d as o3d
from scipy.spatial.distance import cdist
from itertools import combinations

def cluster_point_clouds(point_clouds, normal_threshold=0.98, max_gap=0.4, overlap_threshold=0.2):
    """
    Clusters planar point clouds into volumes based on normal alignment and gap distance.
    Performs clustering iteratively until clusters stabilize (no more changes between iterations).
    
    Args:
        point_clouds (list of o3d.geometry.PointCloud): List of planar point clouds.
        normal_threshold (float): Cosine similarity threshold for normals (default: 0.99 ≈ ~2.5°).
        max_gap (float): Maximum allowed orthogonal gap between planes (in meters).
        overlap_threshold (float): Minimum overlap fraction to merge ambiguous candidates.
    
    Returns:
        list of o3d.geometry.PointCloud: Clustered point clouds (merged fragments).
    """
    def single_iteration_clustering(pcs):
        # --- Step 1: Compute normals and centroids for each point cloud ---
        planes = []
        for pc in pcs:
            points = np.asarray(pc.points)
            centroid = np.mean(points, axis=0)
            
            # Fit plane using PCA (smallest eigenvector = normal)
            cov = np.cov(points.T)
            eigvals, eigvecs = np.linalg.eig(cov)
            normal = eigvecs[:, np.argmin(eigvals)]
            
            # Ensure consistent normal orientation (optional)
            if normal[2] < 0:
                normal *= -1
            
            planes.append({
                "points": points,
                "centroid": centroid,
                "normal": normal,
                "pcd": pc  # Keep original point cloud
            })
        
        # --- Step 2: Build adjacency graph based on planar proximity ---
        n = len(planes)
        adjacency = np.zeros((n, n), dtype=bool)
        
        for i, j in combinations(range(n), 2):
            ni, nj = planes[i]["normal"], planes[j]["normal"]
            ci, cj = planes[i]["centroid"], planes[j]["centroid"]
            
            # Check normal alignment (cosine similarity)
            cos_angle = np.abs(np.dot(ni, nj))
            if cos_angle < normal_threshold:
                continue  # Skip non-parallel planes
            
            # Check orthogonal gap
            gap = np.abs(np.dot(ci - cj, ni))
            if gap < max_gap:
                # Optional: Check 2D overlap (if normals are aligned but gap is ambiguous)
                
                # Project points onto the plane's 2D basis (ignore normal axis)
                proj_i = planes[i]["points"][:, :2]  # Simplified; use PCA in practice
                proj_j = planes[j]["points"][:, :2]
                
                # Check if bounding boxes overlap significantly
                min_i, max_i = np.min(proj_i, axis=0), np.max(proj_i, axis=0)
                min_j, max_j = np.min(proj_j, axis=0), np.max(proj_j, axis=0)
                
                overlap_area = np.prod(np.maximum(0, np.minimum(max_i, max_j) - np.maximum(min_i, min_j)))
                area_i = np.prod(max_i - min_i)
                area_j = np.prod(max_j - min_j)
                
                if overlap_area / min(area_i, area_j) < overlap_threshold:
                    continue  # Insufficient overlap
            
                adjacency[i, j] = True
                adjacency[j, i] = True
        
        # --- Step 3: Find connected components (clusters) ---
        clusters = []
        visited = set()
        
        for i in range(n):
            if i not in visited:
                stack = [i]
                cluster_indices = []
                
                while stack:
                    node = stack.pop()
                    if node not in visited:
                        visited.add(node)
                        cluster_indices.append(node)
                        neighbors = np.where(adjacency[node])[0]
                        stack.extend(neighbors)
                
                clusters.append(cluster_indices)
        
        # --- Step 4: Merge point clouds in each cluster ---
        merged_pcds = []
        for cluster in clusters:
            if len(cluster) == 0:
                continue
            
            # Merge all points in the cluster
            merged_points = np.vstack([planes[i]["points"] for i in cluster])
            merged_pcd = o3d.geometry.PointCloud()
            merged_pcd.points = o3d.utility.Vector3dVector(merged_points)
            
            merged_pcds.append(merged_pcd)
        
        return merged_pcds

    # Initial clustering
    current_clusters = single_iteration_clustering(point_clouds)
    
    # Keep clustering until no more changes occur
    while True:
        new_clusters = single_iteration_clustering(current_clusters)
        
        # Check if clusters have changed
        if len(new_clusters) == len(current_clusters):
            # Compare if all point clouds are the same (order might vary)
            matched = True
            used_indices = set()
            for pc1 in current_clusters:
                found_match = False
                points1 = np.asarray(pc1.points)
                for i, pc2 in enumerate(new_clusters):
                    if i not in used_indices:
                        points2 = np.asarray(pc2.points)
                        if np.array_equal(points1, points2):
                            used_indices.add(i)
                            found_match = True
                            break
                if not found_match:
                    matched = False
                    break
            
            if matched:
                break
        
        current_clusters = new_clusters
    
    return current_clusters

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