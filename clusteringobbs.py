import numpy as np
import open3d as o3d
from itertools import combinations

def cluster_obbs(obbs, normal_threshold=0.98, max_gap=0.4, overlap_threshold=0.2):
    """
    Clusters oriented bounding boxes (OBBs) based on normal alignment and spatial proximity.
    
    Args:
        obbs (list of o3d.geometry.OrientedBoundingBox): List of oriented bounding boxes.
        normal_threshold (float): Cosine similarity threshold for orientation alignment.
        max_gap (float): Max distance between parallel planes for clustering.
        overlap_threshold (float): Minimum area overlap fraction to merge.
    
    Returns:
        list of o3d.geometry.OrientedBoundingBox: Clustered (merged) OBBs.
    """
    def get_plane_normal(obb):
        # Use the direction of the smallest extent as the normal (usually thickness)
        extents = obb.extent
        R = obb.R
        smallest_axis_idx = np.argmin(extents)
        normal = R[:, smallest_axis_idx]
        return normal / np.linalg.norm(normal)

    def compute_overlap_2d(obb1, obb2, plane_normal):
        # Project the 3D OBB corners onto the 2D plane orthogonal to the normal
        def project_to_2d(obb, normal):
            # Build orthonormal basis on the plane
            arbitrary = np.array([1.0, 0.0, 0.0])
            if np.allclose(arbitrary, normal):
                arbitrary = np.array([0.0, 1.0, 0.0])
            tangent1 = np.cross(normal, arbitrary)
            tangent1 /= np.linalg.norm(tangent1)
            tangent2 = np.cross(normal, tangent1)

            corners = np.asarray(obb.get_box_points())
            proj = np.stack([
                np.dot(corners - obb.center, tangent1),
                np.dot(corners - obb.center, tangent2)
            ], axis=-1)
            return proj

        proj1 = project_to_2d(obb1, plane_normal)
        proj2 = project_to_2d(obb2, plane_normal)

        min1, max1 = np.min(proj1, axis=0), np.max(proj1, axis=0)
        min2, max2 = np.min(proj2, axis=0), np.max(proj2, axis=0)

        overlap = np.maximum(0, np.minimum(max1, max2) - np.maximum(min1, min2))
        overlap_area = overlap[0] * overlap[1]
        area1 = np.prod(max1 - min1)
        area2 = np.prod(max2 - min2)

        return overlap_area / min(area1, area2 + 1e-8)

    # Build adjacency matrix
    n = len(obbs)
    adjacency = np.zeros((n, n), dtype=bool)

    normals = [get_plane_normal(obb) for obb in obbs]
    centers = [obb.center for obb in obbs]

    for i, j in combinations(range(n), 2):
        ni, nj = normals[i], normals[j]
        ci, cj = centers[i], centers[j]

        # Cosine similarity of normals
        cos_angle = np.abs(np.dot(ni, nj))
        if cos_angle < normal_threshold:
            continue

        # Orthogonal gap
        gap = np.abs(np.dot(ci - cj, ni))
        if gap >= max_gap:
            continue

        # 2D Overlap
        overlap_ratio = compute_overlap_2d(obbs[i], obbs[j], ni)
        if overlap_ratio < overlap_threshold:
            continue

        adjacency[i, j] = True
        adjacency[j, i] = True

    # Connected components (graph-based)
    clusters = []
    visited = set()

    for i in range(n):
        if i not in visited:
            stack = [i]
            group = []
            while stack:
                node = stack.pop()
                if node not in visited:
                    visited.add(node)
                    group.append(node)
                    stack.extend(np.where(adjacency[node])[0])
            clusters.append(group)

    # Merge OBBs in clusters
    merged_obbs = []
    for cluster in clusters:
        if not cluster:
            continue
        all_points = []
        for idx in cluster:
            obb = obbs[idx]
            corners = obb.get_box_points()
            all_points.append(np.asarray(corners))
        merged_points = np.vstack(all_points)
        merged_pcd = o3d.geometry.PointCloud()
        merged_pcd.points = o3d.utility.Vector3dVector(merged_points)
        merged_obb = merged_pcd.get_oriented_bounding_box()
        merged_obbs.append(merged_obb)

    return merged_obbs