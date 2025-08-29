import numpy as np
import pye57
import open3d as o3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import glob
import os


# Extracts points (and colors) uniformly from a list of e57 files
def read_e57s(paths, pointsPercentage=0.001):
    x, y, z = [], [], []
    r, g, b = [], [], []

    e57s = [pye57.E57(path, mode="r") for path in paths]

    for e57file_i, e57 in enumerate(e57s):
        for scan_i in range(e57.scan_count):
            print(f"Reading scan {scan_i + 1} / {e57.scan_count} of file {e57file_i + 1} / {len(e57s)}...")
            try:
                data = e57.read_scan(scan_i, ignore_missing_fields=True)
            except pye57.libe57.E57Exception:
                print(f"Warning: Could not read scan {scan_i} due to missing pose information. Skipping...")
                continue

            if not all(isinstance(data.get(f"cartesian{axis}"), np.ndarray) for axis in "XYZ"):
                print(f"Warning: Missing coordinate data in scan {scan_i}. Skipping...")
                continue

            print(f"Found {len(data['cartesianX'])} points.")
            pointsInFile = len(data["cartesianX"])
            points = int(pointsInFile * pointsPercentage)
            pas = max(1, pointsInFile // points)

            # Check if color fields exist (E57 stores colors as uint16: 0–65535)
            has_color = all(field in data for field in ("colorRed", "colorGreen", "colorBlue"))

            for i in range(points):
                idx = i * pas
                if idx < pointsInFile:
                    x.append(data["cartesianX"][idx])
                    y.append(data["cartesianY"][idx])
                    z.append(data["cartesianZ"][idx])

                    if has_color:
                        r.append(data["colorRed"][idx] / 65535.0)
                        g.append(data["colorGreen"][idx] / 65535.0)
                        b.append(data["colorBlue"][idx] / 65535.0)
                    else:
                        r.append(1.0)
                        g.append(0.0)
                        b.append(0.0)  # default red if no color

            print(f"{points} points added.\n")

    if not x:
        raise ValueError("No valid point cloud data could be read from the provided E57 files")

    x, y, z = np.array(x), np.array(y), np.array(z)
    colors = np.vstack((np.array(r), np.array(g), np.array(b))).T
    return x, y, z, colors


def save_points_cloud(pcd, output_path):
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)

    if len(points) == 0:
        raise ValueError("Point cloud is empty. Cannot save to .e57 format.")
    output_path = os.path.abspath(output_path)
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    writer = pye57.E57(output_path, mode="w")

    # Convert colors to uint16 (E57 expects this range)
    color_uint16 = np.clip(colors * 65535, 0, 65535).astype(np.uint16)

    writer.write_scan_raw(
        {
            "cartesianX": points[:, 0],
            "cartesianY": points[:, 1],
            "cartesianZ": points[:, 2],
            "colorRed": color_uint16[:, 0],
            "colorGreen": color_uint16[:, 1],
            "colorBlue": color_uint16[:, 2],
        }
    )
    writer.close()


def init_pcd(points, colors=None):
    print("Initializing points cloud...")

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    if colors is not None:
        pcd.colors = o3d.utility.Vector3dVector(colors)
    else:
        pcd.paint_uniform_color([1, 0, 0])  # Red if no colors

    # Downsample
    pcd = pcd.voxel_down_sample(voxel_size=0.05)

    # Remove outliers
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    pcd = pcd.select_by_index(ind)

    # Normals
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=50))
    if not pcd.has_normals():
        raise RuntimeError("Failed to estimate normals for the point cloud.")
    pcd.orient_normals_consistent_tangent_plane(k=50)

    return pcd


def init_pcd2(points, colors=None):
    print("Initializing points cloud...")

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    if colors is not None:
        pcd.colors = o3d.utility.Vector3dVector(colors)
    else:
        pcd.paint_uniform_color([1, 0, 0])  # Red if no colors

    # Downsample
    pcd = pcd.voxel_down_sample(voxel_size=0.05)

    # Remove outliers
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    pcd = pcd.select_by_index(ind)


    return pcd

def sort_pcd_list_by_size(clouds, top_planes_labels=None):
    if top_planes_labels is None:
        top_planes_labels = [None] * len(clouds)
    sorted_pairs = sorted(zip(clouds, top_planes_labels), key=lambda pair: len(np.asarray(pair[0].points)), reverse=True)
    sorted_clouds, sorted_labels = zip(*sorted_pairs)
    if top_planes_labels[0] is None:
        return list(sorted_clouds)
    else:
        return list(sorted_clouds), list(sorted_labels)


# RANSAC distance threshold for plane detection
def find_planes(pcd, ransac_distance_threshold=0.01, assign_colors=True):
    # Color map for planes (cycle through colors)
    colors = plt.get_cmap("tab10").colors

    # Copy the original cloud to modify
    remaining_cloud = pcd  # Start with the entire cloud

    min_points_threshold = 200  # Don't take into account planes with less than this amount of points

    top_planes = []  # List to store planes

    while len(np.asarray(remaining_cloud.points)) > min_points_threshold:
        # Apply RANSAC to detect a plane
        plane_model, inliers = remaining_cloud.segment_plane(
            distance_threshold=ransac_distance_threshold, ransac_n=3, num_iterations=1000
        )

        # Extract the plane points
        plane = remaining_cloud.select_by_index(inliers)

        if len(inliers) > min_points_threshold:
            # Apply Statistical Outlier Removal (SOR) to remove distant outliers
            plane, _ = plane.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.)

            # Paint plane
            if assign_colors:
                plane.paint_uniform_color(colors[len(top_planes) % len(colors)])

            # Store detected plane
            top_planes.append(plane)

        # Remove detected surface (whether it’s a valid plane or too small)
        remaining_cloud = remaining_cloud.select_by_index(inliers, invert=True)

    # Sort planes by number of points (largest first)
    top_planes = sort_pcd_list_by_size(top_planes)

    return top_planes


def show_planes(planes, assign_colors=True):
    # Define color map for planes (cycle through colors)
    colors = plt.get_cmap("tab10").colors

    if assign_colors:
        # Assign a unique color to each plane
        for i, plane in enumerate(planes):
            plane.paint_uniform_color(colors[i % len(colors)])

    # Show all planes
    o3d.visualization.draw_geometries(planes)


def paint_planes(planes, color=None):
    for plane in planes:
        plane.paint_uniform_color(color if color is not None else np.random.rand(3))


def visualize_point_cloud(x, y, z, point_size=0.1, title="E57 Point Cloud Visualization"):
    """
    Creates a dynamic 3D visualization of point cloud data from an E57 file.
    
    Args:
        x, y, z (numpy arrays): Point coordinates returned by read_e57s()
        point_size (float): Size of each point in the visualization
        title (str): Title for the visualization window
    """
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Create the scatter plot
    ax.scatter(x, y, z, s=point_size, c=z, cmap='viridis', marker='.')
    
    # Set labels and title
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.set_title(title)
    
    # Enable interactive rotation/zooming
    plt.tight_layout()
    plt.show()

def get_e57_paths(folder_path):
    """Returns a list of all .e57 file paths in the given folder."""
    return glob.glob(os.path.join(folder_path, "*.e57"))

if __name__ == "__main__":
    x, y, z = read_e57s(get_e57_paths("Canford School E57 files"), 0.001)
    visualize_point_cloud(x, y, z)
