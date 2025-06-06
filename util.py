import numpy as np
import pye57
import open3d as o3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import glob
import os


# Extracts points uniformally from a list of e57 files, with a density of points equal to pointsPercentage
def read_e57s(paths, pointsPercentage=0.001):
    x = []
    y = []
    z = []

    e57s = [pye57.E57(path, mode="r") for path in paths]

    for e57file_i, e57 in enumerate(e57s):
        for scan_i in range(e57.scan_count):
            print(f"Reading scan {scan_i + 1} / {e57.scan_count} of file {e57file_i + 1} / {len(e57s)}...")
            try:
                data = e57.read_scan(scan_i, ignore_missing_fields=True)
            except pye57.libe57.E57Exception as e:
                print(f"Warning: Could not read scan {scan_i} due to missing pose information. Skipping...")
                continue

            # Ensure all 3 coordinates are listed
            if not all(isinstance(data.get(f"cartesian{axis}"), np.ndarray) for axis in "XYZ"):
                print(f"Warning: Missing coordinate data in scan {scan_i}. Skipping...")
                continue

            print(f"Found {len(data['cartesianX'])} points.")
            pointsInFile = len(data["cartesianX"])
            points = int(pointsInFile * pointsPercentage)
            pas = max(1, pointsInFile // points)  # Ensure pas is at least 1

            for i in range(points):
                if i * pas < pointsInFile:  # Ensure we don't go out of bounds
                    x.append(data["cartesianX"][i * pas])
                    y.append(data["cartesianY"][i * pas])
                    z.append(data["cartesianZ"][i * pas])

            print(f"{points} points added.\n")

    if not x:  # If no points were read
        raise ValueError("No valid point cloud data could be read from the provided E57 files")

    x, y, z = np.array(x), np.array(y), np.array(z)
    return x, y, z


def save_points_cloud(pcd, output_path):
    # Extract points from the Open3D point cloud
    points = np.asarray(pcd.points)

    # Ensure there are points to write
    if len(points) == 0:
        raise ValueError("Point cloud is empty. Cannot save to .e57 format.")

    # Create an E57 writer
    writer = pye57.E57(output_path, mode="w")

    # Write the point cloud data
    writer.write_scan_raw(
        {
            "cartesianX": points[:, 0],
            "cartesianY": points[:, 1],
            "cartesianZ": points[:, 2],
        }
    )
    writer.close()


# Creates points cloud object and applies multiple preprocessing operations
def init_pcd(points):
    print("Initializing points cloud...")

    # Create an Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # Downsample using voxel grid filtering
    pcd = pcd.voxel_down_sample(voxel_size=0.05)

    # Remove statistical outliers
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    pcd = pcd.select_by_index(ind)

    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=50))
    # Ensure normals are estimated
    if not pcd.has_normals():
        raise RuntimeError("Failed to estimate normals for the point cloud.")
    pcd.orient_normals_consistent_tangent_plane(k=50)  # Ensure normals face same direction

    # Paint cloud in uniform color
    pcd.paint_uniform_color([1, 0, 0])  # Red

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
