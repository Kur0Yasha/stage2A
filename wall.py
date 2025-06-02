import numpy as np
from scipy.signal import find_peaks, peak_prominences
import matplotlib.pyplot as plt
import open3d as o3d


# Extracts the points with a normal vector whose z component is lower than z_tolerance
def extract_wall_cloud(pcd, z_tolerance=0.001):
    print("Extracting wall cloud...")

    # Extract normals from pcd in a numpy array
    normals = np.asarray(pcd.normals)

    # Extract points whose normals have a low Z component
    mask = np.abs(normals[:, 2]) < z_tolerance
    wall_indices = np.where(mask)[0]
    wall_cloud = pcd.select_by_index(wall_indices)

    return wall_cloud


# Find the two major normal vectors
def find_major_vectors(pcd, plot=False):
    print("Finding major vectors...")

    # Extract pcd normals in a numpy array
    normals = np.asarray(pcd.normals)

    # Normalize just in case
    filtered_normals = normals / np.linalg.norm(normals, axis=1, keepdims=True)

    # Sweep alpha from 0 to pi in n values
    n = 1000
    alphas = np.linspace(0, np.pi, n)[:-1]  # Remove last value to avoid finding twice the same major vector

    # Keep the record of how many vectors are close to each angle
    counts = []

    # Define angular tolerance in radians
    angle_tolerance = np.deg2rad(180 / n)  # Adjusted depending on n
    cos_tolerance = np.cos(angle_tolerance)

    # Check count of similar normal vectors for each possible angle
    for alpha in alphas:
        # Reference vector in the (x, y) plane
        ref_vector = np.array([np.cos(alpha), np.sin(alpha), 0])

        # Compute dot product with all normals (cosine of angle since vectors are normalized)
        dots = filtered_normals @ ref_vector

        # Count normals that are within the angle tolerance (cosine close to ±1)
        count = np.sum(np.abs(dots) >= cos_tolerance)  # ± for colinearity in either direction
        counts.append(count)

    # Find peaks
    peaks, _ = find_peaks(counts)

    # Compute prominence to identify major peaks
    prominences = peak_prominences(counts, peaks)[0]

    # Sort peaks by prominence
    sorted_indices = np.argsort(prominences)[-2:][::-1]  # get top 2 peaks
    major_peaks = peaks[sorted_indices]
    major_alphas = alphas[major_peaks]

    major_vectors = np.array([np.cos(major_alphas), np.sin(major_alphas), np.zeros_like(major_alphas)]).T

    if plot:
        # Plotting
        plt.figure(figsize=(10, 5))
        plt.plot(alphas, counts, label='Number of colinear normals')
        plt.xlabel('Alpha (radians)')
        plt.ylabel('Count of colinear normals')
        plt.title('Colinearity of Normals vs. (cos(α), sin(α), 0)')
        plt.grid(True)
        plt.legend()
        plt.show()

        print(f"Major Vectors found (coordinates): {major_vectors}\nMajor Alphas found (radians): {major_alphas}")

    return major_vectors, major_alphas


# Extract the two points clouds, with points whose normals have a dot product (with each major vector) greater than min_similarity.
def extract_major_vectors_wall_clouds(wall_pcd, major_vectors, min_similarity=0.99):
    print("Extracting two wall clouds form the wall cloud, knowing the two major vectors...")

    # Extract normals from pcd in numpy array
    points = np.asarray(wall_pcd.points)
    normals = np.asarray(wall_pcd.normals)

    vector1, vector2 = major_vectors  # Two major vectors

    # Compute the dot product between normals and major_vectors, then filter indices
    dot_products_vector_1 = np.abs(normals @ vector1)
    indices_vector1 = np.where(dot_products_vector_1 > min_similarity)[0]

    dot_products_vector_2 = np.abs(normals @ vector2)
    indices_vector2 = np.where(dot_products_vector_2 > min_similarity)[0]

    # Rebuild points clouds
    pcd_mv1 = o3d.geometry.PointCloud()
    pcd_mv1.points = o3d.utility.Vector3dVector(points[indices_vector1])

    pcd_mv2 = o3d.geometry.PointCloud()
    pcd_mv2.points = o3d.utility.Vector3dVector(points[indices_vector2])

    return pcd_mv1, pcd_mv2
