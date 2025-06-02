import numpy as np


# Extracts the points with a normal vector whose z component is higher than z_threshold
def extract_floor_cloud(pcd, z_threshold=0.999):
    print("Extracting floor cloud...")

    # Convert normals to numpy array
    normals = np.asarray(pcd.normals)

    # Extract points whose normals have a strong Z component
    mask = np.abs(normals[:, 2]) > z_threshold
    floor_indices = np.where(mask)[0]
    floor_cloud = pcd.select_by_index(floor_indices)

    return floor_cloud
