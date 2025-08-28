import numpy as np
import os
import sys
import subprocess
import argparse

from objects import *
from floor import *
from wall import *
from util import *
from visualize import *
from clustering import *
from holes import *
from precluster import *


def main2(e57_paths, points_proportion=None, simplified_path=None, values = None):
    """
    Recognizes floors and walls along the two major horizontal axes from a set of .e57 point cloud files.

    Args:
        e57_paths (list): A list of paths to .e57 files containing point cloud data from a building's scan.
        points_proportion (float, optional): The proportion of points to sample from the input files. Defaults to 0.001.
        simplified_path (str, optional): Path to save or load a simplified version of the point cloud for faster processing. Defaults to None.

    Returns:
        tuple: A tuple containing:
            - planes_obbs (list): The list of size-sorted Oriented Bounding Boxes (OBBs) for the detected planes.
            - clouds (tuple): A tuple of processed point clouds: The full cloud, extracted floor cloud, extracted wall cloud along first major vector, along second major vector and finally a size-sorted list of the recognized planes' pcds.
    """

    if simplified_path is None:
        print("Warning: No simplified path provided. It is recommanded to provide one so that the program can load faster next time.")

    # Extract the points clouds and store them in a numpy array
    # if os.path.isfile(simplified_path):  # If simplified version already exists, read it
    #     points_all = np.vstack(read_e57s([simplified_path], 1)).T
    # else:  # Else default to reading all the scans in paths (much longer)
    if points_proportion is None: points_proportion = 0.001
    x, y, z, col = read_e57s([e57_paths], points_proportion)
    points_all = np.vstack([x, y, z]).T


    # Initialize the main points cloud
    pcd_all = init_pcd2(points_all, col)
    if not os.path.isfile(simplified_path):  # Save extracted points cloud so that loading is faster next time
        print("Saving simplified points cloud...")
        save_points_cloud(pcd_all, simplified_path)

    if values:
        clutstered_pcds = cluster_large_objects(pcd_all, values[0],values[1],values[2])
    else:
        clutstered_pcds = cluster_large_objects(pcd_all)
    return clutstered_pcds
    # # Extract the floor cloud
    # floor_cloud = extract_floor_cloud(pcd_all)

    # # Extract the wall cloud
    # wall_cloud = extract_wall_cloud(pcd_all, 0.1)

    # # Find the two major vectors of the building's walls
    # major_vectors, major_angles = find_major_vectors(wall_cloud, plot=False)

    # # Use those two vectors to extract the points cloud of the two major wall kinds
    # pcd_mv1, pcd_mv2 = extract_major_vectors_wall_clouds(wall_cloud, major_vectors)  # mvi for Major Vector i

    # # Segment each points cloud into planes
    # print("Segmenting the points clouds into planes...")
    # top_walls_mv1 = find_planes(pcd_mv1, assign_colors=True)
    # top_walls_mv2 = find_planes(pcd_mv2, assign_colors=True)
    # top_floors = find_planes(floor_cloud, assign_colors=True)

    # # Paint floors in blue, walls along mv1 in red and walls along mv2 in green for visualization (Comment code to see a different color for each plane)
    # paint_planes(top_walls_mv1, [1, 0, 0])
    # paint_planes(top_walls_mv2, [0, 1, 0])
    # paint_planes(top_floors, [0, 0, 1])

    # # Clustering step

    

    # #walls_mv1_holes = cluster_point_clouds(walls_mv1_holes)
    # #walls_mv2_holes = cluster_point_clouds(walls_mv2_holes)
    # #floors_holes = cluster_point_clouds(floors_holes)

    # top_walls_mv1 = cluster_point_clouds(top_walls_mv1)
    # top_walls_mv2 = cluster_point_clouds(top_walls_mv2)
    # top_floors = cluster_point_clouds(top_floors)

    # walls_mv1_holes = find_holes(top_walls_mv1, "wall_mv1")
    # walls_mv2_holes = find_holes(top_walls_mv2, "wall_mv2")
    # floors_holes = find_holes(top_floors, "floor")

    # #holes_labels = ["wall_mv1"] * len(walls_mv1_holes) + ["wall_mv2"] * len(walls_mv2_holes) + ["floor"] * len(floors_holes)
    # #holes = walls_mv1_holes + walls_mv2_holes + floors_holes

    # # Sort the top planes by amount of points (The larger they are, the more likely they are to be actual floors or walls)
    # print("Sorting the planes by number of points...")
    # top_planes = top_walls_mv1 + top_walls_mv2 + top_floors
    # top_planes_labels = ["wall_mv1"] * len(top_walls_mv1) + ["wall_mv2"] * len(top_walls_mv2) + ["floor"] * len(top_floors)  # Keep a record of each plane's type
    # top_planes, top_planes_labels = sort_pcd_list_by_size(top_planes, top_planes_labels)


    # # Eventual position of clustering step if clustering before filtering largest planes is incoherent

    
    # # Compute an obb for each plane
    # planes_obbs = get_obbs_from_planes(top_planes, top_planes_labels, major_angles)
    # #holes_obb = get_obbs_from_planes(holes, holes_labels, major_angles)

    # #visualize(holes_obb, holes)
    # # Writing the labels down in a text file for future use
    # file = open("labels.txt","w")
    # for label in top_planes_labels:
    #     file.write(label+"\n")
    # file.close()

    return point_clouds


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Recognize floors and walls from .e57 point cloud files.")
    parser.add_argument("e57_paths", nargs="+", help="Paths to .e57 files containing point cloud data.")
    parser.add_argument("--output_path", default="output", help="Path to save the .obj output file. Defaults to 'output'.")
    parser.add_argument("--simplified_path", default=None, help="Path to save/load a simplified version of the point cloud.")
    parser.add_argument("--points_proportion", type=float, default=0.001, help="Proportion of points to sample from the input files. Defaults to 0.001.")

    args = parser.parse_args()

    # Recognize the planes from the points clouds.
    clustered_point_clouds = main2(
        e57_paths=args.e57_paths,
        points_proportion=args.points_proportion,
        simplified_path=args.simplified_path
    )

    num_clusters = len(clustered_point_clouds)
    print(f"\nFound {num_clusters} clusters.")
    for idx, pcd in enumerate(clustered_point_clouds):
        print(f"Cluster {idx+1}: {len(pcd.points)} points")

    # Ask user which clusters to keep
    input_str = input("\nEnter IDs of clusters to keep (format: 'X,Y,Z,...'): ").strip()

    # Parse input safely
    try:
        selected_ids = [int(s)-1 for s in input_str.split(",") if s.strip().isdigit()]
    except ValueError:
        print("Invalid input format.")
        exit

    # Filter list
    point_cloud_list = [
        clustered_point_clouds[i] for i in selected_ids if 0 <= i < num_clusters
    ]

    print(f"\nSelected {len(point_cloud_list)} clusters.")


    if not point_cloud_list:
        print("Warning: Empty point cloud list provided.")
        exit

    # Optionally assign a unique color to each point cloud for visualization
    colors = [
        [1, 0, 0], [0, 1, 0], [0, 0, 1], 
        [1, 1, 0], [1, 0, 1], [0, 1, 1],
        [0.5, 0.5, 0.5], [1, 0.5, 0], [0, 0.5, 1]
    ]
    for idx, pcd in enumerate(point_cloud_list):
        if not pcd.has_colors():
            color = colors[idx % len(colors)]
            pcd.paint_uniform_color(color)

    # Show them all together
    o3d.visualization.draw_geometries(point_cloud_list, window_name="preclustered items")

    # # Export the recognized objects to an .obj file.
    # export_objects(top_planes_obbs, output=(args.output_path+".obj"))

    # # Call the blender script that is responsible for the creation of the .ifc file.

    # # set the parameters.
    # blend_file = "temp.blend"
    # blender_script = "IFCexport.py"
    # blender_exe = r"C:\Program Files\Blender Foundation\Blender 4.4\blender.exe" 

    # # write down the command.
    # args2 = [
    #     blender_exe,
    #     "--background",  # no UI
    #     blend_file,
    #     "--python", blender_script, "--", 
    #     args.output_path
    #     ]
    
    # export_path = args.output_path + ".ifc"
    # # Delete the file if it exists to avoid redundant data
    # try:
    #     os.remove(export_path)
    #     print(f"File '{export_path}' deleted successfully")
    # except FileNotFoundError:
    #     print(f"File '{export_path}' not found")
    # except PermissionError:
    #     print(f"Permission denied to delete '{export_path}'")
    # except OSError as e:
    #     print(f"Error deleting file: {e}")

    # export_path = "temp.blend1"

    # try:
    #     os.remove(export_path)
    #     print(f"File '{export_path}' deleted successfully")
    # except FileNotFoundError:
    #     print(f"File '{export_path}' not found")
    # except PermissionError:
    #     print(f"Permission denied to delete '{export_path}'")
    # except OSError as e:
    #     print(f"Error deleting file: {e}")

    # # execute the command to call the blender script with the correct arguments.
    # subprocess.run(args2, check=True)