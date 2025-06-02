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
from IFCexport import *


def main(e57_paths, points_proportion=None, simplified_path=None):
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
    if os.path.isfile(simplified_path):  # If simplified version already exists, read it
        points_all = np.vstack(read_e57s([simplified_path], 1)).T
    else:  # Else default to reading all the scans in paths (much longer)
        if points_proportion is None: points_proportion = 0.001
        points_all = np.vstack(read_e57s(e57_paths, points_proportion)).T

    # Initialize the main points cloud
    pcd_all = init_pcd(points_all)
    if not os.path.isfile(simplified_path):  # Save extracted points cloud so that loading is faster next time
        print("Saving simplified points cloud...")
        save_points_cloud(pcd_all, simplified_path)

    # Extract the floor cloud
    floor_cloud = extract_floor_cloud(pcd_all)

    # Extract the wall cloud
    wall_cloud = extract_wall_cloud(pcd_all, 0.1)

    # Find the two major vectors of the building's walls
    major_vectors, major_angles = find_major_vectors(wall_cloud, plot=False)

    # Use those two vectors to extract the points cloud of the two major wall kinds
    pcd_mv1, pcd_mv2 = extract_major_vectors_wall_clouds(wall_cloud, major_vectors)  # mvi for Major Vector i

    # Segment each points cloud into planes
    print("Segmenting the points clouds into planes...")
    top_walls_mv1 = find_planes(pcd_mv1, assign_colors=True)
    top_walls_mv2 = find_planes(pcd_mv2, assign_colors=True)
    top_floors = find_planes(floor_cloud, assign_colors=True)
    # Paint floors in blue, walls along mv1 in red and walls along mv2 in green for visualization (Comment code to see a different color for each plane)
    paint_planes(top_walls_mv1, [1, 0, 0])
    paint_planes(top_walls_mv2, [0, 1, 0])
    paint_planes(top_floors, [0, 0, 1])

    # Sort the top planes by amount of points (The larger they are, the more likely they are to be actual floors or walls)
    print("Sorting the planes by number of points...")
    top_planes = top_walls_mv1 + top_walls_mv2 + top_floors
    top_planes_labels = ["wall_mv1"] * len(top_walls_mv1) + ["wall_mv2"] * len(top_walls_mv2) + ["floor"] * len(top_floors)  # Keep a record of each plane's type
    top_planes, top_planes_labels = sort_pcd_list_by_size(top_planes, top_planes_labels)

    # Compute an obb for each plane
    planes_obbs = get_obbs_from_planes(top_planes, top_planes_labels, major_angles)

    # Writing the labels down in a text file for future use
    file = open("labels.txt","w")
    for label in top_planes_labels:
        file.write(label+"\n")
    file.close()

    return planes_obbs, (pcd_all, floor_cloud, pcd_mv1, pcd_mv2, top_planes)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Recognize floors and walls from .e57 point cloud files.")
    parser.add_argument("e57_paths", nargs="+", help="Paths to .e57 files containing point cloud data.")
    parser.add_argument("--output_path", default="output", help="Path to save the .obj output file. Defaults to 'output'.")
    parser.add_argument("--simplified_path", default=None, help="Path to save/load a simplified version of the point cloud.")
    parser.add_argument("--points_proportion", type=float, default=0.001, help="Proportion of points to sample from the input files. Defaults to 0.001.")
    parser.add_argument("--n", type=int, default=42, help="Number of top planes to keep based on size. Defaults to 42.")

    args = parser.parse_args()

    # Recognize the planes from the points clouds.
    planes_obbs, (pcd_all, floor_cloud, pcd_mv1, pcd_mv2, planes) = main(
        e57_paths=args.e57_paths,
        points_proportion=args.points_proportion,
        simplified_path=args.simplified_path
    )

    # Keep the first n planes by amount of points.
    # You can debug here to find the desired n for your building.
    top_planes, top_planes_obbs = planes[:args.n], planes_obbs[:args.n]

    # View a comparison between the reconstructed planes and their points clouds.
    visualize(top_planes_obbs, top_planes)

    # Export the recognized objects to an .obj file.
    export_objects(top_planes_obbs, output=(args.output_path+".obj"))

    # Call the blender script that is responsible for the creation of the .ifc file.

    # set the parameters.
    blend_file = "temp.blend"
    output_ifc = "exported.ifc"
    blender_script = "IFCexport.py"
    blender_exe = r"C:\Program Files\Blender Foundation\Blender 4.4\blender.exe" 

    # write down the command.
    args = [
        blender_exe,
        "--background",  # no UI
        blend_file,
        "--python", blender_script, "--", 
        output_ifc
        ]

    # execute the command to call the blender script with the correct arguments.
    # subprocess.run(args, check=True)