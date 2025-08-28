# -*- coding: utf-8 -*-
"""
# O'Reilly: 3D Data Science with Python
## Chapter 9 - 3D Shape Detection

General Information
* Created by: 🦊 Florent Poux. 
* Copyright: Florent Poux.
* License: MIT
* Status: Review Only (Confidential)

Dependencies:
* Anaconda or Miniconda
* An Anaconda new environment
* Libraries as described in the Chapter

Have fun with this Code Solution.

🎵 Note: Styling was not taken care of at this stage.

Enjoy!
"""

#%% 1. Importing libraries

#Base libraries
import numpy as np

#3D Library
import open3d as o3d

#%% 2. I/O + Centering + Viz'

def find_planes(pcd):

    

    center = pcd.get_center()
    pcd.translate(-center)

    # o3d.visualization.draw_geometries([pcd])

    #%% 3. Computing the average distance

    nn_distance = pcd.compute_nearest_neighbor_distance()

    #%% 4. Definition of the parameters

    #estimate parameters with this: https://github.com/plusk01/pointcloud-plane-segmentation

    p_radius = np.mean(nn_distance)*3.2
    p_normal_variance_threshold_deg = 50
    p_coplanarity_deg = 75
    p_outlier_ratio=0.6
    p_min_plane_edge_length=0
    p_min_num_points=0

    #%% 5. Estimate Normals (optional)

    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius = p_radius, max_nn=30))

    #%% 6. Combine RANSAC with Region Growing based on this paper: https://www.inf.ufrgs.br/~oliveira/pubs_files/RE/RE.html

    oboxes = pcd.detect_planar_patches(
        p_normal_variance_threshold_deg,
        p_coplanarity_deg,
        p_outlier_ratio,
        p_min_plane_edge_length,    
        p_min_num_points,
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30)
        )

    print("Detected {} patches".format(len(oboxes)))

    #%% 7. Generate 3D Oriented Bounding-Box Meshes for each planar segment

    p_obox = 0.5

    geometries = []
    obox_volumes = []

    for obox in oboxes:
        mesh = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(obox, scale=[1, 1, p_obox])
        mesh.paint_uniform_color(obox.color)
        mesh.compute_vertex_normals()
        # geometries.append(mesh)    
        obox_volumes.append(obox.volume())
        geometries.append(obox)
    
    return geometries,center

    #%% 8. Visualize the 3D results

    # o3d.visualization.draw_geometries(geometries+[pcd])

    # struct_vol = np.sum(obox_volumes)
    # print(f"Estimated Occupied Volume: {struct_vol} m3")