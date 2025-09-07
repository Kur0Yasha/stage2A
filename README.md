# PIDR-G13 *- Automated Recognition of Structural Elements from Point Cloud Scans*


## Main Objectives

The main objective of this project is to write a program that
takes a point cloud representing a building as an input,
and returns a 3D model of the scanned building.

For practical usage, we decided to also focus on the program's
complexity. Since this process can also be done by humans,
the objective is to have a program that runs fast enough
so that it could replace the currently needed human action.

The program is written in Python, can read a list of .e57
files, and returns a reconstruction of
detected floors and walls as an .ifc file.

## Installing & Running

The project works with ***Python 3.11***.9 since the bpy
library isn't ported to later versions at the moment.

Python 3.11.9: https://www.python.org/downloads/release/python-3119/

The program needs the 4.4 version of blender as well as the Bonsai (Formerly BlenderBIM) addon to run.

Blender: https://www.blender.org/download/
Bonsai (BlenderBIM): https://bonsaibim.org/download.html

To install the project, you first need to clone the repository:

```bash
cd yourPath/scan2BIM
git clone git@gibson.telecomnancy.univ-lorraine.fr:eliott.sentenac/pidrg13.git
```

Then, install the requirements:

```bash
pip install -r requirements.txt
```

To run the program, run `interface.py` to open the interface from which you can use the implemented features.

As of right now, not all features of shown in the interface have been implemented and some are here as placeholder.

Those features are meant to be implemented in case further work is made on this project.

The currently implemented features includes : 

The possibility to load an .e57 file, the path has to be typed in the textbox at the top.

Multiple variables can be changed when loading a file, here are the details :

Sampling proportion : The quantity of point you wish to load, ranging from 0 to 1, 1 will load all points. For exemple, if you want to load only 1% of the total points, that value has to be set at 0.01.

eps : Epsilon parameter for the pre-clustering ran on the file when loaded to separate the large objects. It is advised not to change it.

Min samples : Minimal neighboor parameter for the pre-clustering. It is advised not to change it.

Min point per cluster : Minimal number of points that has to be contained in a detected cluster during pre-clustering for it to be considered.


When a file is loaded, it is first separated in large clusters, before being further separated in smaller cluster each representing a structural element or a part of it. Each one of these elements is projected in a 2D plan as an image in `output_images`. To add context, nearby elements are shown in purple, while the main element is shown in red. There are multiple angles for each item mentioned in the picture's name.

T stands for top view.
X stands for perspective (45°) view alongside the X axis.
Y stands for perspective view alongside the Y axis.

Additional views can easily be added with 2 lines in the `save_pointcloud_views` function in the `clusterpictures.py` file.

The large objects can be selected to be enabled or not when running further process via checkboxes once the file is loaded.

The large objects can be visualized as point clouds by clicking on `Visualize`.

The expected reconstruction of the planes can be visualized by clicking on `Visualize reconstruction`.



The partially deprecated, placeholder and work in progress features includes : 

The option to export the file's reconstruction by inputting an input path in the textbox and clicking on `Export reconstruction`.

The reconstruction algorithm is implemented, but due to changes made to the format of data for the latest attempt to find a fitting reconstruction, the previously implemented method needs to be adapted slightly.

Lastly, a table that allows for sorting and managing elements is available although currently unused.

The features includes : 

- Displaying sorted elements under specified categories.
- Naming each element and giving it a category.
- Changing an element's category.
- Changing a category's color (used for visualization).
- Enabling or not the use of the element, category or all elements for further process, which allows for flexibility according to the client's needs.

All functions to add, remove and access elements and their properties are already implemented.

## Utility funtions 

The following section contains details on the implemented utility function, including their name, file of origin, purpose and parameters.

First, two files do not contain any functions as they are meant to be called as blender macros.

To run a Blender macro automatically, use the `subprocess.run(command, check=True)` function, with command following this structure : 

`[blender_exe,"--background", blend_file, "--python", blender_script, "--", arguments]`

- blender_exe should be a string that correspond to the path to your blender installation.
- "--background" allows for the process to be run without opening a Blender window.
- blend_file should be a string that correspond to the path to a placeholder blender file. There is already such a file being called temp.blend in the root of the project. When using it make sure not to use it for multiple actions at once, and make sure to delete the temp files created after use to avoid any issues for further use of it.
- "--python" corresponds to the language in which the macro is made.
- blender_script should be a string that correspond to the path to the .py file to execute in Blender.
- arguments corresponds to the arguments to be given to the ran macro.

Custome arguments can be handled that way within the macro : 

```
argv = sys.argv
if "--" in argv:
    idx = argv.index("--")
    custom_args = argv[idx + 1:]
else:
    custom_args = []
```

The files in question are : 

`substraction.py`, which can substract two .obj files named positive.obj and negative.obj which are automatically created within other export phases.

`IFCexport.py`, which converts a .obj file given in argument as well as the details regarding the semantics given in a text file named labels.txt.

Both can also be given an export path as argument.

For the utility functions, the list will be formated as such :

(File name) function_name(arguments) : return_value

"""Description"""

(util.py) read_e57s(paths, point_percentage = 0.001) : x, y, z, colors

"""
This function takes in argument a string that is the path to the file to open, as well as a float ranging from 0 to 1 that is the sampling desnity wanted.

It returns three lists of floats being respectively the x, y and z coordinates of the points, as well as a list of list of ints being their color.
"""

(util.py) save_points_cloud(pcd, output_path): None

"""
This function allows to save an open 3D point cloud (pcd) as an .e57 file under the output_path path.
"""

(util.py) init_pcd(points, colors = None): pcd

"""
This function takes as argument points, which is a tuple made of the previously obtained three lists of coordinates, as well as eventually the list of colors.
It returns an open3D point cloud after applying preprocessing and calculating normals.
"""

(util.py) init_pcd2(points, colors = None): pcd

"""
This function takes as argument points, which is a tuple made of the previously obtained three lists of coordinates, as well as eventually the list of colors.
It returns an open3D point cloud after applying preprocessing but does not calculate the normals.
"""

(planardetec.py) find_planes(pcd): geometries, center

"""
This function takes in argument an open3D point clouds.
It returns the oriented bounding boxes matching the planes found within the point cloud, as well as the center of the point cloud used to deal with issues regarding offset during visualisation.
"""

(precluster.py) cluster_large_objects(point_cloud, eps=0.1, min_samples=4, min_points_per_cluster=1000): clusters

"""
This functtion takes in arguments an open3D point cloud as well as the arguments required for it, which contains default values.
It returns a list of open3D point clouds corresponding to the large clusters found.
"""

(objects.py) get_min_volume_box(points): obb

"""
This function takes in argument an open3D point cloud.
It returns an open3D oriented bounding box that correspond to the minimal regular shape fitting the point cloud whole.
"""

(objects.py) export_objects(obbs, output="my_mesh.obj"): None

"""
This function takes in argument a list of open3D oriented bounding boxes and eventually an export path.
It exports the reconstruction of the obbs as a .obj file under the given path.
"""

(main2.py) main2(e57_paths, points_proportion=None, simplified_path=None, values = None): pcds

"""
This function takes in argument the path to an .e57 file, and eventually other arguments.
It returns a list of point clounds that corresponds to the large objects contained within the file given as arguments.
"""

(clusterpictures.py) save_pointcloud_views(pcd, index, output_dir="output_images"): None

"""
This function takes in argument an open3D point cloud, as well as an interger called index meant to differenciate point cloud and eventually a folder to export the images.
It exports different viewpoints of the given point cloud as projected images.
"""

(interface.py) obb_to_pcd(obbs, voxel_size=0.01): pcds

"""
This function takes in argument a list of open3D oriented bounding boxes, and a voxel size that corresponds to the density of the created point clouds.
It returns a list of open3D point clouds corresponding to the obbs filled uniformously using the given voxel size.
"""

(interface.py) concatenate(pcds): pcd

"""
This function takes in argument a list of open3D point clounds and returns a single open3D point cloud that is the concatenation of them all.
"""

Multiple functions are also contained within the class used for the interface.

These includes the functions to manipulate the grid in the interface which simply require the element to be given as argument.

## Further details

There are currently multiple useable .e57 datasets which can be found [here](https://drive.google.com/drive/folders/1Em_lOaIjXhHR3UcF3qaC_Z7ydC2Ca18b?usp=sharing).

If further work on this project has to be done, either request for acces to this repository, fork, or clone and push into a new repository.

## Inquiries and reports

If there are missing informations within the documentation, or issues comming from the implemented code, you may contact me through the following email adress :

Eliott.Sentenac@telecomnancy.eu