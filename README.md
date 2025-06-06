# PIDR-G13 *- Automated Recognition of Structural Elements from Point Cloud Scans*


## Main Objectives

The main objective of this project is to write a program that
takes a point cloud representing a building as an input,
and returns a 3D model of the scanned building.
Due to time constraints, the extent of the project is limited to
specifically recognize floors and walls.

For practical usage, we decided to also focus on the program's
complexity. Since this process can also be done by humans,
the objective is to have a program that runs fast enough
so that it could replace the currently needed human action.

The program is written in Python, can read a list of .e57
files, and returns a reconstruction of
detected floors and walls as an .obj file.

## Installing & Running

The project works with ***Python 3.11***.9 since the bpy
library isn't ported to later versions at the moment.

Python 3.11.9: https://www.python.org/downloads/release/python-3119/

The programm needs the 4.4 version of blender as well as the Bonsai (Formerly BlenderBIM) addon to run.

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

To run the program, run `main.py` and pass your desired arguments.

```bash
python main.py [.e57 path] --output_path [output path] --simplified_path [simplified .e57 path] --points_proportion 0.001 --n 42
```

## Details of the detection process

Most of the code is largely commented to hopefully help you
use the program. You can always find more details on the
recognition process in our [research paper](ResearchPaper.pdf).

There are currently multiple useable .e57 datasets which can be found [here](https://drive.google.com/file/d/1mieE06ak5C0Pzx_eCouriF02A7h-f9wP/view?usp=sharing).

Please note that, as of right now, previous .ifc files need to be deleted for blender to export a correct .ifc file if you run the program again.

Additionaly, if you wish to change the sampling value when opening an .e57 file while giving an existing simplified path, you first need to delete the simplified path as it takes priority over any kind of sampling.