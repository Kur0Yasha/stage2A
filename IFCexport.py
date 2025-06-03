# Script meant to be run using blender as it requires the use of blenderBIM addon, which can only be used inside of Blender.

import sys
import bpy

# Get custom args after "--"
argv = sys.argv
if "--" in argv:
    idx = argv.index("--")
    custom_args = argv[idx + 1:]
else:
    custom_args = []

# Extract the output path
if len(custom_args) < 1:
    raise Exception("Expected output IFC path")
output_path = custom_args[0]

obj_path = output_path+".obj"
label_path = "labels.txt"



# read classification from label file

with open(label_path, 'r') as f:
    class_assignments = [line.strip() for line in f.readlines()]

# import objects from .obj files

bpy.ops.wm.obj_import(filepath=obj_path)

# Get all imported objects (excluding cameras, lights, etc.)
imported_objects = [obj for obj in bpy.context.scene.objects if obj.type == 'MESH']
print(f"Imported {len(imported_objects)} objects")

# Start a new IFC project
bpy.ops.bim.create_project()

# assign ifc classes

for obj, ifc_class in zip(imported_objects, class_assignments):
    bpy.context.view_layer.objects.active = obj
    try:
        # For classes that need predefined types (like walls)
        if ifc_class.lower() in ["wall_mv1","wall_mv2"]:
            bpy.ops.bim.assign_class(ifc_class="IfcWall", predefined_type="SOLIDWALL")
        elif ifc_class.lower() == "floor":
            bpy.ops.bim.assign_class(ifc_class="IfcSlab", predefined_type="FLOOR")
        else:
            bpy.ops.bim.assign_class(ifc_class=ifc_class)
        print(f"Assigned {ifc_class} to {obj.name}")
    except Exception as e:
        print(f"Failed to assign {ifc_class} to {obj.name}: {str(e)}")

# Set export path
export_path = output_path+".ifc"

# Export to IFC
bpy.ops.bim.save_project(filepath=export_path)