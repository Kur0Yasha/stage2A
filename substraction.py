import bpy
import sys
import os

# Get custom args after "--"
argv = sys.argv
if "--" in argv:
    idx = argv.index("--")
    custom_args = argv[idx + 1:]
else:
    custom_args = []

# Extract the output path
if len(custom_args) < 1:
    raise Exception("Expected output OBJ path")
output_path = custom_args[0]

# Set export path
export_path = output_path+".obj"

def import_and_join_obj(filepath, name_prefix):


    # Import .obj file
    bpy.ops.wm.obj_import(filepath=filepath)

    # Get all imported objects
    imported_objects = [obj for obj in bpy.context.selected_objects if obj.type == 'MESH']

    # Deselect all first
    bpy.ops.object.select_all(action='DESELECT')
    
    for obj in imported_objects:
        obj.select_set(True)

    # Join all selected objects into one
    bpy.context.view_layer.objects.active = imported_objects[0]
    bpy.ops.object.join()

    # Rename the joined object
    joined_obj = bpy.context.active_object
    joined_obj.name = f"{name_prefix}_Combined"
    
    return joined_obj

def subtract_meshes(obj1, obj2):
    # Ensure both objects are visible and on the same layer
    obj1.select_set(True)
    obj2.select_set(True)

    # Create a new mesh object that will be the result
    bpy.ops.object.select_all(action='DESELECT')
    obj1.select_set(True)
    bpy.context.view_layer.objects.active = obj1

    # Add Boolean modifier
    boolean_modifier = obj1.modifiers.new(name="Boolean_Subtract", type='BOOLEAN')
    boolean_modifier.operation = 'DIFFERENCE'
    boolean_modifier.use_self = False
    boolean_modifier.object = obj2

    # Apply the modifier
    bpy.ops.object.modifier_apply(modifier=boolean_modifier.name)

    # Optionally, delete the second object
    bpy.data.objects.remove(obj2, do_unlink=True)

    return obj1

# Replace these with your actual file paths
obj1_path = "positive.obj"
obj2_path = "negative.obj"

# Delete default cube and other scene objects
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)

# Import and combine both files
base_obj = import_and_join_obj(obj1_path, "Base")
cutter_obj = import_and_join_obj(obj2_path, "Cutter")

# Perform subtraction
result_obj = subtract_meshes(base_obj, cutter_obj)

bpy.ops.wm.obj_export(filepath=export_path)