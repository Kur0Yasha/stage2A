from open3d.visualization import draw_geometries as dg
from util import *


# Display a comparison of a list of planes
def visualize(planes_obbs, pcds=None):
    for plane_obb in planes_obbs:
        plane_obb.color = [0, 0, 0]  # Set color to black for better visualization
    if pcds is None:
        pcds = []
    dg(planes_obbs + pcds)  # Display comparison of pcd/obj with open3d


if __name__ == "__main__":
    obj_path = "MaisonLyonObject.obj"
    pcd_path = "SimplifiedMaisonLyon.e57"
    visualize(obj_path)
