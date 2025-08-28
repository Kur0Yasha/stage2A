import tkinter as tk
from tkinter import ttk, colorchooser
from tkinter import filedialog
import open3d as o3d
import numpy as np
from main2 import *
from planardetec import *
import threading
import time
from holes import *
from objects import *
from clusteringobbs import *
from clusterpictures import *

def pick_color(initial):
    color = colorchooser.askcolor(color=initial)[1]
    return color if color else initial

def obb_to_pcd(obbs, voxel_size=0.01):
    """
    Convert a list of Open3D OrientedBoundingBoxes into point clouds.
    Ensures each OBB has at least 2 voxels along each dimension.

    Parameters:
        obbs (list[open3d.geometry.OrientedBoundingBox]): List of OBBs.
        voxel_size (float): Distance between points.

    Returns:
        list[open3d.geometry.PointCloud]: List of point clouds for each OBB.
    """
    pointclouds = []

    for obb in obbs:
        # Copy extent to modify safely
        extent = np.array(obb.extent)
        rotation = obb.R
        center = obb.center

        # Ensure at least 2 voxels along each axis
        min_extent = 2 * voxel_size
        adjusted_extent = np.maximum(extent, min_extent)

        # Compute number of points per axis
        num_x, num_y, num_z = np.ceil(adjusted_extent / voxel_size).astype(int)

        # Generate local grid coordinates
        x = np.linspace(-adjusted_extent[0] / 2, adjusted_extent[0] / 2, num_x)
        y = np.linspace(-adjusted_extent[1] / 2, adjusted_extent[1] / 2, num_y)
        z = np.linspace(-adjusted_extent[2] / 2, adjusted_extent[2] / 2, num_z)
        grid = np.stack(np.meshgrid(x, y, z, indexing='ij'), -1).reshape(-1, 3)

        # Transform grid to world coordinates
        rotated_points = (rotation @ grid.T).T + center

        # Create the point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(rotated_points)

        pointclouds.append(pcd)

    return pointclouds

def concatenate(pcds):
    """
    Concatenate a list of Open3D point clouds into a single point cloud,
    merging points and colors if available (normals are ignored).
    
    Parameters:
        pcds (list of o3d.geometry.PointCloud): list of point clouds to merge
        
    Returns:
        o3d.geometry.PointCloud: the concatenated point cloud
    """
    # Merge points
    combined_points = np.vstack([np.asarray(pcd.points) for pcd in pcds])

    # Merge colors if at least one cloud has them
    if any(pcd.has_colors() for pcd in pcds):
        combined_colors = np.vstack([
            np.asarray(pcd.colors) if pcd.has_colors() 
            else np.zeros((len(pcd.points), 3))  # black if no color
            for pcd in pcds
        ])
    else:
        combined_colors = None

    # Create final point cloud
    combined_pcd = o3d.geometry.PointCloud()
    combined_pcd.points = o3d.utility.Vector3dVector(combined_points)
    if combined_colors is not None:
        combined_pcd.colors = o3d.utility.Vector3dVector(combined_colors)

    return combined_pcd

def compute_centers(point_clouds):
    """Compute the center of each point cloud."""
    centers = []
    for pcd in point_clouds:
        points = np.asarray(pcd.points)
        center = np.mean(points, axis=0)
        centers.append(center)
    return np.array(centers)

def build_adjacency_matrix(centers, distance_threshold):
    """
    Build adjacency matrix where clouds are adjacent if
    their centers are within distance_threshold.
    """
    n = len(centers)
    adj_matrix = np.zeros((n, n), dtype=int)
    for i in range(n):
        for j in range(i+1, n):
            dist = np.linalg.norm(centers[i] - centers[j])
            if dist <= distance_threshold:
                adj_matrix[i, j] = 1
                adj_matrix[j, i] = 1
    return adj_matrix

def get_cloud_with_neighbors(point_clouds, adj_matrix, index):
    """
    Given a point cloud index, return a concatenated cloud
    with the main cloud in original colors and neighbors in purple.
    """
    main_cloud = point_clouds[index]
    result_cloud = o3d.geometry.PointCloud(main_cloud)  # copy

    neighbors = np.where(adj_matrix[index] == 1)[0]
    for n_idx in neighbors:
        neighbor_copy = o3d.geometry.PointCloud(point_clouds[n_idx])
        # Change color to purple
        purple = np.array([[1.0, 0.0, 1.0]])
        neighbor_copy.colors = o3d.utility.Vector3dVector(
            np.tile(purple, (len(neighbor_copy.points), 1))
        )
        result_cloud += neighbor_copy

    return result_cloud

def split_point_cloud_by_obbs(point_cloud, obbs):

    # Get the numpy array of points from the cloud
    points = np.asarray(point_cloud.points)
    
    # Extract colors if present
    has_colors = len(point_cloud.colors) > 0
    colors = np.asarray(point_cloud.colors) if has_colors else None

    sub_point_clouds = []
    for obb in obbs:
        # Mask for points inside the OBB
        mask = obb.get_point_indices_within_bounding_box(point_cloud.points)
        
        if len(mask) > 0:
            sub_pcd = o3d.geometry.PointCloud()
            sub_pcd.points = o3d.utility.Vector3dVector(points[mask])
            if has_colors:
                sub_pcd.colors = o3d.utility.Vector3dVector(colors[mask])
            sub_point_clouds.append(sub_pcd)
        else:
            # Append an empty point cloud if no points are inside
            sub_point_clouds.append(o3d.geometry.PointCloud())

    return sub_point_clouds

class DynamicPCDApp:
    def __init__(self, master):
        self.master = master
        
        master.title("Dynamic Point Cloud Loader")
        self.selected_algorithm = tk.StringVar(value="DBSCAN") 
        # === GUI elements ===
        self.filename_entry = tk.Entry(master, width=40)
        self.filename_entry.pack(pady=5)
        
        self.slider_value = tk.DoubleVar(value=0.5)
        self.slider = tk.Scale(master, from_=0.0, to=1.0, resolution=0.001, 
                               orient=tk.HORIZONTAL, variable=self.slider_value,
                               length=300, label="Sampling proportion (0.000 - 1.000)")
        self.slider.pack(pady=5)

        self.algorithms_frame = tk.Frame(master)
        self.algorithms_frame.pack(pady=5)

            

        tk.Label(master, text=f"eps").pack(anchor='w')
        self.s1 = tk.Scale(master, from_=0, to=1, resolution = 0.01, orient=tk.HORIZONTAL)
        self.s1.set(0.1)
        self.s1.pack(anchor='w')
                
        tk.Label(master, text=f"Min samples").pack(anchor='w')
        self.s2 = tk.Scale(master, from_=0, to=50, resolution = 1, orient=tk.HORIZONTAL)
        self.s2.set(4)
        self.s2.pack(anchor='w')
                
        tk.Label(master, text=f"Min point per cluster").pack(anchor='w')
        self.s3 = tk.Scale(master, from_=0, to=10000, resolution = 10, orient=tk.HORIZONTAL)
        self.s3.set(1000)
        self.s3.pack(anchor='w')



        self.load_button = tk.Button(master, text="Load File", command=self.load_file)
        self.load_button.pack(pady=5)
        


        self.checkboxes_frame = tk.Frame(master)
        self.checkboxes_frame.pack(pady=10)
        
        # === Visualize button ===
        self.visualize_button = tk.Button(master, text="Visualize", command=self.start_visualization)
        self.visualize_button.pack(pady=5)

        self.visualize_button_rc = tk.Button(master, text="Visualize reconstruction", command=self.start_visualization_rc)
        self.visualize_button_rc.pack(pady=5)

        self.xpfilename_entry = tk.Entry(master, width=40)
        self.xpfilename_entry.pack(pady=5)


        self.xpvisualize_button = tk.Button(master, text="Export reconstruction", command=self.export_obj_sub)
        self.xpvisualize_button.pack(pady=5)

        self.vis_thread = None
        self.stop_event = threading.Event()
        
        self.sub_pcd_list = []  # List of (sub_pcd, IntVar)
        self.recongeom_list = []
        self.current_planes = []
        self.full_pcd = []
        self.categories = {
            "Murs": {
                "color": "#ff0000",
                "show": tk.BooleanVar(value=True),
                "expanded": tk.BooleanVar(value=True),
                "items": [
                    {"name": "Item1", "show": tk.BooleanVar(value=True)},
                    {"name": "Item2", "show": tk.BooleanVar(value=True)},
                ]
            },
            "Sols": {
                "color": "#00ff00",
                "show": tk.BooleanVar(value=True),
                "expanded": tk.BooleanVar(value=False),
                "items": [
                    {"name": "Item3", "show": tk.BooleanVar(value=True)},
                    {"name": "Item4", "show": tk.BooleanVar(value=True)},
                ]
            },
            "Autre": {
                "color": "#0000ff",
                "show": tk.BooleanVar(value=True),
                "expanded": tk.BooleanVar(value=False),
                "items": [
                    {"name": "Item5", "show": tk.BooleanVar(value=True)},
                    {"name": "Item6", "show": tk.BooleanVar(value=True)},
                ]
            }
        }
        self.global_show = tk.BooleanVar(value=True)
        
        self.build_category_list()

    def build_category_list(self):
        # Main frame for list
        self.list_frame = tk.Frame(self.master)
        self.list_frame.pack(fill="both", expand=True)
        
        # Header
        header = tk.Frame(self.list_frame)
        header.pack(fill="x")
        tk.Checkbutton(header, variable=self.global_show, command=self.toggle_all).pack(side="left")
        tk.Label(header, text="Category").pack(side="left", padx=5)
        tk.Label(header, text="Items").pack(side="right", padx=5)
        
        self.category_frames = {}
        
        for cat_name, cat_data in self.categories.items():
            self.add_category_row(cat_name, cat_data)

    def add_category_row(self, cat_name, cat_data):
        cat_frame = tk.Frame(self.list_frame, borderwidth=1, relief="solid")
        cat_frame.pack(fill="x", pady=1)
        
        # Color button
        tk.Button(
            cat_frame, 
            bg=cat_data["color"], 
            width=2, 
            command=lambda c=cat_name: self.change_category_color(c)
        ).pack(side="left")
        
        # Show/hide checkbox
        tk.Checkbutton(
            cat_frame, 
            variable=cat_data["show"], 
            command=lambda c=cat_name: self.toggle_category(c)
        ).pack(side="left")
        
        # Expand button
        expand_btn = tk.Button(
            cat_frame, 
            text="-" if cat_data["expanded"].get() else "+", 
            width=2,
            command=lambda c=cat_name, b=None: self.toggle_expand(c)
        )
        expand_btn.pack(side="left")
        
        # Category name
        tk.Label(cat_frame, text=cat_name).pack(side="left", padx=5)
        
        # Item count
        count_lbl = tk.Label(cat_frame, text=str(len(cat_data["items"])))
        count_lbl.pack(side="right", padx=5)
        
        # Items frame
        items_frame = tk.Frame(self.list_frame)
        self.category_frames[cat_name] = (items_frame, expand_btn, count_lbl)
        if cat_data["expanded"].get():
            self.add_items_to_frame(cat_name, items_frame)
            items_frame.pack(fill="x", padx=20)
    
    def add_items_to_frame(self, cat_name, items_frame):
        for item in self.categories[cat_name]["items"]:
            row = tk.Frame(items_frame)
            row.pack(fill="x")
            tk.Checkbutton(row, variable=item["show"]).pack(side="left")
            tk.Label(row, text=item["name"]).pack(side="left", padx=5)
            
            # Dropdown to move category
            options = list(self.categories.keys())
            var = tk.StringVar(value=cat_name)
            menu = ttk.OptionMenu(row, var, cat_name, *options, command=lambda new_cat, i=item, old_cat=cat_name: self.move_item(i, old_cat, new_cat))
            menu.pack(side="right")
    
    def change_category_color(self, cat_name):
        new_color = pick_color(self.categories[cat_name]["color"])
        self.categories[cat_name]["color"] = new_color
        self.refresh_ui()
    
    def toggle_all(self):
        val = self.global_show.get()
        for cat in self.categories.values():
            cat["show"].set(val)
            for item in cat["items"]:
                item["show"].set(val)
    
    def toggle_category(self, cat_name):
        val = self.categories[cat_name]["show"].get()
        for item in self.categories[cat_name]["items"]:
            item["show"].set(val)
    
    def toggle_expand(self, cat_name):
        self.categories[cat_name]["expanded"].set(not self.categories[cat_name]["expanded"].get())
        self.refresh_ui()
    
    def move_item(self, item, old_cat, new_cat):
        if old_cat != new_cat:
            self.categories[old_cat]["items"].remove(item)
            self.categories[new_cat]["items"].append(item)
            self.refresh_ui()
    
    def refresh_ui(self):
        self.list_frame.destroy()
        self.build_category_list()     
        
    
    
    def load_file(self):
        filename = self.filename_entry.get()
        if not filename:
            print("No file name entered.")
            return
        proportions = self.slider_value.get()

        self.sub_pcd_list = []
        self.clear_checkboxes()
        
        values = [self.s1.get(),self.s2.get(),self.s3.get()]

        sub_pcds = main2(filename, proportions, "Simplified"+filename, values)
        self.full_pcd = concatenate(sub_pcds)

        planes = []
        for sub_pcd in sub_pcds:
            tplanes,center = find_planes(sub_pcd)
            for plane in tplanes:
                plane.translate(center)
            planes += tplanes

        new_pcd = removeobb(self.full_pcd,planes)


        clusters = cluster_large_objects(new_pcd,0.1,4,50) + split_point_cloud_by_obbs(self.full_pcd,planes)

        centers = compute_centers(clusters)
        adj_mtx = build_adjacency_matrix(centers, 1.5)

        

        for k in range(len(clusters)):
            save_pointcloud_views(get_cloud_with_neighbors(clusters,adj_mtx,k),k)
            
        for idx, pcd in enumerate(sub_pcds):
            var = tk.IntVar(value=1)
            cb = tk.Checkbutton(self.checkboxes_frame, text=f"SubCloud {idx+1}", variable=var)
            cb.pack(anchor='w')
            self.sub_pcd_list.append((pcd, var))
        
    
    def clear_checkboxes(self):
        for widget in self.checkboxes_frame.winfo_children():
            widget.destroy()
    
    
    def start_visualization(self):
        # Stop any existing visualization
        self.stop_visualization()
        
        # Start a new visualization thread
        self.stop_event = threading.Event()
        self.vis_thread = threading.Thread(target=self.visualization_loop)
        self.vis_thread.start()

    def start_visualization_rc(self):
        # Stop any existing visualization
        self.stop_visualization()
        
        # Start a new visualization thread
        self.stop_event = threading.Event()
        self.vis_thread = threading.Thread(target=self.visualization_loop_rc)
        self.vis_thread.start()
    
    def stop_visualization(self):
        if self.vis_thread and self.vis_thread.is_alive():
            print("Stopping previous visualization...")
            self.stop_event.set()
            self.vis_thread.join()
            print("Visualization stopped.")
    
    def visualization_loop(self):
        print("Starting visualization loop...")
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name='Open3D Viewer', width=800, height=600)
        
        # Add selected point clouds
        for sub_pcd, var in self.sub_pcd_list:
            sub_pcd.paint_uniform_color(np.random.rand(3))
            if var.get() == 1:
                vis.add_geometry(sub_pcd)
        
        while not self.stop_event.is_set():
            vis.poll_events()
            vis.update_renderer()
            time.sleep(0.02)
        
        vis.destroy_window()
        print("Visualizer window closed.")
    

    def visualization_loop_rc(self):
        print("Starting visualization loop...")
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name='Open3D Viewer', width=800, height=600)
        
        pcds = []
        planes = []
        # Add selected point clouds
        for sub_pcd, var in self.sub_pcd_list:
            sub_pcd.paint_uniform_color(np.random.rand(3))
            if var.get() == 1:
                #pcds.append(sub_pcd)
                tplanes,center = find_planes(sub_pcd)
                for plane in tplanes:
                    plane.translate(center)
                planes += tplanes

        
        #cloud = concatenate(pcds)

        #planes = find_planes(cloud)

        self.current_planes = planes

        for plane in planes:
            vis.add_geometry(plane)

        while not self.stop_event.is_set():
            vis.poll_events()
            vis.update_renderer()
            time.sleep(0.02)
        
        vis.destroy_window()
        print("Visualizer window closed.")
    
    def export_obj_sub(self):
        
        planes = self.current_planes
        pcd = self.full_pcd

        holes = []

        for k in planes:
            temp = findholes(pcd,k)
            for hole in temp:
                holes.append(hole)
        

        
        holes_obb = holeobb(holes)
        
        planes_pcds = obb_to_pcd(planes)
        holes_pcds = obb_to_pcd(holes_obb)

        temp_volumes = cluster_point_clouds(planes_pcds,0.98,0.5,0.1)
        temp_holes_volumes = cluster_point_clouds(holes_pcds,0.9,0.5,0.3)

        volumes = []
        holes_volumes = []

        for k in temp_volumes:
            volumes.append(o3d.geometry.OrientedBoundingBox.create_from_points(k.points))

        for k in temp_holes_volumes:
            holes_volumes.append(o3d.geometry.OrientedBoundingBox.create_from_points(k.points))

        export_objects(volumes,"positive.obj")
        export_objects(holes_volumes,"negative.obj")


        # set the parameters.
        blend_file = "temp.blend"
        blender_script = "substraction.py"
        blender_exe = r"C:\Program Files\Blender Foundation\Blender 4.4\blender.exe" 
        outp = self.xpfilename_entry.get()
        # write down the command.
        args2 = [
            blender_exe,
            "--background",  # no UI
            blend_file,
            "--python", blender_script, "--", 
            outp
            ]
        

        export_path = "temp.blend1"

        try:
            os.remove(export_path)
            print(f"File '{export_path}' deleted successfully")
        except FileNotFoundError:
            print(f"File '{export_path}' not found")
        except PermissionError:
            print(f"Permission denied to delete '{export_path}'")
        except OSError as e:
            print(f"Error deleting file: {e}")

        # execute the command to call the blender script with the correct arguments.
        subprocess.run(args2, check=True)

    def update_algorithm_parameters(self):
        selected = self.selected_algorithm.get()
        print(f"Algorithm selected: {selected}")
    
        # Hide all parameter frames
        for frame in self.algorithm_params_frames.values():
            frame.pack_forget()
    
        # Show only the selected one
        if selected in self.algorithm_params_frames:
            self.algorithm_params_frames[selected].pack(pady=5)

    def close(self):
        print("Shutting down...")
        self.stop_visualization()
        self.master.destroy()
    
    

    

if __name__ == "__main__":
    root = tk.Tk()
    app = DynamicPCDApp(root)
    root.protocol("WM_DELETE_WINDOW", app.close)
    root.mainloop()