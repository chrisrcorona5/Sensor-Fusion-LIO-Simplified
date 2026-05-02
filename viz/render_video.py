import open3d as o3d
import numpy as np
import os
from natsort import natsorted

is_paused = False
def toggle_pause(vis):
    global is_paused
    is_paused = not is_paused
    return False



if __name__ == "__main__":
    directory = "data/2011_09_26/2011_09_26_drive_0070_sync/velodyne_points/data/"

    # Sorting sequentially correct order, even though they are already processed in order
    files =  natsorted([os.path.join(directory, f) for f in os.listdir(directory) if f.endswith('.bin')])
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window()

    # Register spacebar to toggle pause/play
    vis.register_key_callback(32, toggle_pause)

    pcd = o3d.geometry.PointCloud()

    # Load first point cloud to initialize visualizer
    first_points = np.fromfile(files[0], dtype=np.float32).reshape(-1, 4)
    #Why reshape? Because reshape into a 2D array 

    pcd.points = o3d.utility.Vector3dVector(first_points[:, :3])

    vis.add_geometry(pcd)


    render_option = vis.get_render_option()
    render_option.point_size = 2.0
    render_option.background_color = np.asarray([0, 0, 0])

    file_idx = 1
    while file_idx < len(files):
        if not is_paused:
            points = np.fromfile(files[file_idx], dtype=np.float32).reshape(-1, 4)
            pcd.points = o3d.utility.Vector3dVector(points[:, :3])
            vis.update_geometry(pcd)
            file_idx += 1


        vis.poll_events()
        vis.update_renderer()
    vis.destroy_window()