import os
import numpy as np
import open3d as o3d
import copy
import tf.transformations
import rospkg

from yolo_class import CLASS

mesh_dir = os.path.abspath(os.path.join(rospkg.RosPack().get_path('crinmi_mr'),'mesh'))
id = 5
angle_set = []

vis = o3d.visualization.Visualizer()
vis.create_window()


mesh = o3d.io.read_triangle_mesh(mesh_dir + "/" + CLASS[id]['name'] + CLASS[id]['type'])
mesh.scale(0.001, center=([0, 0, 0]))
vis.add_geometry(mesh)


id = 16
mesh = o3d.io.read_triangle_mesh(mesh_dir + "/" + CLASS[id]['name'] + CLASS[id]['type'])
mesh.scale(0.001, center=([0, 0, 0]))
vis.add_geometry(mesh)


# origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=mesh.get_center())
# vis.add_geometry(origin_frame)

# print(np.rad2deg(np.arccos(40/46.14108798)))


vis.run()
