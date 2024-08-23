import os
import numpy as np
import open3d as o3d
import copy
import tf.transformations
import rospkg

from yolo_class import CLASS

id = 6
angle = [0, 0, 0]

mesh_dir = os.path.abspath(os.path.join(rospkg.RosPack().get_path('crinmi_mr'),'mesh'))
mesh = o3d.io.read_triangle_mesh(mesh_dir + "/" + CLASS[id]['name'] + CLASS[id]['type'])
mesh.scale(0.001, center=([0, 0, 0]))
angle_set = np.array([np.pi/2, 0, 0]) + angle
matrix = tf.transformations.euler_matrix(angle_set[0], angle_set[1], angle_set[2])
mesh.transform(matrix)
mesh_pcd = mesh.sample_points_uniformly(number_of_points = CLASS[id]['number_of_points'])
mesh_pcd = mesh_pcd.voxel_down_sample(voxel_size=0.001)
mesh_pcd.paint_uniform_color([0.1, 0.1 ,0.1])

min_val = np.min(mesh_pcd.points,axis=0)
min_list = np.where(np.array(mesh_pcd.points)[:,2] > (min_val[2] + 0.001))
mesh_pcd = mesh_pcd.select_by_index(np.array(min_list[0]))

vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(mesh)
vis.add_geometry(mesh_pcd)
mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=mesh.get_center())
vis.add_geometry(mesh_frame)
vis.run()
