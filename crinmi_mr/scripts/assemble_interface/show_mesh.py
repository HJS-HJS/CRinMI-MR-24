import os
import numpy as np
import open3d as o3d
import copy
import tf.transformations
import rospkg

from yolo_class import CLASS

mesh_dir = os.path.abspath(os.path.join(rospkg.RosPack().get_path('crinmi_mr'),'mesh'))
id = 3
angle_set = []
angle_set.append(np.array([np.pi/2, 0, 0]) + np.array([0, 0, 0]))
# angle_set.append(np.array([np.pi/2, 0, 0]) + np.array([0, 0, np.pi]))
angle_set.append(np.array([np.pi/2, 0, 0]) + np.array([np.pi, 0, 0]))
# angle_set.append(np.array([np.pi/2, 0, 0]) + np.array([np.pi/2, np.pi/2, 0]))
# angle_set.append(np.array([np.pi/2, 0, 0]) + np.array([np.pi/2, 0, 0]))
# angle_set.append(np.array([np.pi/2, 0, 0]) + np.array([np.pi/2, 0, 0]))
# angle_set.append(np.array([np.pi/2, 0, 0]) + np.array([0, np.pi/2, 0]))
# angle_set.append(np.array([np.pi/2, 0, 0]) + np.array([np.pi/2, np.pi/2, 0]))
# angle_set.append(np.array([np.pi/2, 0, 0]) + np.array([np.pi/2, 0, 0]))
# angle_set.append(np.array([np.pi/2, 0, 0]) + np.array([0, np.pi*3/4, 0]))
# angle_set.append(np.array([np.pi/2, 0, 0]) + np.array([0, np.pi/2, 0 ]))
# angle_set.append(np.array([np.pi/2, 0, 0]) + np.array([np.pi, 0, np.pi/2]))
# angle_set.append([np.pi/2, 0, 0] + [0, 0, 0])
print(angle_set)


vis = o3d.visualization.Visualizer()
vis.create_window()


for angle in angle_set:

    mesh = o3d.io.read_triangle_mesh(mesh_dir + "/" + CLASS[id]['name'] + CLASS[id]['type'])
    matrix = tf.transformations.euler_matrix(angle[0], angle[1], angle[2])
    mesh.transform(matrix)

    vis.add_geometry(mesh)

    # mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=mesh.get_center())
    # vis.add_geometry(mesh_frame)

    mesh_pcd = mesh.sample_points_uniformly(number_of_points = 4000)
    array = np.asarray(mesh_pcd.points)[:,:2]
    eigen_value, vector = np.linalg.eig(np.cov(array[:,0], array[:,1]))
    vector = vector[np.argmax(eigen_value)]
    eigen_angle = -np.arctan2(vector[1], vector[0])
    print(np.rad2deg(eigen_angle))


origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=mesh.get_center())
vis.add_geometry(origin_frame)

# print(np.rad2deg(np.arccos(40/46.14108798)))


vis.run()
