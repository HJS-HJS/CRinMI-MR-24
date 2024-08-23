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
mesh_2 = o3d.io.read_triangle_mesh(mesh_dir + "/" + CLASS[id]['name'] + CLASS[id]['type'])
mesh_2.scale(0.001, center=([0, 0, 0]))

matrix =    np.array([
      [ 0.,    1.,    0.,    0.  ],
      [-1.,    0.,   -0.,    0.02],
      [-0.,    0.,    1.,    0.  ],
      [ 0.,    0.,    0.,    1.  ],
    ])
mesh_2.transform(matrix)
vis.add_geometry(mesh_2)




origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=mesh.get_center())
vis.add_geometry(origin_frame)

origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=mesh_2.get_center())
vis.add_geometry(origin_frame)


vis.run()
