import os
import numpy as np
import open3d as o3d
import copy
import tf.transformations
import rospkg

from yolo_class import CLASS

id = 12

mesh_dir = os.path.abspath(os.path.join(rospkg.RosPack().get_path('crinmi_mr'),'mesh'))

mesh = o3d.io.read_triangle_mesh(mesh_dir + "/" + CLASS[id]['name'] + CLASS[id]['type'])

print(mesh.get_center())
mesh.translate(-mesh.get_center())
print(mesh.get_center())
vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(mesh)
mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=mesh.get_center())
vis.add_geometry(mesh_frame)
vis.run()


# o3d.io.write_triangle_mesh(mesh_dir + "/" + CLASS[id]['name'] + '_test' + CLASS[id]['type'], mesh)