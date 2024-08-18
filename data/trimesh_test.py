import numpy as np
import open3d as o3d
import trimesh as trimesh

obj_list = 9
obj_pcd = []

matchting_list = {
    "0": "a_L_shape_angles",
    "1": "a_bearing_holders",
    "2": "a_cam_zig",
    "3": "a_hex_wrench's",
    "4": "a_rod_end_bearings",
    "5": "a_rod_end_bearings",
    "6": "a_rod_end_bearings",
    "7": "a_rod_end_bearings",
    "8": "a_rod_end_bearings",
    "9": "a_rod_end_bearings",
}

voxel_size = 0.01
name = "a_cam_zig"

# depth pointcloud
np_pcd = np.load("/home/rise/catkin_ws/src/crinmi/data/64/" + str(6) + ".npy")
o3d_pcd = o3d.geometry.PointCloud()
o3d_pcd.points = o3d.utility.Vector3dVector(np_pcd)
voxel_down_pcd = o3d_pcd.voxel_down_sample(voxel_size=voxel_size)
cl, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=200, std_ratio=0.02)
o3d_pcd = voxel_down_pcd.select_by_index(ind)
t_pcd = trimesh.points.PointCloud(o3d_pcd.points)

# CAD mesh
mesh = trimesh.load("/home/rise/catkin_ws/src/crinmi/data/mesh_file/" + name + ".obj")
scale_factor = 0.254
scale_matrix = np.eye(4)
scale_matrix[:3, :3] *= scale_factor
mesh = mesh.apply_transform(scale_matrix)

# transfrom mesh close to pcd
trans_matrix = trimesh.transformations.translation_matrix(t_pcd.centroid - mesh.center_mass)
mesh.apply_transform(trans_matrix)

# print(type(pcd))
# print(type(mesh))
# # print(pcd.shape)
# T, _, _ = trimesh.registration.icp(pcd.vertices, mesh, max_iterations=40)

T = [[ 6.43399144e-01,  1.29234636e-02, -4.01164027e-02,  1.43163366e+01],
     [-3.68901412e-02,  4.69484587e-01, -4.40411330e-01,  7.14359767e+01],
     [ 2.03828154e-02,  4.41764650e-01,  4.69219921e-01,  5.82816694e+01],
     [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]

print(T)

o3d_mesh = mesh.as_open3d
# o3d_mesh.base_color = [128, 128, 128, 150]
# o3d_mesh.transmission = 0.5

# o3d_pcd = o3d_pcd.transform(T[:3,:3])

vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(o3d_mesh)
vis.add_geometry(o3d_pcd)

vis.run()


# # visualize
# scene = trimesh.Scene()
# scene.add_geometry(t_pcd)

# mesh.apply_transform(np.linalg.inv(T))
# scene.add_geometry(mesh)
# scene.show()
