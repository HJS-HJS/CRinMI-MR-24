import numpy as np
import open3d as o3d

obj_list = 9
obj_pcd = []

vis = o3d.visualization.Visualizer()
vis.create_window()




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

np_pcd = np.load("/home/rise/catkin_ws/src/crinmi/data/64/" + str(6) + ".npy")
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np_pcd)
voxel_down_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
cl, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=200, std_ratio=0.02)
pcd_r = voxel_down_pcd.select_by_index(ind)
print(pcd_r)
obj_pcd.append(pcd_r)
vis.add_geometry(pcd_r)



name = "a_cam_zig"
mesh = o3d.io.read_triangle_mesh("/home/rise/catkin_ws/src/crinmi/data/mesh_file/" + name + ".obj")
mesh.scale(0.254, center=mesh.get_center())
R = mesh.get_rotation_matrix_from_xyz((np.pi+np.pi/3, np.pi, 0))
mesh.rotate(R, center=(0, 0, 0))
mesh.translate(pcd_r.get_center() - mesh.get_center())

pcd = mesh.sample_points_uniformly(number_of_points=10000)
pcd = pcd.voxel_down_sample(voxel_size=voxel_size)

pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 20, max_nn=1000))
pcd_r.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 20, max_nn=1000))

# vis.add_geometry(pcd)

# vis.run()


# vis = o3d.visualization.Visualizer()
# vis.create_window()
# vis.add_geometry(pcd_r)


# icp
threshold = 0.1
trans_init = np.eye(4)
# demo_icp_pcds = o3d.data.DemoICPPointClouds()

# source = o3d.io.read_point_cloud(demo_icp_pcds.paths[0])
# target = o3d.io.read_point_cloud(demo_icp_pcds.paths[1])

# reg_p2p = o3d.pipelines.registration.registration_icp(
#     source=pcd_r, target=pcd, max_correspondence_distance=voxel_size * 1.5,
#     init=trans_init,
#     estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane()
# )

# pcd_f = o3d.pipelines.registration.compute_fpfh_feature(
#     pcd,
#     o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*5, max_nn=450))

# pcd_r_f = o3d.pipelines.registration.compute_fpfh_feature(
#     pcd_r,
#     o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*10, max_nn=900))
# print(pcd_f)
# print(pcd_r_f)

# reg_p2p = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
#     pcd_r, pcd, pcd_r_f, pcd_f, True,
#     threshold,
#     o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
#     3, [
#         o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
#             # 0.9),
#             0.1),
#         o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
#             threshold*10)
#     ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))


reg_p2p = o3d.pipelines.registration.registration_icp(pcd, pcd_r, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration = 2000))

reg_p2l = o3d.pipelines.registration.registration_icp(pcd, pcd_r, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())


print(reg_p2p.transformation)
print(reg_p2l.transformation)
pcd = pcd.transform(reg_p2p.transformation)
# pcd = pcd.transform(reg_p2p.transformation)
# pcd = pcd.transform(reg_p2l.transformation)
vis.add_geometry(mesh)
vis.add_geometry(pcd)

vis.run()






# for i in range(obj_list):
#     print(matchting_list[str(i)])
#     np_pcd = np.load("/home/rise/catkin_ws/src/crinmi/data/64/" + str(i) + ".npy")
#     pcd = o3d.geometry.PointCloud()
#     pcd.points = o3d.utility.Vector3dVector(np_pcd)
#     voxel_down_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
#     cl, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=200, std_ratio=0.02)
#     pcd_r = voxel_down_pcd.select_by_index(ind)
#     obj_pcd.append(pcd_r)
#     vis.add_geometry(pcd_r)
#     vis.update_renderer()
# vis.update_renderer()
# vis.close()
# vis.destroy_window()
