import numpy as np
import open3d as o3d

obj_list = 9
obj_pcd = []





matchting_list = {
    "0": "a_L_shape_angles",
    "1": "a_bearing_holders",
    "2": "a_cam_zig",
    "3": "a_hex_wrenchs",
    "4": "a_rod_end_bearings",
    "5": "a_rod_end_bearings",
    "6": "a_rod_end_bearings",
    "7": "a_rod_end_bearings",
    "8": "a_rod_end_bearings",
    "9": "a_rod_end_bearings",
}




voxel_size = 0.01
# name = "a_L_shape_angle"
name = "a_cam_zig"
# name = "a_bearing_holder"
# name = "a_bearing_holder"
# name = "a_rod_end_bearing"
# name = "a_cam_zig"
# name = "a_hex_wrench"
# name = "a_L_shape_angle"
# name = "a_cam_zig"
# name = "a_roller"

# name = "g_hex_wrench_x"
# name = "g_hex_wrench_y"
# name = "g_cam_zig_x"
# name = "g_cam_zig_y"
# name = "g_roller_x"
# name = "g_L_shape_angle"
# name = "g_roller_y"



np_pcd = np.load("/home/rise/catkin_ws/src/crinmi/CRinMi_MR/data/76/" + str(1) + ".npy")
np_pcd = np_pcd[np.arange(1,np_pcd.shape[0],2)]
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np_pcd)
voxel_down_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
cl, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=200, std_ratio=0.02)
pcd1 = voxel_down_pcd.select_by_index(ind)
pcd1.paint_uniform_color([1, 0, 0])



mesh2 = o3d.io.read_triangle_mesh("/home/rise/catkin_ws/src/crinmi/CRinMi_MR/data/mesh_file/" + name + ".obj")
mesh2.scale(0.254, center=pcd1.get_center())

R = mesh2.get_rotation_matrix_from_xyz((0, 0, np.pi/2))
mesh2.rotate(R, center=pcd1.get_center())

# pcd2 = mesh2.sample_points_uniformly(number_of_points = 1500) # assamble
pcd2 = mesh2.sample_points_uniformly(number_of_points = 10000)
pcd2 = pcd2.voxel_down_sample(voxel_size=voxel_size)
pcd2.paint_uniform_color([0.2, 0.2 ,0.2])

# pcd1.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=1000))
# pcd2.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=1000))

# icp
threshold = 0.05
trans_init = np.eye(4)

def find_nearest_neighbors(source_pc, target_pc, nearest_neigh_num):
    # Find the closest neighbor for each anchor point through KDTree
    point_cloud_tree = o3d.geometry.KDTreeFlann(source_pc)
    # Find nearest target_point neighbor index
    points_arr = []
    for point in target_pc.points:
        [_, idx, _] = point_cloud_tree.search_knn_vector_3d(point, nearest_neigh_num)
        points_arr.append(source_pc.points[idx[0]])
    return np.asarray(points_arr)


def icp(source, target):
    target_points = np.asarray(target.points)

    # While loop variables
    curr_iteration = 0
    cost_change_threshold = 0.05
    stop_threshold = 200
    stop_idx = 20
    curr_cost = 1000
    prev_cost = 10000

    while (True):
        # 1. Find nearest neighbors
        new_source_points = find_nearest_neighbors(source, target, 1)

        # 2. Find point cloud centroids and their repositions
        source_centroid = np.mean(new_source_points, axis=0)
        target_centroid = np.mean(target_points, axis=0)
        source_repos = np.zeros_like(new_source_points)
        target_repos = np.zeros_like(target_points)
        source_repos = np.asarray([new_source_points[ind] - source_centroid for ind in range(len(new_source_points))])
        target_repos = np.asarray([target_points[ind] - target_centroid for ind in range(len(target_points))])

        # 3. Find correspondence between source and target point clouds
        cov_mat = target_repos.transpose() @ source_repos

        U, X, Vt = np.linalg.svd(cov_mat)
        R = U @ Vt
        t = target_centroid - R @ source_centroid
        t = np.reshape(t, (1,3))
        curr_cost = np.linalg.norm(target_repos - (R @ source_repos.T).T)
        print("Curr_cost=", curr_cost)
        if ((prev_cost - curr_cost) > cost_change_threshold):
            prev_cost = curr_cost
            transform_matrix = np.hstack((R, t.T))
            transform_matrix = np.vstack((transform_matrix, np.array([0, 0, 0, 1])))
            # If cost_change is acceptable, update source with new transformation matrix
            source = source.transform(transform_matrix)
            curr_iteration += 1
        else:
            break
        if (curr_iteration > stop_idx) and (curr_cost > stop_threshold):
            break
    print("\nIteration=", curr_iteration)
    return transform_matrix, curr_cost


vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(pcd1)
vis.add_geometry(pcd2)
vis.run()

cost = 1000
while cost > 110:
# while cost > 170:
    R = pcd2.get_rotation_matrix_from_xyz(np.random.random(3) * np.pi)
    pcd2.rotate(R, center=pcd1.get_center())
    result, cost = icp(pcd2, pcd1)

print(result)


vis = o3d.visualization.Visualizer()
vis.create_window()
pcd2 = pcd2.transform(result)
vis.add_geometry(pcd1)
vis.add_geometry(pcd2)

vis.run()






# for i in range(obj_list):
#     print(matchting_list[str(i)])
#     np_pcd = np.load("/home/rise/catkin_ws/src/crinmi/CRinMi_MR/data/64/" + str(i) + ".npy")
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
