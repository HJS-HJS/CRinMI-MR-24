import os
import numpy as np
import open3d as o3d

import rospkg

from assemble_interface.yolo_class import CLASS

class ICP():

    voxel_size = 0.001
    threshold = 0.05
    mesh_dir = os.path.abspath(os.path.join(rospkg.RosPack().get_path('crinmi_mr'),'mesh'))

    def __init__(self):
        pass

    @staticmethod
    def get_depth_pcd(np_pcd):
        # np_pcd = np_pcd[np.arange(1,np_pcd.shape[0],2)]
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np_pcd)
        voxel_down_pcd = pcd.voxel_down_sample(voxel_size=ICP.voxel_size)
        cl, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=200, std_ratio=0.02)
        depth_pcd = voxel_down_pcd.select_by_index(ind)
        # voxel_down_pcd = voxel_down_pcd.select_by_index(ind)
        # cl, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=800, std_ratio=1.0)
        # depth_pcd = voxel_down_pcd.select_by_index(ind)
        depth_pcd.paint_uniform_color([1, 0, 0])
        return depth_pcd

    @staticmethod
    def get_mesh_pcd(id, depth_pcd):
        mesh = o3d.io.read_triangle_mesh(ICP.mesh_dir + "/" + CLASS[id]['name'] + CLASS[id]['type'])
        # mesh.scale(0.000996078, center=depth_pcd.get_center())
        mesh.scale(0.001, center=depth_pcd.get_center())
        mesh_pcd = mesh.sample_points_uniformly(number_of_points = 5000)
        mesh_pcd = mesh_pcd.voxel_down_sample(voxel_size=ICP.voxel_size)
        mesh_pcd.paint_uniform_color([0.1, 0.1 ,0.1])
        R = mesh_pcd.get_rotation_matrix_from_xyz(np.array([np.pi/2, 0, 0]))
        mesh_pcd.rotate(R, center=depth_pcd.get_center())
        return mesh_pcd

    def run_icp(depth_pcd, mesh_pcd, id):
        # trans_init = np.eye(4)
        cost = 1000
        while cost > CLASS[id]["cost"]:
        # while cost > 170:
            # R = mesh_pcd.get_rotation_matrix_from_xyz(np.random.random(3) * np.pi)
            # R = mesh_pcd.get_rotation_matrix_from_xyz(np.array([np.pi/2, 0, 0]))
            # mesh_pcd.rotate(R, center=depth_pcd.get_center())
            result, cost = ICP.icp(mesh_pcd, depth_pcd, CLASS[id]['stop_idx'], CLASS[id]['stop_threshold'])

        return result

    @staticmethod
    def find_nearest_neighbors(source_pc, target_pc, nearest_neigh_num):
        # Find the closest neighbor for each anchor point through KDTree
        point_cloud_tree = o3d.geometry.KDTreeFlann(source_pc)
        # Find nearest target_point neighbor index
        points_arr = []
        for point in target_pc.points:
            [_, idx, _] = point_cloud_tree.search_knn_vector_3d(point, nearest_neigh_num)
            points_arr.append(source_pc.points[idx[0]])
        return np.asarray(points_arr)

    @staticmethod
    def icp(source, target, stop_idx, stop_threshold):
        target_points = np.asarray(target.points)

        # While loop variables
        curr_iteration = 0
        # cost_change_threshold = 0.005
        cost_change_threshold = 0.000005
        curr_cost = 1000
        prev_cost = 10000

        while (True):
            # 1. Find nearest neighbors
            new_source_points = ICP.find_nearest_neighbors(source, target, 1)

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

    @staticmethod
    def vis_pcd(depth_pcd, mesh_pcd, mesh_transpose:np.array=np.eye(4)):
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        mesh_pcd = mesh_pcd.transform(mesh_transpose)
        vis.add_geometry(depth_pcd)
        vis.add_geometry(mesh_pcd)
        depth_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.06, origin=depth_pcd.get_center())
        mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.06, origin=mesh_pcd.get_center())
        vis.add_geometry(depth_frame)
        vis.add_geometry(mesh_frame)
        vis.run()

    @staticmethod
    def get_class_name(id):
        return CLASS[id]["name"]
