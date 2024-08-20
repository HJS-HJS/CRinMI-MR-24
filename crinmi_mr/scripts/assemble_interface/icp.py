import os
import numpy as np
import open3d as o3d
import copy
import tf.transformations
import rospkg

from assemble_interface.yolo_class import CLASS

class ICP():
    def __init__(self, voxel_size:float = 0.001, cost_change_threshold:float = 0.000005):
        self.voxel_size = voxel_size
        self.cost_change_threshold = cost_change_threshold
        self.mesh_dir = os.path.abspath(os.path.join(rospkg.RosPack().get_path('crinmi_mr'),'mesh'))
    
    def get_depth_pcd(self, np_pcd, is_guide = True):
        np_pcd = np_pcd[np.where(np_pcd[:,2] > 0.01)]
        np_pcd = np_pcd[np.where(np_pcd[:,2] < 0.3)]
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np_pcd)
        voxel_down_pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)
        cl, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=200, std_ratio=0.02)
        depth_pcd = voxel_down_pcd.select_by_index(ind)
        if len(depth_pcd.points) > 4000:
            print("down sample")
            depth_pcd = depth_pcd.voxel_down_sample(voxel_size=self.voxel_size*1.5)
        # if not is_guide:
            # cl, ind = depth_pcd.remove_statistical_outlier(nb_neighbors=800, std_ratio=1.0)
            # depth_pcd = depth_pcd.select_by_index(ind)
        depth_pcd.paint_uniform_color([1, 0, 0])
        return depth_pcd

    def get_mesh_pcd(self, id, depth_pcd):
        mesh = o3d.io.read_triangle_mesh(self.mesh_dir + "/" + CLASS[id]['name'] + CLASS[id]['type'])
        mesh.scale(0.001, center=([0, 0, 0]))
        mesh_pcd = mesh.sample_points_uniformly(number_of_points = 4000)
        mesh_pcd = mesh_pcd.voxel_down_sample(voxel_size=self.voxel_size)
        mesh_pcd.paint_uniform_color([0.1, 0.1 ,0.1])
        matrix = tf.transformations.euler_matrix(np.pi/2, 0, 0)
        matrix[0:3,3] = depth_pcd.get_center() - mesh_pcd.get_center()
        mesh_pcd.transform(matrix)
        return mesh_pcd, matrix
    
    def get_mesh_angle_pcd(self, id, depth_pcd, angle_list:np.array=np.array([0, 0, 0]), is_guide:bool = False):
        mesh = o3d.io.read_triangle_mesh(self.mesh_dir + "/" + CLASS[id]['name'] + CLASS[id]['type'])
        mesh.scale(0.001, center=([0, 0, 0]))
        mesh_pcd = mesh.sample_points_uniformly(number_of_points = 4000)
        mesh_pcd = mesh_pcd.voxel_down_sample(voxel_size=self.voxel_size)
        mesh_pcd.paint_uniform_color([0.1, 0.1 ,0.1])
        angle_set = np.array([np.pi/2, 0, 0]) + angle_list
        matrix = tf.transformations.euler_matrix(angle_set[0], angle_set[1], angle_set[2])
        matrix[0:3,3] = depth_pcd.get_center() - mesh_pcd.get_center()
        mesh_pcd.transform(matrix)
        mesh_pcd.translate([0, 0, -0.02])
        matrix[0:3,3] += [0, 0, -0.02]
        if is_guide:
            print('mesh_pcd.get_center()', mesh_pcd.get_center())
            print(len(mesh_pcd.points))
            min_val = np.min(mesh_pcd.points,axis=0)
            min_list = np.where(np.array(mesh_pcd.points)[:,2] > (min_val[2] + 0.03))
            print(min_list[0])
            print(len(min_list[0]))
            mesh_pcd = mesh_pcd.select_by_index(np.array(min_list))
            print(len(mesh_pcd.points))
            print('mesh_pcd.get_center()',mesh_pcd.get_center())

        return mesh_pcd, matrix

    def run_icp(self, depth_pcd, id:int, is_guide:bool = True):
        if is_guide:
            print("Guide")
            mesh_pcd, matrix = self.get_mesh_angle_pcd(id, depth_pcd,np.array([0, 0, 0]), is_guide)
            mesh = copy.deepcopy(mesh_pcd)
            pose, cost = self.icp(mesh, depth_pcd, 20, 600)
            if CLASS[id]['rotate']:
                mesh_pcd2, matrix2 = self.get_mesh_angle_pcd(id, depth_pcd, np.array([0, 0, np.pi]), is_guide)
                mesh = copy.deepcopy(mesh_pcd2)
                pose2, cost2 = self.icp(mesh, depth_pcd, 20, 600)
                if cost2 < cost:
                    pose = pose2
                    mesh_pcd = mesh_pcd2
                    matrix = matrix2
                    print("\n\n\ncase 2\n\n\n")
                mesh_pcd3, matrix3 = self.get_mesh_angle_pcd(id, depth_pcd, np.array([0, 0, np.pi/2]), is_guide)
                mesh = copy.deepcopy(mesh_pcd3)
                pose3, cost3 = self.icp(mesh, depth_pcd, 20, 600)
                if cost3 < cost:
                    pose = pose3
                    mesh_pcd = mesh_pcd3
                    matrix = matrix3
                    print("\n\n\ncase 2\n\n\n")
            return pose, mesh_pcd, matrix
                    
        else:
            print("Assmeble")
            min_cost = 1000.0
            min_pose = np.eye(4)
            min_matrix = np.eye(4)
            min_mesh_pcd = None
            iter_limit = 50
            iter = 0
            while iter < iter_limit:
                mesh_pcd, matrix = self.get_mesh_angle_pcd(id, depth_pcd, np.array(np.random.random(3) * np.pi * 2))
                mesh = copy.deepcopy(mesh_pcd)
                pose, cost = self.icp(mesh, depth_pcd, CLASS[id]['stop_idx'], CLASS[id]['stop_threshold'])
                if cost < min_cost:
                    min_cost = cost
                    min_matrix = matrix
                    min_pose = pose
                    min_mesh_pcd = copy.deepcopy(mesh_pcd)
                print(min_cost)
                print(CLASS[id]["cost"])
                print(iter)
                if min_cost < CLASS[id]["cost"]: break
                iter += 1
            print("Finish ICP. Iter: ", iter, ", Cost: ", min_cost)
            return min_pose, min_mesh_pcd, min_matrix

    def icp(self, source, target, stop_idx, stop_threshold):
        target_points = np.asarray(target.points)

        # While loop variables
        curr_iteration = 0
        # cost_change_threshold = 0.005
        self.cost_change_threshold = 0.000005
        curr_cost = 1000
        prev_cost = 10000

        total_transform_matrix = np.eye(4)

        while (True):
            # 1. Find nearest neighbors
            new_source_points = self.find_nearest_neighbors(source, target, 1)

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
            if ((prev_cost - curr_cost) > self.cost_change_threshold):
                prev_cost = curr_cost
                transform_matrix = np.hstack((R, t.T))
                transform_matrix = np.vstack((transform_matrix, np.array([0, 0, 0, 1])))
                # If cost_change is acceptable, update source with new transformation matrix
                source = source.transform(transform_matrix)
                total_transform_matrix = total_transform_matrix @ transform_matrix
                curr_iteration += 1
            else:
                break
            if (curr_iteration > stop_idx) and (curr_cost > stop_threshold):
                break
        print("Curr_cost=", curr_cost)
        print("\nIteration=", curr_iteration)
        return total_transform_matrix, curr_cost


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
    def vis_pcd(depth_pcd, mesh_pcd, mesh_transpose=np.eye(4)):
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        mesh = copy.deepcopy(mesh_pcd).transform(mesh_transpose)
        vis.add_geometry(depth_pcd)
        vis.add_geometry(mesh)
        # depth_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=depth_pcd.get_center())
        # mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=mesh.get_center())
        # vis.add_geometry(depth_frame)
        # vis.add_geometry(mesh_frame)
        vis.run()

    @staticmethod
    def get_class_name(id):
        return CLASS[id]["name"]
    
    @staticmethod
    def rotation_matrix(list):
        return tf.transformations.euler_matrix(list[0], list[1], list[2])