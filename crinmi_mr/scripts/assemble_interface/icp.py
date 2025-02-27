import os
import numpy as np
import open3d as o3d
import copy
import tf.transformations
import rospkg

from assemble_interface.yolo_class import CLASS
# from yolo_class import CLASS

class ICP():
    def __init__(self):
        """ Python class for Point-to-Point ICP.
        """
        # Path where mesh file is saved
        self.mesh_dir = os.path.abspath(os.path.join(rospkg.RosPack().get_path('crinmi_mr'),'mesh'))
    
    def get_depth_pcd(self, id:int, np_pcd:np.array):
        """ Convert pcd to open3d format.

        Args:
            id (int): Id of depth pointcloud.
            np_pcd (np.array): Depth pointcloud in n*3.

        Returns:
            o3d.geometry.PointCloud(): Depth pointcloud in open3d pointcloud type.
        """
        # Remove point cloud by z-axis height (filtering)
        np_pcd = np_pcd[np.where(np_pcd[:,2] > 0.01)]
        np_pcd = np_pcd[np.where(np_pcd[:,2] < 0.3)]
        
        # Pcd in open3d
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np_pcd)
        # Downsampling depth pcd
        depth_pcd = pcd.voxel_down_sample(voxel_size=CLASS[id]['voxel_size'])
        
        # Separately filtered according to mesh type
        # roller
        if id == 5:
            cl, ind = depth_pcd.remove_statistical_outlier(nb_neighbors=30, std_ratio=0.05)
            depth_pcd = depth_pcd.select_by_index(ind)
        # guide hex wrench y
        elif id == 12:
            min_val = np.min(depth_pcd.points,axis=0)
            min_list = np.where(np.array(depth_pcd.points)[:,2] > (min_val[2] + 0.005))
            depth_pcd = depth_pcd.select_by_index(np.array(min_list[0]))
        else:
            min_val = np.min(depth_pcd.points,axis=0)
            min_list = np.where(np.array(depth_pcd.points)[:,2] > (min_val[2] + 0.001))
            depth_pcd = depth_pcd.select_by_index(np.array(min_list[0]))

        return depth_pcd
    
    def get_mesh_angle_pcd(self, id:int, depth_pcd:o3d.geometry.PointCloud, angle_list:np.array=np.array([0, 0, 0]), is_guide:bool = False):
        """ Convert mesh to pcd as open3d format. Match the position of the mesh pointcloud to the depth pointcloud and rotate it according to the angle list.

        Args:
            id (int): Id of depth pointcloud.
            depth_pcd (o3d.geometry.PointCloud): Depth pcd from ICP.get_depth_pcd() function.
            angle_list (np.array, optional): Angle to rotate mesh. Defaults to np.array([0, 0, 0]).
            is_guide (bool, optional): Determine whether the corresponding mesh is a guide or an assembly. Defaults to False.

        Returns:
            o3d.geometry.PointCloud: Pointcloud from mesh.
            np.array: Transform matrix of mesh [4*4].
        """
        
        # Import mesh
        mesh = o3d.io.read_triangle_mesh(self.mesh_dir + "/" + CLASS[id]['name'] + CLASS[id]['type'])
        mesh.scale(0.001, center=([0, 0, 0]))
        # Mesh to pcd
        mesh_pcd = mesh.sample_points_uniformly(number_of_points = CLASS[id]['number_of_points'])
        
        if is_guide:
            # Rotation
            # TODO mesh의 rotation 이후의 eigen vector 방향이 반영되어야 하는데 그렇지 못함.
            eigen_angle = self.eigen_angle(mesh_pcd)
            angle_set = np.array([np.pi/2, 0, -eigen_angle]) + angle_list
            # Generate rotation matrix
            matrix = tf.transformations.euler_matrix(angle_set[0], angle_set[1], angle_set[2])
            # Translate
            # Move mesh pointcloud center to depth pointcloud center.
            matrix[0:3,3] = depth_pcd.get_center() - mesh_pcd.get_center()
            # Shift the mesh pointcloud down slightly for stable ICP.
            matrix[0:3,3] += [0, 0, -0.02]
            
            # Apply transform matrix to mesh pointcloud.
            mesh_pcd.transform(matrix)

            # In the case of wrench y, the sides may be captured strangely, so the center of the depth pointcloud is calculated only in some areas.
            if id == 12:
                max_val = np.max(depth_pcd.points,axis=0)[2]
                max_list = np.where(np.array(depth_pcd.points)[:,2] > (max_val - 0.0015))
                depth_up_pcd = copy.deepcopy(depth_pcd).select_by_index(np.array(max_list[0]))
                trans = depth_up_pcd.get_center() - mesh_pcd.get_center()
                trans[2] = 0
                matrix[0:3,3] += trans
                mesh_pcd.translate(trans)

            # Considering the case where the floor is held together, only a certain height based on the z-axis is used.
            min_val = np.min(mesh_pcd.points,axis=0)
            min_list = np.where(np.array(mesh_pcd.points)[:,2] > (min_val[2] + 0.001))
            mesh_pcd = mesh_pcd.select_by_index(np.array(min_list[0]))
            
        else:
            # rotation
            # TODO mesh의 rotation 이후의 eigen vector 방향이 반영되어야 하는데 그렇지 못함.
            if id == 0:
                eigen_angle = 0
            else:
                eigen_angle = self.eigen_angle(mesh_pcd)
            angle_set = np.array([np.pi/2, 0, -eigen_angle]) + angle_list
            # Generate rotation matrix
            matrix = tf.transformations.euler_matrix(angle_set[0], angle_set[1], angle_set[2])

            # transform
            # Move mesh pointcloud center to depth pointcloud center.
            matrix[0:3,3] = depth_pcd.get_center() - mesh_pcd.get_center()
            # Shift the mesh pointcloud down slightly for stable ICP.
            matrix[0:3,3] += [0, 0, -0.015]

            # Apply transform matrix to mesh pointcloud.
            mesh_pcd.transform(matrix)

            # Considering the case where the floor is held together, only a certain height based on the z-axis is used.
            min_val = np.min(mesh_pcd.points,axis=0)
            if id == 1:
                min_list = np.where(np.array(mesh_pcd.points)[:,2] > (min_val[2] + 0.001))
            elif id == 3:
                min_list = np.where(np.array(mesh_pcd.points)[:,2] > (min_val[2] + 0.001))
            else: 
                min_list = np.where(np.array(mesh_pcd.points)[:,2] > (min_val[2] + 0.004))
            mesh_pcd = mesh_pcd.select_by_index(np.array(min_list[0]))
        return mesh_pcd, matrix

    def run_icp(self, id:int, depth_pcd:o3d.geometry.PointCloud, is_guide:bool = True):
        """ Set the ICP to run according to the predetermined case.

        Args:
            id (int): Id of depth pointcloud.
            depth_pcd (o3d.geometry.PointCloud): Depth pcd from ICP.get_depth_pcd() function.
            is_guide (bool, optional): Determine whether the corresponding mesh is a guide or an assembly. Defaults to False.

        Returns:
            np.array: _description_
        """

        eigen_angle = self.eigen_angle(depth_pcd)
        
        min_cost   = 1000.0
        min_pose   = np.eye(4)
        min_matrix = np.eye(4)
        min_mesh_pcd = None
        guide_idx = id

        if id == 0:
            min_val = np.min(depth_pcd.points,axis=0)[2]
            max_list = np.where(np.array(depth_pcd.points)[:,2] > (min_val + 0.01))
            depth_up_pcd = copy.deepcopy(depth_pcd).select_by_index(np.array(max_list[0]))
            eigen_angle = self.eigen_angle(depth_up_pcd)

        if is_guide:
            print("Guide")
            if CLASS[id]['rotate'] > 10:
                angle_set = np.pi / 4 / ((CLASS[id]['rotate'] - 1) % 10)
            else: 
                angle_set = np.pi * 2 / CLASS[id]['rotate']

            for i in range(CLASS[id]['rotate'] % 10):
                print('case ', i + 1)
                angle = [0, 0, eigen_angle + angle_set * i]
                mesh_pcd, matrix = self.get_mesh_angle_pcd(id, depth_pcd, np.array(angle), is_guide)
                mesh = copy.deepcopy(mesh_pcd)
                pose, cost = self.icp(mesh, depth_pcd)
                if cost < min_cost:
                    min_cost = cost
                    min_matrix = matrix
                    min_pose = pose
                    min_mesh_pcd = copy.deepcopy(mesh_pcd)
                    
        else:
            print("Assmeble")
            for idx, case in enumerate(CLASS[id]['rotate']):
                print('case #', idx + 1)
                if case[1] > 10:
                    angle_set = np.pi / 2 / ((case[1] - 1) % 10)
                else: 
                    angle_set = np.pi * 2 / case[1]
                for i in range(case[1] % 10):
                    angle = np.array(case[0]) + [0, 0, eigen_angle + angle_set * i]
                    mesh_pcd, matrix = self.get_mesh_angle_pcd(id, depth_pcd, np.array(angle), is_guide)
                    mesh = copy.deepcopy(mesh_pcd)
                    pose, cost = self.icp(mesh, depth_pcd)
                    if cost < min_cost:
                        min_cost = cost
                        min_matrix = matrix
                        min_pose = pose
                        min_mesh_pcd = copy.deepcopy(mesh_pcd)
                        guide_idx = case[2]

        min_mesh_pcd = min_mesh_pcd.voxel_down_sample(voxel_size=CLASS[id]['voxel_size']/10)
        return min_mesh_pcd, min_pose, min_matrix, guide_idx

    def icp(self, source, target):
        """_summary_

        Args:
            source (_type_): _description_
            target (_type_): _description_

        Returns:
            _type_: _description_
        """
        target_points = np.asarray(target.points)

        # While loop variables
        curr_iteration = 0
        curr_cost = 1000
        prev_cost = 10000 
        cost_change_threshold = 0.0000001
        total_transform_matrix = np.eye(4)

        while (True):
            # 1. Find nearest neighbors
            new_source_points = self.find_nearest_neighbors(source, target, 3)

            # 2. Find point cloud centroids and their repositions
            source_centroid = np.mean(new_source_points, axis=0)
            target_centroid = np.mean(target_points, axis=0)
            source_repos = np.zeros_like(new_source_points)
            target_repos = np.zeros_like(target_points)
            source_repos = np.asarray([new_source_points[ind] - source_centroid for ind in range(len(new_source_points))])
            target_repos = np.asarray([target_points[ind] - target_centroid for ind in range(len(target_points))])

            # 3. Find correspondence between source and target point clouds
            cov_mat = np.zeros((3,3))
            cov_mat[:2,:2] = (target_repos.transpose() @ source_repos)[:2,:2]

            U, X, Vt = np.linalg.svd(cov_mat)
            R = U @ Vt
            t = target_centroid - R @ source_centroid
            t = np.reshape(t, (1,3))
            curr_cost = np.linalg.norm(target_repos - (R @ source_repos.T).T)
            # print("Curr_cost=", curr_cost)
            if ((prev_cost - curr_cost) > cost_change_threshold):
                prev_cost = curr_cost
                transform_matrix = np.hstack((R, t.T))
                transform_matrix = np.vstack((transform_matrix, np.array([0, 0, 0, 1])))
                # If cost_change is acceptable, update source with new transformation matrix
                source = source.transform(transform_matrix)
                total_transform_matrix = total_transform_matrix @ transform_matrix
                curr_iteration += 1
            else:
                break
        print("Curr_cost=", curr_cost)
        print("Iteration=", curr_iteration)
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
        depth_pcd.paint_uniform_color([1, 0, 0])
        mesh = copy.deepcopy(mesh_pcd).transform(mesh_transpose)
        mesh.paint_uniform_color([0.1, 0.1 ,0.1])
        vis.add_geometry(depth_pcd)
        vis.add_geometry(mesh)
        depth_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=depth_pcd.get_center())
        mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=mesh.get_center())
        origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
        vis.add_geometry(depth_frame)

        vis.run()


    @staticmethod
    def vis_pcds(pcd):
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        for cloud in pcd:
            pcd_o3d = o3d.geometry.PointCloud()
            pcd_o3d.points = o3d.utility.Vector3dVector(cloud)
            vis.add_geometry(pcd_o3d)
            pcd_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=pcd_o3d.get_center())
            vis.add_geometry(pcd_frame)

        origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
        vis.add_geometry(origin_frame)

        vis.run()

    @staticmethod
    def get_class_name(id):
        return CLASS[id]["name"]
    
    @staticmethod
    def rotation_matrix(list):
        return tf.transformations.euler_matrix(list[0], list[1], list[2])
    
    @staticmethod
    def eigen_angle(depth_pcd):
        array = np.asarray(depth_pcd.points)[:,:2]
        eigen_value, vector = np.linalg.eig(np.cov(array[:,0], array[:,1]))
        vector = vector[np.argmax(eigen_value)]
        return -np.arctan2(vector[1], vector[0])
    
    def crop_mesh_pointcloud(self, id):
        mesh = o3d.io.read_triangle_mesh(self.mesh_dir + "/" + CLASS[id]['name'] + CLASS[id]['type'])
        mesh.scale(0.001, center=([0, 0, 0]))
        # mesh to pcd
        mesh_pcd = mesh.sample_points_uniformly(number_of_points = CLASS[id]['number_of_points'])
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        vis.add_geometry(mesh_pcd)
        origin_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.01, origin=[0, 0, 0])
        vis.add_geometry(origin_frame)
        vis.run()

        vol = o3d.visualization.SelectionPolygonVolume()
        vol.orthogonal_axis = "Z"
        vol.axis_min = -2000
        vol.axis_max = 4000

        id = 12 
        mesh2 = o3d.io.read_triangle_mesh(self.mesh_dir + "/" + CLASS[id]['name'] + CLASS[id]['type'])
        mesh_pcd2 = mesh.sample_points_uniformly(number_of_points = 5000)

        vol.bounding_polygon = o3d.utility.Vector3dVector(mesh_pcd.points)

        cropped_pcd = vol.crop_point_cloud(mesh_pcd2)
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        vis.add_geometry(cropped_pcd)
        vis.run()

if __name__ == '__main__':
    icp = ICP(0.001, 0.0000001)
    icp.crop_mesh_pointcloud(6)
