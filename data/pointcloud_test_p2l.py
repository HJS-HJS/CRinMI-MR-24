import numpy as np
import open3d as o3d
from sklearn.neighbors import KDTree

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



np_pcd = np.load("/home/rise/catkin_ws/src/crinmi/data/64/" + str(6) + ".npy")
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np_pcd)
voxel_down_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
cl, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=200, std_ratio=0.02)
pcd1 = voxel_down_pcd.select_by_index(ind)




mesh2 = o3d.io.read_triangle_mesh("/home/rise/catkin_ws/src/crinmi/data/mesh_file/" + name + ".obj")
mesh2.scale(0.254, center=pcd1.get_center())

# R = mesh2.get_rotation_matrix_from_xyz((np.pi/3, np.pi/3, 0))
R = mesh2.get_rotation_matrix_from_xyz((0, 0, np.pi))
mesh2.rotate(R, center=pcd1.get_center())

pcd2 = mesh2.sample_points_uniformly(number_of_points = 10000)
pcd2 = pcd2.voxel_down_sample(voxel_size=voxel_size)

pcd1.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=1000))
pcd2.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=1000))

# icp
threshold = 0.05
trans_init = np.eye(4)

def nearest_neighbor(src, dst):
    tree = KDTree(src, leaf_size=2)
    dist, ind = tree.query(dst, k=1)
    src_neighbored = src[ind].reshape(-1, 3)
    return src_neighbored, dist

# from sklearn.neighbors import KDTree
def normal_vector_set(pc):
    tree = KDTree(pc, leaf_size=2)
    dist, ind = tree.query(pc, k=3)
    n_vectors = []
    for i in range(len(pc)):
        v1 = pc[ind[i, 1]] - pc[i]
        v2 = pc[ind[i, 2]] - pc[i]
        n = np.cross(v1, v2)
        n_vectors.append(n)
    return np.array(n_vectors)

def point2plane(src, dst, norms):
    N = src.shape[0]
    matrix_b = np.zeros((N, 1))
    for i in range(N):
        matrix_b[i] += norms[i, 0]*dst[i, 0] + norms[i, 1]*dst[i, 1] + norms[i, 2]*dst[i, 2]
        matrix_b[i] -= norms[i, 0]*src[i, 0] + norms[i, 1]*src[i, 1] + norms[i, 2]*dst[i, 2]
    matrix_A = np.zeros((N, 6))
    matrix_A[:, 3:] = norms
    for i in range(N):
        matrix_A[i, :3] = np.cross(src[i], norms[i])
        # matrix_A[i, 0] = norms[i, 2]*src[i, 1] - norms[i, 1]*src[i, 2]
        # matrix_A[i, 1] = norms[i, 0]*src[i, 2] - norms[i, 2]*src[i, 0]
        # matrix_A[i, 2] = norms[i, 1]*src[i, 0] - norms[i, 0]*src[i, 1]
    # U, S, Vt = np.linalg.svd(matrix_A, full_matrices=False)
    # S_inv = np.zeros((U.shape[1], Vt.shape[0]))
    # for i in range(len(S)):
    #     S_inv[i, i] = S[i]
    # S_inv = np.linalg.inv(S_inv)
    # matrix_A_inv = Vt.T.dot(S_inv).dot(U.T)
    matrix_A_inv = np.linalg.pinv(matrix_A)
    x_opt = matrix_A_inv.dot(matrix_b)  # alpha, beta ,gamma, tx, ty, tz
    M = np.eye(4)
    M[0, 1] = -x_opt[2]
    M[0, 2] = x_opt[1]
    M[0, 3] = x_opt[3]
    M[1, 0] = x_opt[2]
    M[1, 2] = -x_opt[0]
    M[1, 3] = x_opt[4]
    M[2, 0] = -x_opt[1]
    M[2, 1] = x_opt[0]
    M[2, 3] = x_opt[3]
    return M


def icp(src, dst, tolerance=1e-7):
    m = src.shape[1]
    assert m == 3
    N = min(src.shape[0], dst.shape[0])
    sampler = np.random.sample(range(N), N)
    source = np.ones((m+1, N))
    tar = np.ones((m+1, N))
    source[:m, :] = np.copy(src[sampler, :].T)
    tar[:m, :] = np.copy(dst[sampler, :].T)
    prev_error = 0
    count = 0
    M = np.eye(4)
    while True:
        src, dist = nearest_neighbor(src, dst)
        norms = normal_vector_set(dst)
        T = point2plane(src, dst, norms)    # T = 4x4

        mean_error = np.mean(dist)
        if np.abs(prev_error - mean_error) < tolerance:
            print('Iterate loop:: ', count)
            break
        source = T.dot(source)
        M = M.dot(T)
        src = source[:3, :].T
        prev_error = mean_error
        print(prev_error)
        count += 1
    return src, M

vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(pcd1)
vis.add_geometry(pcd2)
vis.run()


result = icp(pcd2, pcd1)
print(result)


vis = o3d.visualization.Visualizer()
vis.create_window()
pcd2 = pcd2.transform(result)
vis.add_geometry(pcd1)
vis.add_geometry(pcd2)

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
