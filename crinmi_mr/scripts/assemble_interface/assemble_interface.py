# from icp import ICP
from assemble_interface.icp import ICP

class AssembleInterface():
    def __init__(self):
        pass
    def get_pose(self, pcd, id):
        print("target:\t", ICP.get_class_name(id))
        depth_pcd = ICP.get_depth_pcd(pcd)
        mesh_pcd = ICP.get_mesh_pcd(id, depth_pcd)
        ICP.vis_pcd(depth_pcd, mesh_pcd)
        transpose = ICP.run_icp(depth_pcd, mesh_pcd, id)
        ICP.vis_pcd(depth_pcd, mesh_pcd, transpose)
        # ICP.vis_pcd(depth_pcd, mesh_pcd, transpose)

        pass

