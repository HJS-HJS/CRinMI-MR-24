import numpy as np
import copy
from assemble_interface.icp import ICP

class AssembleInterface():
    def __init__(self):
        pass

    def get_pose(self, pcd, id):

        print("target:\t", ICP.get_class_name(id))

        icp = ICP()
        if id <= 5:
            is_guide = False
        else:
            is_guide = True

        depth_pcd = icp.get_depth_pcd(id, pcd)

        pose, mesh_pcd, matrix, guide_idx = icp.run_icp(id, depth_pcd, is_guide)
        # icp.vis_pcd(depth_pcd, mesh_pcd)
        # icp.vis_pcd(depth_pcd, mesh_pcd, pose)

        mesh_pcd.transform(pose)

        return pose @ matrix, mesh_pcd.points, guide_idx
    @staticmethod
    def show_pointcloud(*args):
        ICP.vis_pcds(args)
    