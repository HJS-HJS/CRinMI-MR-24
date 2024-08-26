import numpy as np
import copy
from assemble_interface.icp import ICP

class AssembleInterface():
    def __init__(self):
        pass

    def get_pose(self, pcd, id):

        print("target:\t", ICP.get_class_name(id))
        if id <= 5:
            icp = ICP(0.001, 0.0000001)
            is_guide = False
        else:
            icp = ICP(0.001, 0.0005)
            is_guide = True

        depth_pcd = icp.get_depth_pcd(pcd, id, is_guide)
        mesh_pcd, translate= icp.get_mesh_pcd(id, depth_pcd)
        mesh_pcd = mesh_pcd.translate([0, 0, -0.02])
        translate[0:3,3] += [0, 0, -0.02]

        pose, mesh_pcd, matrix, guide_idx = icp.run_icp(depth_pcd, id, is_guide)
        # icp.vis_pcd(depth_pcd, mesh_pcd)
        # icp.vis_pcd(depth_pcd, mesh_pcd, pose)

        mesh_pcd.transform(pose)

        return pose @ matrix, mesh_pcd.points, guide_idx
