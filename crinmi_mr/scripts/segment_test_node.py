#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import os
import cv2
import yaml
import numpy as np
import rospy
import rospkg

current_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_dir)

from robot_interface.robotSDK_interface import *
from robot_interface.robotGripper_interface import *
from camera_interface.camera_interface import CameraInterface
from tf_interface.tf_interface import TFInterface
from visualize_interface.visualize_interface import VisualizeInterface
from data_save.data_save import DataSaveInterface
from utils.utils import *
from segment_interface.yolo_segment_interface import *
from assemble_interface.assemble_interface import AssembleInterface

class Test(object):
    
    def __init__(self):

        # get parameter
        config_file = os.path.abspath(os.path.join(rospkg.RosPack().get_path('crinmi_mr'),'config'))
        save_dir    = os.path.abspath(os.path.join(rospkg.RosPack().get_path('crinmi_mr'),'data'))
        self.workspace_config     = rospy.get_param("~robot")
        self.ip_config            = rospy.get_param("~robot_ip")[str(self.workspace_config)]
        self.pose_config          = rospy.get_param("~robot_pose")
        
        # ========= RB10 interface test =========
        # robot_server = RobotControlServer(self.ip_config["robot"])
        rospy.loginfo('Robot Control Server Ready')
        # ========= RB10 interface test =========
        # gripper_server = GripperControlServer(self.ip_config["gripper"], 502)
        rospy.loginfo('Robot Gripper Server Ready')
        # ========= camera interface test =========
        camera = CameraInterface()
        rospy.loginfo('Camera Interface Ready')
        # ========= data save interface test =========
        self.data_save = DataSaveInterface(save_dir)
        rospy.loginfo('Data Save Interface Ready')
        # ========= tf marker posision test =========
        self.tf_interface = TFInterface(self.workspace_config)
        rospy.loginfo('TF Interface Ready')
        # ========= tf marker posision test =========
        self.assemble = AssembleInterface()
        rospy.loginfo('Assemble Interface Ready')

        # Generate TF msg
        # robot state for test
        rospy.sleep(1)
        # robot_state = robot_server.RecvRobotState()
        robot_state = camera.temp_read_state("0061")
        # robot_state = np.array(
        #     [
        #         [ 0.54718528,  0.05478036,  0.83521697,  0.20298141],
        #         [ 0.8358647 ,  0.01645447, -0.54868885, -0.65046209],
        #         [-0.04380043,  0.99836284, -0.03678533,  0.75620735],
        #         [ 0.        ,  0.        ,  0.        ,  1.        ],
        #         ]
        #     )
        self.tf_interface.set_tf_pose(self.tf_interface.tf_base2eef, robot_state, m = True, deg = True)
        rospy.sleep(0.5)

        vis = VisualizeInterface()
        camera.read_image("0061")
        pcd = camera.pcd()
        vis.pub_pcd(pcd[np.arange(1,pcd.shape[0],1)])

        while True:
            user_input = input('Press enter to record, q to quit...')
            if user_input == 'q':
                break
            elif user_input == '':
                color_img = camera.color_img
                segment_server = SegmentInterface()
                segment_server.run(color_img)
                seg = segment_server.img2SegmentMask()
                camera.vis_segment(seg)
                for idx in seg:
                    obj_seg = idx[0]
                    obj_depth = obj_seg * camera.depth_img
                    obj_pcd = camera.depth2pcd(obj_depth, self.tf_interface.matrix("base_link", "camera_calibration"))
                    # obj_pcd = camera.depth2pcd(obj_depth, self.tf_interface.matrix("camera_color_frame", "base_link"))
                    # obj_pcd = camera.depth2pcd(obj_depth)
                    vis.pub_target_pcd(obj_pcd[np.arange(1,obj_pcd.shape[0],5)])
                    pose = self.assemble.get_pose(obj_pcd, idx[-1])
                    # camera.vis_pcd(obj_pcd, reduction_ratio=1) # visualize with matplotlib
                # idx = 6
                # obj_seg = seg[idx][0]
                # obj_depth = obj_seg * camera.depth_img
                # obj_pcd = camera.depth2pcd(obj_depth)
                # pose = self.assemble.get_pose(obj_pcd, seg[idx][-1])
                rospy.sleep(1)
            else:
                pass
    
if __name__ == '__main__':
    rospy.init_node('crinmi_mr')
    server = Test()
    rospy.spin()
    # rate = rospy.Rate(10)  # 20hz
    # while not rospy.is_shutdown():
    #     server.SpinOnce()
    #     rate.sleep()

    try:
        pass
    except KeyboardInterrupt:
        robot_server.Disconnect()
        gripper_server.Disconnect()
        rospy.loginfo("Disconnect robot & gripper server")

