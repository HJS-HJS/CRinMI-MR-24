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
from calibrate_interface.calibration import CalibrationInterface

class Test(object):
    
    def __init__(self):

        # get parameter
        config_file = os.path.abspath(os.path.join(rospkg.RosPack().get_path('crinmi_mr'),'config'))
        save_dir    = os.path.abspath(os.path.join(rospkg.RosPack().get_path('crinmi_mr'),'data'))
        self.workspace_config     = rospy.get_param("~robot")
        self.ip_config            = rospy.get_param("~robot_ip")[str(self.workspace_config)]
        self.pose_config          = rospy.get_param("~robot_pose")
        
        # ========= RB10 interface test =========
        # ========= camera interface test =========
        camera = CameraInterface()
        rospy.loginfo('Camera Interface Ready')
        # ========= tf marker posision test =========
        self.tf_interface = TFInterface(self.workspace_config)
        rospy.loginfo('TF Interface Ready')
        # ========= IMPORT Calibration Interface =========
        self.calibration = CalibrationInterface()
        rospy.loginfo('Calibration Interface Ready')

        # Generate TF msg
        # robot state for test
        rospy.sleep(1)
        # read_num = "0061"
        # read_num = "0062"
        read_num = "depth_0063"
        robot_state = camera.temp_read_state(read_num)
        # robot_state = np.array(
        #     [
        #         [ 0.54718528,  0.05478036,  0.83521697,  0.20298141],
        #         [ 0.8358647 ,  0.01645447, -0.54868885, -0.65046209],
        #         [-0.04380043,  0.99836284, -0.03678533,  0.75620735],
        #         [ 0.        ,  0.        ,  0.        ,  1.        ],
        #         ]
        #     )
        # robot_state = robot_server.RecvRobotState()
        self.tf_interface.set_tf_pose(self.tf_interface.tf_base2eef, robot_state, m = True, deg = True)
        
        base2cam = self.tf_interface.matrix("base_link", "camera_calibration")

        # Assemble/Guide
        assemble = True

        # Update workspace
        self.calibration.update_yaml_workspace(base2cam, "/home/rise/catkin_ws/src/keti/crinmi/CRinMI_MR/crinmi_mr/scripts/calibrate_interface/capture_pose.npz", assemble)

        # Update offsets
        self.calibration.update_yaml_offsets("/home/rise/catkin_ws/src/keti/crinmi/CRinMI_MR/crinmi_mr/config/workspace.yaml", assemble)
        
        rospy.sleep(1)
    
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

