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
        self.robot_server = RobotControlServer(self.ip_config["robot"])
        # ========= RB10 interface test =========
        self.gripper_server = GripperControlServer(self.ip_config["gripper"], 502)
        # ========= camera interface test =========
        camera = CameraInterface()
        rospy.loginfo('Camera Interface Ready')
        # ========= tf marker posision test =========
        self.tf_interface = TFInterface(self.workspace_config)
        rospy.loginfo('TF Interface Ready')
        # ========= IMPORT Calibration Interface =========
        self.calibration = CalibrationInterface()
        rospy.loginfo('Calibration Interface Ready')


    def grip(self):
        self.robot_server.RobotMoveJ(self.pose_config[str(self.workspace_config)]["home_pose"])
        rospy.sleep(1)
        while not self.robot_server.wait:
            print("wait")
            rospy.sleep(1)

        self.gripper_server.GripperMoveRelease()
        rospy.sleep(1)
        self.gripper_server.GripperMoveGrip()
        rospy.sleep(1)

    def cali(self, assemble:str):
        self.robot_server.RobotMoveJ(self.pose_config[str(self.workspace_config)]["home_pose"])
        rospy.sleep(1)
        while not self.robot_server.wait:
            print("wait")
            rospy.sleep(1)

        if assemble == 'a':
            pose = np.array(self.pose_config[str(self.workspace_config)]['assemble_capture_pose'])
            is_assemble = True
        elif assemble == 'g':
            pose = np.array(self.pose_config[str(self.workspace_config)]['guide_capture_pose'])
            is_assemble = False
        self.robot_server.RobotMoveL(pose)
        rospy.sleep(1)
        while not self.robot_server.wait:
            print("wait")
            rospy.sleep(1)

        robot_state = self.robot_server.RecvRobotState()
        self.tf_interface.set_tf_pose(self.tf_interface.tf_base2eef, robot_state, m = True, deg = True)
        
        rospy.sleep(1)
        base2cam = self.tf_interface.matrix("base_link", "camera_calibration")

        # Update workspace
        self.calibration.update_yaml_workspace(base2cam, "/home/rise/catkin_ws/src/keti/crinmi/CRinMI_MR/crinmi_mr/scripts/calibrate_interface/capture_pose.npz", is_assemble)

        # Update offsets
        self.calibration.update_yaml_offsets("/home/rise/catkin_ws/src/keti/crinmi/CRinMI_MR/crinmi_mr/config/workspace.yaml", is_assemble)
        
        rospy.sleep(1)

    def test(self, assemble:str):
        self.gripper_server.GripperMoveRelease()
        rospy.sleep(1)
        self.gripper_server.GripperMoveGrip()
        rospy.sleep(1)
        if assemble == 'a':
            home_pose = np.array(self.pose_config[str(self.workspace_config)]['assemble_capture_pose'])
            is_assemble = True
        elif assemble == 'g':
            home_pose = np.array(self.pose_config[str(self.workspace_config)]['guide_capture_pose'])
            is_assemble = False

        translate_set =[
             [0.35134845, -0.79415213, 0.01944302],
             [0.28806185, -0.63775786, 0.01706747],
             [0.36244581, -0.59076256, 0.01697350],
             [0.35564148, -0.86935763, 0.02809093],
             [0.28266254, -0.73858798, 0.01663999],
             [0.35881405, -0.69877554, 0.01762767],

        ]

        offset, translate = self.calibration.calculate_offset(translate_set[0], assemble = is_assemble)
        pose = pose2matrix([translate[0], translate[1], translate[2] + 0.246, 90, 0, 42.8])
        pose[2,3] += 0.30
        self.robot_server.RobotMoveL(pose)
        rospy.sleep(1)
        while not self.robot_server.wait:
            print("wait")
            rospy.sleep(1)
        rospy.sleep(1)


        for translate in translate_set:
            offset, translate = self.calibration.calculate_offset(translate, assemble = is_assemble)
            pose = pose2matrix([translate[0], translate[1], translate[2] + 0.246, 90, 0, 42.8])
            pose[2,3] += 0.03

            self.robot_server.RobotMoveL(pose)
            rospy.sleep(1)
            while not self.robot_server.wait:
                print("wait")
                rospy.sleep(1)
            rospy.sleep(5)

        offset, translate = self.calibration.calculate_offset(translate_set[-1], assemble = is_assemble)
        pose = pose2matrix([translate[0], translate[1], translate[2] + 0.246, 90, 0, 42.8])
        pose[2,3] += 0.30
        self.robot_server.RobotMoveL(pose)
        rospy.sleep(1)
        while not self.robot_server.wait:
            print("wait")
            rospy.sleep(1)
        rospy.sleep(1)

        self.robot_server.RobotMoveL(home_pose)
        rospy.sleep(1)
        while not self.robot_server.wait:
            print("wait")
            rospy.sleep(1)
        rospy.sleep(5)


if __name__ == '__main__':
    rospy.init_node('crinmi_mr')
    server = Test()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        user_input = input('g: grip\nca: cali assemble\ncg: cali guide\nta:test assemble\ntg:test guide\n')
        if user_input == 'q':
            break
        elif user_input == 'g':
            server.grip()
        elif user_input == 'ca':
            server.cali('a')
        elif user_input == 'cg':
            server.cali('g')
        elif user_input == 'ta':
            server.test('a')
        elif user_input == 'tg':
            server.test('g')