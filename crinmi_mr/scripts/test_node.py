#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import os
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
from utils.utils import *

class Test(object):
    
    def __init__(self):

        # get parameter
        config_file = os.path.abspath(os.path.join(rospkg.RosPack().get_path('crinmi_mr'),'config', 'workspace.yaml'))
        self.workspace_config     = rospy.get_param("~robot")
        self.ip_config            = rospy.get_param("~robot_ip")[str(self.workspace_config)]
        self.pose_config          = rospy.get_param("~robot_pose")

        self.tf_interface = TFInterface(self.workspace_config)
        # tf_interface.base2eef(robot_server.RecvRobotState(), mm = True, deg = False)
        self.tf_interface.base2eef(self.pose_config["parts_capture_pose"][0], mm = False, deg = True)
        self.tf_interface.cam2point(self.pose_config["marker_pose"][0], mm = True, deg = True)

        # ========= RB10 interface test =========
        # robot_server = RobotControlServer(self.ip_config["robot"])
        # rospy.loginfo('Robot Control Server Ready')
        
        # # test1 (start from arbitrary pose & come back to home position)
        # rospy.loginfo('Move to Home pose using MoveJ')
        # robot_server.SetVelocity(20)
        # robot_server.RobotMoveJ(self.pose_config["home_pose"])
        # while not robot_server.wait:
        #     rospy.sleep(1)

        # robot_server.SetVelocity(10)

        # rospy.sleep(1)
        # # # test2 (start from home pose & Move cartesian motion)
        # rospy.loginfo('Move to specific pose using MoveL')
        # for pose in self.pose_config["parts_capture_pose"]:
        #     H = pose2matrix(pose)
        #     robot_server.RobotMoveL(H)
        #     rospy.sleep(1)
        #     while not robot_server.wait:
        #         rospy.sleep(1)

        ## gripper set ##
        # gripper_server = GripperControlServer(self.ip_config["gripper"], 502)
        # gripper_server.GripperMoveGrip()
        # rospy.sleep(5)

        # rospy.sleep(1)
        # m_base2eef = robot_server.RecvRobotState()
        # m_eef2gripper = tf_interface.m_eef2gripper
        # m_eef2gripper[:3,:3] = rotation(-37, 0, 90 ,axes = "ryzx")
        # m_gripper2cam = tf_interface.m_gripper2cam
        # m_cam2point = pose2matrix(self.pose_config["marker_pose"][0], mm=True)
        # m_base2point = m_base2eef @ m_eef2gripper @ m_gripper2cam @ m_cam2point
        # m_base2point[:3,:3] = rotation(90, 0, 37)
        # m_base2point[3][2] += 1.5

        # robot_server.RobotMoveL(m_base2point)
        # rospy.sleep(1)
        # while not robot_server.wait:
        #     rospy.sleep(1)

        # m_base2point[3][2] -= 1.0
        # robot_server.RobotMoveL(m_base2point)
        # rospy.sleep(1)
        # while not robot_server.wait:
        #     rospy.sleep(1)

    def SpinOnce(self):
        self.tf_interface.broadcast()
    
if __name__ == '__main__':
    rospy.init_node('crinmi_mr')
    server = Test()
    
    rate = rospy.Rate(10)  # 20hz
    while not rospy.is_shutdown():
        server.SpinOnce()
        rate.sleep()

    try:
        pass
    except KeyboardInterrupt:
        robot_server.Disconnect()
        gripper_server.Disconnect()
        rospy.loginfo("Disconnect robot & gripper server")

