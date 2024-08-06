#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import sys
import os

current_dir = os.path.dirname(os.path.realpath(__file__))
robotSDK_dir = os.path.dirname(os.path.realpath(__file__) + '/robotSDK_interface/ketirobotsdk')
gripper_dir = os.path.dirname(os.path.realpath(__file__) + '/robotSDK_interface/gripper')
sys.path.append(current_dir)
sys.path.append(robotSDK_dir)
sys.path.append(gripper_dir)

from robotSDK_interface.robotSDK_interface import RobotControlServer
from camera_interface.camera_interface import CameraInterface

from sdk import *
from zimmergripper import KetiZimmer


class Test(object):
    
    def __init__(self):
        # camera interface test
        camera = CameraInterface()
        rospy.loginfo('ready')
        rospy.sleep(1)
        camera.show_intrinsic()
        camera.vis_image()

        # RB10 interface test
        robot_server = RobotControlServer()
        rospy.loginfo('ready')
        rospy.sleep(1)
        robot_server.RecvRobotState()
        robot_server.RobotMoveJ()
        robot_server.RobotMoveL()
        robot_server.RobotMoveB()
        robot_server.SetVelocity(5)

        # RB10 gripper test
        
    
if __name__ == '__main__':
    rospy.init_node('crinmi_mr')
    server = Test()
    
    rospy.spin()
