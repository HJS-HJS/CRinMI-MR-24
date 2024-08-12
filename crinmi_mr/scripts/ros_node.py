#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import sys
import os
import numpy as np
import rospy
import rospkg

current_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_dir)

from robot_interface.robotSDK_interface import *
from robot_interface.robotGripper_interface import *
from camera_interface.camera_interface import CameraInterface
from tf_interface.tf_interface import TFInterface

class MRServer(object):
    
    def __init__(self):

        # get parameter
        config_file = os.path.abspath(os.path.join(rospkg.RosPack().get_path('crinmi_mr'),'config'))
        self.workspace_config     = rospy.get_param("~robot")
        self.ip_config            = rospy.get_param("~robot_ip")[str(self.workspace_config)]
        self.pose_config          = rospy.get_param("~robot_pose")

        # custom module
        # 1. robot
        robot_server = RobotControlServer(self.ip_config["robot"])
        rospy.loginfo('Robot Control Server Ready')
        # 2. gripper
        gripper_server = GripperControlServer(self.ip_config["gripper"], 502)
        gripper_server.GripperMoveGrip()
        # 2.5 camera
        camera = CameraInterface()
        # 3. perception
        # 4. 6D pose
        # 5. assemble planning
        # 6. grip planning
        # 7. calibration
    
    def run(self):
        rospy.loginfo("run the mr code")
        pass
    
if __name__ == '__main__':
    rospy.init_node('crinmi_mr')
    server = MRServer()
    server.run()
    
    rospy.spin()
