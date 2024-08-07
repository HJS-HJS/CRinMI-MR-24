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
from calibration_interface.calibration_interface import CalibrationInterface

class Test(object):
    
    def __init__(self):

        # get parameter
        config_file = os.path.abspath(os.path.join(rospkg.RosPack().get_path('crinmi_mr'),'config', 'workspace.yaml'))
        self.workspace_config     = rospy.get_param("~robot")

        # ========= camera interface test =========
        calibration = CalibrationInterface(self.workspace_config)
        rospy.loginfo('ready')
        print(calibration.matrix)
        # ========= camera interface test END =========

        # ========= camera interface test =========
        camera = CameraInterface()
        rospy.loginfo('Camera Interface Ready')
        rospy.sleep(1)
        camera.show_intrinsic()
        camera.vis_image()
        # ========= camera interface test END =========

        # ========= RB10 interface test =========
        home_joint_array = [-np.pi/2.0, 0.0, np.pi/2.0, 0.0, np.pi/2.0, 37.0*np.pi/180.0]
        robot_server = RobotControlServer()
        rospy.loginfo('Robot Control Server Ready')
        rospy.sleep(3)
        
        # test1 (get current robot state)
        rospy.loginfo('Recieve current robot state')
        robot_server.RecvRobotState() 
        rospy.sleep(3)
        
        # test2 (start from arbitrary pose & come back to home position)
        rospy.loginfo('Move to Home pose using MoveJ')
        robot_server.RobotMoveJ(home_joint_array)
        rospy.sleep(3)

        # test3 (start from home pose & Move cartesian motion)
        rospy.loginfo('Move to specific pose using MoveL')
        R = [[-1, 0, 0], [0, 0, 1], [0, 1, 0]]
        np_R = np.array(R, dtype=np.float32)
        np_T = np.array([0.179784, -0.692979, 0.4])
        robot_server.RobotMoveL(np_R, np_T)
        rospy.sleep(3)

        # test4 (move Cartesian motion with blend)
        rospy.loginfo('Move to consecutive specific pose using MoveB')
        np_R = np.vstack((np_R, np_R))
        np_T = np.vstack((np_T, np.array([0.179784, -0.692979, 0.1])))
        np_R = np.vstack((np_R, np_R))
        np_T = np.vstack((np_T, np.array([0.179784, -0.692979, 0.4])))
        np_R = np.vstack((np_R, np_R))
        np_T = np.vstack((np_T, np.array([-0.151513, -0.637172, 0.4])))
        np_R = np.vstack((np_R, np_R))
        np_T = np.vstack((np_T, np.array([-0.151513, -0.637172, 0.1])))
        robot_server.RobotMoveB(np_R, np_T)
        rospy.sleep(3)

        # test5 (change default velocity)
        rospy.loginfo('Move to home pose with slower velocity')
        robot_server.SetVelocity(5)
        robot_server.RobotMoveJ(home_joint_array)
        rospy.sleep(3)
        # ========= RB10 interface test END =========

        # ========= RB10 gripper interface test =========
        gripper_server = GripperControlServer()
        rospy.loginfo('Robot Gripper Server Ready')
        rospy.sleep(3)

        rospy.loginfo('Recieve current gripper width')
        gripper_server.RecvGripperWidth()
        rospy.sleep(3)

        rospy.loginfo('Gripper grip object')
        gripper_server.GripperMoveGrip()
        rospy.sleep(3)

        rospy.loginfo('Gripper release object')
        gripper_server.GripperMoveRelease()
        rospy.sleep(3)

        rospy.loginfo('TEST END')

        # ========= RB10 gripper interface test END =========
        
    
if __name__ == '__main__':
    rospy.init_node('crinmi_mr')
    server = Test()
    
    rospy.spin()

    try:
        pass
    except KeyboardInterrupt:
        robot_server.Disconnect()
        gripper_server.Disconnect()
        rospy.loginfo("Disconnect robot & gripper server")

