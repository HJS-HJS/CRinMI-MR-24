#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import os
import rospy
import numpy as np
import time
from dataclasses import dataclass

robotSDK_dir = os.path.dirname(os.path.realpath(__file__)) + '/ketirobotsdk'
sys.path.append(robotSDK_dir)
# setLibPath(os.path.dirname(os.path.realpath(__file__)) + '/ketirobotsdk/librobotsdk.so')

from sdk import *

class RobotControlServer():

    def __init__(self, robot_ip):
        """ Initialize robot connection using given robot_ip.
        Args:
            'str': RB10 robot ip 
        """
        self.rb10 = Robot()
        self.rb10.SetRobotConf(RB10, robot_ip, 5000)
        
        # robot connection check
        robot_connected = self.rb10.RobotConnect() 
        assert robot_connected is True, "robot connection is not established!!"

        # Set default robot velocity
        self.rb10.SetVelocity(10)

        # Update current state of rb10 (wait or moving)
        self.wait = True
        duration = rospy.Duration(0.1)
        timer = rospy.Timer(duration, self._state_update_callback)

    def _state_update_callback(self, event):
        """ 
            Update robot joint, EE pose, moving status (update frequency: 10Hz)
        """
        robotInfo = self.rb10.RobotInfo()
        # update current robot state: '1' indicates state "Wait" & '2' indicates "Moving"
        if robotInfo.State == 1:
            self.wait = True
        else:
            self.wait = False

        # update current joint position (6 joints)
        self.current_joint = np.array([robotInfo.Jnt[0], robotInfo.Jnt[1], robotInfo.Jnt[2], robotInfo.Jnt[3], robotInfo.Jnt[4], robotInfo.Jnt[5]])

        # update current Homogenous transformation (4x4)
        self.current_pose_matrix = np.array([robotInfo.Mat[0], robotInfo.Mat[1], robotInfo.Mat[2], robotInfo.Mat[3],
                                          robotInfo.Mat[4], robotInfo.Mat[5], robotInfo.Mat[6], robotInfo.Mat[7],
                                          robotInfo.Mat[8], robotInfo.Mat[9], robotInfo.Mat[10], robotInfo.Mat[11],
                                          robotInfo.Mat[12], robotInfo.Mat[13], robotInfo.Mat[14], robotInfo.Mat[15]]).reshape(4, 4)

        self.R = self.current_pose_matrix[:3, :3]
        self.T = self.current_pose_matrix[3, :3]

    def RecvRobotState(self):
        """ 
            date robot joint, EE pose, moving status (update frequency: 10Hz)
        """
        # robotInfo = self.rb10.RobotInfo()
        # print("current_state : {0}".format(self.wait))
        # print("current_joint : {0}".format(self.current_joint))
        # print("current_pose_matrix : ")
        # print(self.current_pose_matrix)
        return self.current_pose_matrix
        
    def RobotMoveJ(self, joint_array):
        """ Move each robot joints using sdk libraray

        Args:
            `joint_array`: 6 joint_array values (unit: rad)
        """
        # check joint array type & length 
        assert type(joint_array) == list, "joint_array must be list type!!"
        assert len(joint_array) == 6, "length of joint array list is not 6!!"

        if self.wait is True:
            self.rb10.movej(joint_array)
    
    def RobotMoveL(self, H):
        """ Move robot End effector with cartesian motion using sdk libraray

        Args:
            `R`: rotation matrix relative to robot base frame, input must be type 'numpy.ndarray'
            't': translation matrix relative to robot base frame, input must be type 'numpy.ndarray'
        """
        # check rotation, translation matrix type & length
        assert isinstance(H, np.ndarray), "MoveL homogenous matrix value must be numpy type!!"
        assert H.shape == (4,4), "MoveL rotation matrix must be size 3x3!!"

        # Adjust 4x4 numpy matrix to fit into "movel" input type(1-dim array)
        flatten_H = [element for row in H.tolist() for element in row]
        if self.wait is True:
            self.rb10.movel(0, flatten_H)

    def RobotMoveB(self, H_array):
        """ Move robot End effector with cartesian motion with blending using sdk libraray

        Args:
            `H_array`: rotation matrix relative to robot base frame consecutively, input must be type 'numpy.ndarray'
        """

        # check rotation, translation matrix type & length
        assert isinstance(H_array, np.ndarray), "MoveB homogenous matrix array value must be numpy type!!"

        # internal code of sdk ('moveb') is little-bit changed because of input     
        if self.wait is True:
            self.rb10.moveb(0, 0.02, 5, *H_array.reshape(-1,16).tolist())

    def SetVelocity(self, velocity):
        """ Set robot default speed for moving  
            
            Args:
                `velocity`: Unit for velocity is not sure.. we have to check it for challenge...
        """
        rospy.loginfo("robot default velocity changed... Be careful")
        self.rb10.SetVelocity(velocity)

    def Disconnect(self):
        """ 
           Disconnect robot
        """
        self.rb10.RobotDisconnect()
        time.sleep(1.0)
        return

    def __del__(self):
        """ 
           Destructor to disconnect robot safely
        """
        self.Disconnect()
        return