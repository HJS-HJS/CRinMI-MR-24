#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import os
import rospy
import numpy as np
import time

gripper_dir = os.path.dirname(os.path.realpath(__file__)) + '/gripper'
sys.path.append(gripper_dir)
from zimmergripper import KetiZimmer

class GripperControlServer():
    def __init__(self, gripper_ip='127.0.0.1', gripper_port=502):
        self.gripper = KetiZimmer(gripper_dir + '/libzimmergripper.so')
        self.gripper.Connect(gripper_ip, gripper_port)
        gripper_connected = self.gripper.IsAlive()

        if gripper_connected is True:
            self.gripper.Init()
        else:
            rospy.loginfo("gripper connection is not established!!")

    def RecvGripperWidth(self):
        return self.gripper.CurPos()

    def GripperMove(self, width):
        self.gripper.Move(width)
        return
        
    def GripperMoveGrip(self):
        self.gripper.Grip()
        return 
    
    def GripperMoveRelease(self):
        self.gripper.Release()
        return
        
    def Disconnect(self):
        self.gripper.Disconnect()
        time.sleep(1.0)
        return

    def __del__(self):
        self.gripper.Disconnect()
        time.sleep(1.0)
        return