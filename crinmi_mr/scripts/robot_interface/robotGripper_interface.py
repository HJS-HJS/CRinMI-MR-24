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
        print("Move width (m)", width)
        width_transformed = int((3997.5 - width * 48750))
        print(width_transformed)
        self.gripper.Move(width_transformed)
        return
        
    def GripperTest(self):
        
        for i in range(5):
            width = (i + 1) * 0.01
            self.gripper.GripperMove(width)
            time.sleep(5)
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

if __name__ == '__main__':
    controller = GripperControlServer('192.168.3.113')
    controller.GripperMoveGrip()
    controller.GripperMoveRelease()
    # controller.GripperTest()
    time.sleep(5)
    controller.GripperMove(0.02)
    time.sleep(5)
    controller.GripperMoveGrip()
    time.sleep(1)
    controller.GripperMoveRelease()
    time.sleep(5)
    controller.GripperMove(0.03)
    time.sleep(5)
    controller.GripperMoveGrip()
    time.sleep(1)
    controller.GripperMoveRelease()
    time.sleep(5)
    controller.GripperMove(0.01)
    # time.sleep(1.0)
    # controller.GripperMove(500)
    # time.sleep(1.0)
    # controller.GripperMoveGrip()
    # controller.GripperMove(1100)
    # time.sleep(1.0)
    # controller.GripperMove(2000)
    # time.sleep(1.0)
    # controller.GripperMove(3000)
    # time.sleep(1.0)
    # controller.GripperMove(4000)