#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import sys
import os

current_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_dir)

from camera_interface.camera_interface import CameraInterface

class Test(object):
    
    def __init__(self):
        # camera interface test
        camera = CameraInterface()
        rospy.loginfo('ready')
        rospy.sleep(1)
        camera.show_intrinsic()
        camera.vis_image()
    
if __name__ == '__main__':
    rospy.init_node('crinmi_mr')
    server = Test()
    
    rospy.spin()
