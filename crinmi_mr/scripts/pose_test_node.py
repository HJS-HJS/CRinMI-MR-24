#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import os
import math
import numpy as np
import matplotlib.pyplot as plt
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
from assemble_interface.assemble_interface import AssembleInterface

class Test(object):    
    def __init__(self):
        # get parameter
        config_file = os.path.abspath(os.path.join(rospkg.RosPack().get_path('crinmi_mr'),'config'))
        save_dir    = os.path.abspath(os.path.join(rospkg.RosPack().get_path('crinmi_mr'),'data'))
        self.workspace_config     = rospy.get_param("~robot")
        self.ip_config            = rospy.get_param("~robot_ip")[str(self.workspace_config)]
        self.pose_config          = rospy.get_param("~robot_pose")
        # self.match_tf_config      = rospy.get_param("~match_pose")
        # self.grip_tf_config       = rospy.get_param("~match_grip")   
        
        self.gripper_server = GripperControlServer(self.ip_config["gripper"], 502)

        self.robot_server = RobotControlServer(self.ip_config["robot"])
        rospy.loginfo('Robot Control Server Ready')
        # ========= camera interface test =========
        camera = CameraInterface()
        rospy.loginfo('Camera Interface Ready')
        # ========= tf marker posision test =========
        self.tf_interface = TFInterface(self.workspace_config)
        rospy.loginfo('TF Interface Ready')
        # ========= data save interface test =========
        self.data_save = DataSaveInterface(save_dir)
        rospy.loginfo('Data Save Interface Ready')
        # ========= assemble posision test =========
        self.assemble = AssembleInterface()
        rospy.loginfo('Assemble Interface Ready')

        self.gripper_server.GripperMoveGrip()
        # Generate TF msg
        rospy.sleep(1)
        robot_state = self.robot_server.RecvRobotState()
        self.tf_interface.set_tf_pose(self.tf_interface.tf_base2eef, robot_state, m = True, deg = True)
        rospy.sleep(0.5)
        # vis = VisualizeInterface  ()
        # camera.read_image("0070")
        # vis.pub_mesh()


        base2cam = self.tf_interface.matrix('base_link', 'camera_calibration')
        # print('LEFT POSE')
        # print(base2cam @ np.array([ -0.08674788, -0.17385123,  0.65928375, 1]))
        # print(base2cam @ np.array([ -0.08654046, -0.00559269,  0.66089516, 1]))
        # print(base2cam @ np.array([ -0.0833861,   0.17033368,  0.65371758, 1]))

        # print('RIGHT POSE')
        # print(base2cam @ np.array([ 0.09496325, -0.17252077,  0.65590934 , 1]))
        # print(base2cam @ np.array([ 0.09587742, -0.00417192,  0.6569243, 1]))
        # print(base2cam @ np.array([ 0.09436888,  0.17021987,  0.64664289, 1]))
        
        print(base2cam @ np.array([-0.05836069, 0.06045932, 0.76822921, 1]))
        print(base2cam @ np.array([-0.19016622, -0.01752662, 0.80027995, 1]))
        print(base2cam @ np.array([-0.01992643, -0.04271945, 0.78222865, 1]))
        print(base2cam @ np.array([0.0317865, 0.15514228, 0.73019495, 1]))
        print(base2cam @ np.array([0.06610551, 0.05619921, 0.75518468, 1]))

        rospy.sleep(1)
            
        while True:
            user_input = input('Press enter to record, q to quit...')
            if user_input == 'q':
                break
            elif user_input == '':
                break                
            else:
                pass

    def SpinOnce(self):
        # self.tf_interface.broadcast()
        pass
    
if __name__ == '__main__':
    rospy.init_node('crinmi_mr')
    server = Test()
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        server.SpinOnce()
        rate.sleep()

    try:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Disconnect robot & gripper server")

