#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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
from visualize_interface.visualize_interface import VisualizeInterface
from data_save.data_save import DataSaveInterface
from utils.utils import *

class Test(object):
    
    def __init__(self):

        # get parameter
        config_file = os.path.abspath(os.path.join(rospkg.RosPack().get_path('crinmi_mr'),'config'))
        save_dir    = os.path.abspath(os.path.join(rospkg.RosPack().get_path('crinmi_mr'),'data'))
        self.workspace_config     = rospy.get_param("~robot")
        self.ip_config            = rospy.get_param("~robot_ip")[str(self.workspace_config)]
        self.pose_config          = rospy.get_param("~robot_pose")
        
        # ========= RB10 interface test =========
        robot_server = RobotControlServer(self.ip_config["robot"])
        rospy.loginfo('Robot Control Server Ready')
        # ========= RB10 interface test =========
        gripper_server = GripperControlServer(self.ip_config["gripper"], 502)
        rospy.loginfo('Robot Gripper Server Ready')
        # ========= camera interface test =========
        camera = CameraInterface()
        rospy.loginfo('Camera Interface Ready')
        # ========= data save interface test =========
        self.data_save = DataSaveInterface(save_dir)
        rospy.loginfo('Data Save Interface Ready')
        # ========= tf marker posision test =========
        self.tf_interface = TFInterface(self.workspace_config)
        rospy.loginfo('TF Interface Ready')

        self.vis = VisualizeInterface()

        # print(np.array(self.pose_config[str(self.workspace_config)]))
        self.guide_top_view = np.array(self.pose_config[str(self.workspace_config)]['guide_capture_pose'])

        while True:
            user_input = input('Press enter to record, q to quit...')
            if user_input == 'q':
                break
            elif user_input == '':
                robot_state = robot_server.RecvRobotState()
                self.tf_interface.set_tf_pose(self.tf_interface.tf_base2eef, robot_state, m = True, deg = True)
                pcd = camera.pcd(np.eye(4))
                self.vis.pub_pcd(pcd[np.arange(1,pcd.shape[0],10)])
                cam_matrix =  self.tf_interface.matrix(target="base_link", source="camera_calibration")
                print('robot_state')
                print(robot_state)
                print('cam_matrix')
                print(cam_matrix)
            else:
                pass



    def SpinOnce(self):
        # self.tf_interface.broadcast()
        pass
    
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

