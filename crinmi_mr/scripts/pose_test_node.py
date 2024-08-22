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

        # Generate TF msg
        rospy.sleep(1)
        robot_state = np.array(self.pose_config["assemble_capture_pose"][1])
        self.tf_interface.set_tf_pose(self.tf_interface.tf_base2eef, robot_state, m = True, deg = True)
        rospy.sleep(0.5)

        vis = VisualizeInterface()
        camera.read_image("0070")
        vis.pub_mesh()

        pose = np.eye(4)
        self.tf_interface.add_stamp("base_link", "asset_1", pose, m = True, deg = False)
        pose = pose2matrix([0, 0, -0.0235, 90, 0, 0])
        self.tf_interface.add_stamp("asset_1", "asset_8", pose, m = True, deg = False)            
        vis.pub_mesh()

        rospy.sleep(1)
        print(self.tf_interface.matrix("asset_8", "asset_1"))
        rospy.sleep(5)

        # # for i in range(0,17):
        #     self.tf_interface.add_stamp("base_link", "asset_0", pose, m = True, deg = False)
        #     pose = pose2matrix(0 -0.3 0, 0,0,0)
        #     self.tf_interface.add_stamp("asset_0", "asset_", pose, m = True, deg = False)            
        #     vis.pub_mesh()
        #     rospy.sleep(5)
        #     # self.tf_interface.broadcast_once()

        vis.pub_mesh()
            
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

