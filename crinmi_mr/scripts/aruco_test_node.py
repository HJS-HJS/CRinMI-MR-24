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
from utils.utils import *

class Test(object):
    
    def __init__(self):

        # get parameter
        config_file = os.path.abspath(os.path.join(rospkg.RosPack().get_path('crinmi_mr'),'config'))
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
        # ========= tf marker posision test =========
        self.tf_interface = TFInterface(self.workspace_config)
        rospy.loginfo('TF Interface Ready')
        # ========= tf marker posision test =========
        marker_dir = "/aruco/capture_pose1.npz"
        aruco_list = ["4", "5", "29", "37", "40"]
        target_idx = "37"
        self.marker_set           = np.load(config_file + marker_dir)
        for id in aruco_list:
            self.tf_interface.add_stamp("camera_color_optical_frame", "marker" + id, self.marker_set["marker_" + id + "_pose.npy"], m = True, deg = False)

        
        rospy.sleep(1)
        robot_state = robot_server.RecvRobotState()
        self.tf_interface.set_tf_pose(self.tf_interface.tf_base2eef, robot_state, m = True, deg = True)

        vis = VisualizeInterface()
        pcd = camera.pcd(self.tf_interface.matrix(target="base_link", source="camera_depth_optical_frame"))
        vis.pub_pcd(pcd[np.arange(1,pcd.shape[0],1)])

        while True:
            user_input = input('Press enter to start, q to quit...')
            if user_input == 'q':
                break
            elif user_input == '':
                self.execution(self.tf_interface.matrix(target="base_link", source="marker"+target_idx))
            else:
                pass


    def SpinOnce(self):
        # self.tf_interface.broadcast()
        pass

    def create_target_poses(self, marker_pose):
        '''
        Return target poses(unit: mm)
        Target poses are aligned with the z-axis to marker pose

        target_pose: offset(260mm) in z-axis from marker_pose to avoid gripper collision
        pre_target_pose: offset(90mm) in z-axis from target_pose to avoid moving collision
        
        '''

        # create base pose aligned with marker pose's z-axis
        base_pose = np.eye(4)
        base_pose[:3,:3] = rotation(90, 0, -37)

        # calculate target_pose
        target_pose = base_pose
        target_pose[:3, 3] = marker_pose[: 3]
        target_pose = target_pose[2, 3] - 260
        
        # calculate pre_target_pose
        pre_target_pose = target_pose[2, 3] - 350
        
        return pre_target_pose, target_pose

    def execution(self, marker_pose):
        '''
        Execute scenario

        1. Read marker poses
        2. Move to home pose
        3. Move to pre target pose
        4. Move to target pose
        '''

        # Setting robot velocity
        rospy.loginfo('Set Velocity 10')
        self.robot_server.SetVelocity(10)

        # Calculate target poses
        rospy.loginfo('Create target poses')
        pre_target_pose, target_pose = self.create_target_poses(marker_pose=marker_pose)

        # Move to home_pose
        rospy.sleep(2)
        rospy.loginfo('Move to Home pose using MoveJ')
        self.robot_server.RobotMoveJ(self.pose_config["home_pose"])
        while not self.robot_server.wait:
            rospy.sleep(1)

        # Move to pre_target_pose
        rospy.sleep(5)
        rospy.loginfo('Move to pre_target_pose pose using MoveL')
        self.robot_server.RobotMoveL(pre_target_pose)
        rospy.sleep(1)
        while not self.robot_server.wait:
            rospy.sleep(1)
        
        # Move to target_pose
        rospy.sleep(5)
        rospy.loginfo('Move to target_pose pose using MoveL')
        self.robot_server.RobotMoveL(target_pose)
        rospy.sleep(1)
        while not self.robot_server.wait:
            rospy.sleep(1)

        return
    
if __name__ == '__main__':
    rospy.init_node('crinmi_mr')
    server = Test()
    rate = rospy.Rate(10)  # 20hz
    while not rospy.is_shutdown():
        server.SpinOnce()
        rate.sleep()

    # try:
    #     pass
    # except KeyboardInterrupt:
    #     robot_server.Disconnect()
    #     gripper_server.Disconnect()
    #     rospy.loginfo("Disconnect robot & gripper server")

