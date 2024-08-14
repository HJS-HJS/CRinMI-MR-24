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
        self.robot_server = RobotControlServer(self.ip_config["robot"])
        rospy.loginfo('Robot Control Server Ready')
        
        # ========= tf marker posision test =========
        # temp marker_set for test
        target_id = 37
        self.marker_set           = np.load(config_file + "/aruco/capture_pose1.npz")
        marker = self.marker_set["marker_{}_pose".format(target_id)]
        # temp robot state for test
        robot_state = np.array([
            [0.54718528, 0.05478036, 0.8352169, 0.20298141],
            [0.8358647, 0.01645447, -0.54868885, -0.65046209],
            [-0.04380043, 0.99836284, -0.03678533, 0.75620735],
            [0, 0, 0 ,1],
        ])
        # robot_state = robot_server.RecvRobotState()


        self.tf_interface = TFInterface(self.workspace_config)
        self.tf_interface.base2eef(robot_state, mm = True, deg = True)
        self.tf_interface.cam2marker(marker, mm = True, deg = True)
        rospy.sleep(1)
        # marker_pose = self.tf_interface.matrix(target="base_link", source="marker")
        # print("base_link to marker")
        # print(marker_pose)

        camera = CameraInterface()
        camera.read_image('1')
        # camera.vis_pcd(self.tf_interface.matrix(target="base_link", source="camera_link"))

        vis = VisualizeInterface()
        pcd = camera.pcd(self.tf_interface.matrix(target="base_link", source="camera_link"))
        vis.pub_pcd(pcd[np.arange(1,pcd.shape[0],1)])


        # # test1 (start from arbitrary pose & come back to home position)
        # rospy.loginfo('Move to Home pose using MoveJ')
        # robot_server.SetVelocity(20)
        # robot_server.RobotMoveJ(self.pose_config["home_pose"])
        # while not robot_server.wait:
        #     rospy.sleep(1)

        # robot_server.SetVelocity(10)

        # rospy.sleep(1)
        # # # test2 (start from home pose & Move cartesian motion)
        # rospy.loginfo('Move to specific pose using MoveL')
        # for pose in self.pose_config["parts_capture_pose"]:
        #     H = pose2matrix(pose)
        #     robot_server.RobotMoveL(H)
        #     rospy.sleep(1)
        #     while not robot_server.wait:
        #         rospy.sleep(1)

        # gripper set ##
        # gripper_server = GripperControlServer(self.ip_config["gripper"], 502)
        # gripper_server.GripperMoveGrip()
        # rospy.sleep(5)

        # robot_server.RobotMoveL(m_base2marker)
        # rospy.sleep(1)
        # while not robot_server.wait:
        #     rospy.sleep(1)

    def SpinOnce(self):
        # self.tf_interface.broadcast()
        pass

    def create_target_poses(self, marker_pose):
        '''
        Return target poses(unit: mm)
        Target poses are aligned with the z-axis to marker pose

        target_pose: offset 5mm in z-axis from marker_pose
        pre_target_pose: offset gripper length(246mm) in z-axis from target_pose
        
        '''

        # create base pose aligned with marker pose's z-axis
        base_pose = np.eye(4)
        base_pose[:3,:3] = rotation(90, 0, -37)

        # calculate target_pose
        target_pose = base_pose
        target_pose[:3, 3] = marker_pose[: 3]
        target_pose = target_pose[2, 3] - 350
        
        # calculate pre_target_pose
        pre_target_pose = target_pose[2, 3] - 260
        
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

