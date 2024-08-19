#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import os
import cv2
import yaml
import numpy as np
import matplotlib as plt
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
from segment_interface.yolo_segment_interface import *
from assemble_interface.assemble_interface import AssembleInterface

class Test(object):
    
    def __init__(self):

        # get parameter
        config_file = os.path.abspath(os.path.join(rospkg.RosPack().get_path('crinmi_mr'),'config'))
        save_dir    = os.path.abspath(os.path.join(rospkg.RosPack().get_path('crinmi_mr'),'data'))
        self.workspace_config     = rospy.get_param("~robot")
        self.ip_config            = rospy.get_param("~robot_ip")[str(self.workspace_config)]
        self.pose_config          = rospy.get_param("~robot_pose")
        
        # ========= RB10 interface test =========
        self.robot_server = RobotControlServer(self.ip_config["robot"])
        rospy.loginfo('Robot Control Server Ready')
        # ========= RB10 interface test =========
        self.gripper_server = GripperControlServer(self.ip_config["gripper"], 502)
        rospy.loginfo('Robot Gripper Server Ready')
        # ========= IMPORT Camera Interface =========
        self.camera = CameraInterface()
        rospy.loginfo('Camera Interface Ready')
        # ========= IMPORT DataSave Interface =========
        self.data_save = DataSaveInterface(save_dir)
        rospy.loginfo('Data Save Interface Ready')
        # ========= IMPORT TF Interface =========
        self.tf_interface = TFInterface(self.workspace_config)
        rospy.loginfo('TF Interface Ready')
        # ========= IMPORT Assemble Interface =========
        self.assemble = AssembleInterface()
        rospy.loginfo('Assemble Interface Ready')
        # ========= IMPORT Visualize Interface =========
        self.vis = VisualizeInterface()
        rospy.loginfo('Visualize Interface Ready')
        # ========= IMPORT Segment Interface =========
        self.segment_server = SegmentInterface()
        rospy.loginfo('Segment Interface Ready')

        # Generate TF msg
        # robot state for test
        rospy.sleep(1)
        # robot_state = robot_server.RecvRobotState()
        robot_state = self.camera.temp_read_state("0061")
        # robot_state = np.array(
        #     [
        #         [ 0.54718528,  0.05478036,  0.83521697,  0.20298141],
        #         [ 0.8358647 ,  0.01645447, -0.54868885, -0.65046209],
        #         [-0.04380043,  0.99836284, -0.03678533,  0.75620735],
        #         [ 0.        ,  0.        ,  0.        ,  1.        ],
        #         ]
        #     )
        self.tf_interface.set_tf_pose(self.tf_interface.tf_base2eef, robot_state, m = True, deg = True)
        rospy.sleep(0.5)

        self.camera.read_image("0061")
        pcd = self.camera.pcd()
        self.vis.pub_pcd(pcd[np.arange(1,pcd.shape[0],1)])

        self.guide_top_view = None
        self.assembly_top_view = None

        self.guide = {}
        self.assembly_pose = None
        self.pre_grasp_pose = None
        self.grasp_pose = None
        self.guide_pose = None
        self.pre_placing_pose = None

    def show_status(self):
        """
        PRINT ALL POSES
        """
        print(self.guide)
        print(self.assembly_pose)
        print(self.pre_grasp_pose)
        print(self.grasp_pose)
        print(self.guide_pose)
        print(self.pre_placing_pose)
    
    def set_tf(self):
        # robot state for test
        rospy.sleep(1)
        robot_state = self.robot_server.RecvRobotState()
        self.tf_interface.set_tf_pose(self.tf_interface.tf_base2eef, robot_state, m = True, deg = True)
        rospy.sleep(0.5)

    def get_segment(self):
        """
        RUN Segment with YOLOv8
        """
        color_img = self.camera.color_img
        self.segment_server.run(color_img)
        seg = self.segment_server.img2SegmentMask()
        # Visualization
        # server.camera.vis_segment(seg)
        return seg
    
    def get_guide_poses(self):
        """
        RUN ICP for all guide objects
        """

        # initiate assembly pose
        self.guide = {}

        # Move to guide top view and set tf
        self.robot_server.RobotMoveL(self.guide_top_view)
        self.set_tf()

        # Get poses of guide objects
        print("Get poses of guide objects")
        seg = self.get_segment()

        for obj in seg:
            obj_idx = obj[1]
            obj_seg = obj[0]
            obj_depth = obj_seg * self.camera.depth_img
            obj_pcd = self.camera.depth2pcd(obj_depth, self.tf_interface.matrix("base_link", "camera_calibration"))
            # obj_pcd = camera.depth2pcd(obj_depth, self.tf_interface.matrix("camera_color_frame", "base_link"))
            # obj_pcd = camera.depth2pcd(obj_depth)
            self.vis.pub_target_pcd(obj_pcd[np.arange(1,obj_pcd.shape[0],5)])
            pose = self.assemble.get_pose(obj_pcd, obj_idx)
            # camera.vis_pcd(obj_pcd, reduction_ratio=1) # visualize with matplotlib
            if pose is not None and obj_idx not in self.guide:
                self.guide[obj_idx] = pose
        
        if len(self.guide) > 0:
            print("GUIDE POSE ESTIMATED")
            return True
        else:
            print("GUIDE POSE NOT FOUND")
            return False
    
    def get_assembly_pose(self, idx, grasp=True):
        """
        RUN ICP for target object and generate grasp pose
        """

        # initiate assembly pose
        self.assembly_pose = None

        # Move to guide top view and set tf
        self.robot_server.RobotMoveL(self.assembly_top_view)
        self.set_tf()

        print("Get poses of guide objects")

        # SEGMENT
        seg = self.get_segment()

        for obj in seg:
            obj_idx = obj[1]
            if obj_idx == idx:
                obj_seg = obj[0]
                obj_depth = obj_seg * self.camera.depth_img
                obj_pcd = self.camera.depth2pcd(obj_depth, self.tf_interface.matrix("base_link", "camera_calibration"))
                # obj_pcd = camera.depth2pcd(obj_depth, self.tf_interface.matrix("camera_color_frame", "base_link"))
                # obj_pcd = camera.depth2pcd(obj_depth)
                self.vis.pub_target_pcd(obj_pcd[np.arange(1,obj_pcd.shape[0],5)])
                self.assembly_pose = self.assemble.get_pose(obj_pcd, obj_idx)
        
        # GET GRASP POSE
        if grasp:
            self.get_grasp_pose(seg, idx)
        
    def get_grasp_pose(self, seg, idx):
        """
        RUN KETINET
        """

    def grasp(self):
        """
        GRASP
        """
        if self.grasp_pose is None:
            print("GRASP POSE NOT FOUND")
            return
        
        # initiate
        self.pre_grasp_pose = None
        self.set_tf()

        # Calculate pre-grasp pose
        self.pre_grasp_pose = self.grasp_pose.copy()
        self.pre_grasp_pose[2,3] += 0.5
        
        # EXECUTE
        self.move_to_pose(self.pre_grasp_pose)
        self.move_to_pose(self.grasp_pose)
        #### Gripper close ####
        self.move_to_pose(self.pre_grasp_pose)
        self.set_tf()

    def matching_guide(self, idx):
        """
        Temporary matching algorithm
        0 -> 
        1 ->
        2 ->
        3 ->
        4 ->
        5 ->
        """
        matching_dict = {}

        if matching_dict[idx] in self.guide:
            return matching_dict[idx]
        else:
            return None
    
    def place(self, idx):
        if self.guide[idx] is None:
            print("GUIDE POSE NOT FOUND")
            return
        
        if self.pre_grasp_pose is None:
            print("PRE-GRASP POSE NOT FOUND")
            return
        
        # initiate
        self.pre_placing_pose = None
        self.set_tf()

        # calculate pre-placing pose
        a_t_g = self.target_pose @ np.linalg.inv(self.assembly_pose)
        self.pre_placing_pose = a_t_g @ self.pre_grasp_pose

        # EXECUTE
        self.move_to_pose(self.pre_placing_pose)
        self.move_to_pose(self.guide_pose)
        #### Gripper open ####
        self.set_tf()

    def plot_pose(self, ax, pose):
        """
        3D 공간에서 주어진 포즈를 시각화합니다.
        
        :param ax: matplotlib 3D 축
        :param pose: 4x4 변환 행렬 (포즈)
        :param label: 포즈 라벨
        :param color: 화살표 색상
        """
        # 포즈의 원점을 추출
        origin = pose[:3, 3]
        
        # 회전 행렬로부터 축 벡터 추출
        x_axis = pose[:3, 0]
        y_axis = pose[:3, 1]
        z_axis = pose[:3, 2]
        
        # 축 그리기 (각 축을 원점에서부터 축 벡터 방향으로 그립니다)
        ax.quiver(*origin, *x_axis, color='r', length=1.0)
        ax.quiver(*origin, *y_axis, color='g', length=1.0)
        ax.quiver(*origin, *z_axis, color='b', length=1.0)
    
    def plot_poses(self, poses):
        """
        3개의 포즈를 3D 공간에서 시각화합니다.
        
        :param object_pose: 객체 포즈 (4x4 행렬)
        :param grasping_pose: 그리핑 포즈 (4x4 행렬)
        :param target_pose: 타겟 포즈 (4x4 행렬)
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        # 각 포즈를 그리기
        for pose in poses:    
            self.plot_pose(ax, pose)

        # 축 범위 설정
        ax.set_xlim([-15, 15])
        ax.set_ylim([-15, 15])
        ax.set_zlim([-15, 15])
        
        # 축 라벨
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        
        plt.show()

    def move_to_home(self):
        rospy.sleep(2)
        rospy.loginfo('Move to Home pose using MoveJ')
        self.robot_server.RobotMoveJ(self.pose_config["home_pose"])
        while not self.robot_server.wait:
            rospy.sleep(1)
        self.set_tf()

    def move_to_pose(self, pose):
        rospy.sleep(5)
        rospy.loginfo('Move to target_pose pose using MoveL')
        self.robot_server.RobotMoveL(pose)
        rospy.sleep(1)
        while not self.robot_server.wait:
            rospy.sleep(1)

    def main(self):
        
        # Move to home pose
        self.move_to_home()

        # Get guide poses
        self.get_guide_poses()

        # TARGET IDX(TEMP)
        target_idx = None
        
        # Get target pose and grasp pose
        self.get_assembly_pose(target_idx)
        
        # Grasp
        self.grasp()
        
        # Get pose of target matched guide
        guide_idx = None
        self.guide_pose = self.guide[guide_idx]
        
        # Place
        self.place(guide_idx)
        

if __name__ == '__main__':
    rospy.init_node('crinmi_mr')
    server = Test()
    rospy.spin()

    # rate = rospy.Rate(10)  # 20hz
    # while not rospy.is_shutdown():
    #     server.SpinOnce()
    #     rate.sleep()

    # try:
    #     pass
    # except KeyboardInterrupt:
    #     robot_server.Disconnect()
    #     gripper_server.Disconnect()
    #     rospy.loginfo("Disconnect robot & gripper server")

    while True:
        print()
        user_input = input('q: quit / h: move to home pose / a: getting target assembly pose and grasping pose / g: getting guide poses / m: grasp /p: place')
        if user_input == 'q':
            break
        elif user_input == 'e':
            server.execute()
        elif user_input == 'h':
            server.move_to_home()
        elif user_input == 'a':
            idx_input = input('TYPE TARGET ASSEMBLY INDEX')
            server.get_assembly_pose(idx_input)
        elif user_input == 'g':
            server.get_guide_poses()
        elif user_input == 'm':
            server.grasp()
        elif user_input =='p':
            idx_input = input('TYPE TARGET GUIDE INDEX')
            server.place()
