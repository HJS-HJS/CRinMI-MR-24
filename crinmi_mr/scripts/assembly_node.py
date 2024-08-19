#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import os
import cv2
import yaml
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
        robot_state = camera.temp_read_state("0061")
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

        self.assembly_pose = np.array(
            [[ 1., 0. , 0. , 6.13629426],
            [ 0., 1., 0., -7.38735657],
            [ 0.,  0. ,  1., -1.23485192],
            [ 0.,          0. ,         0.        ,  1.        ]]
        )

        self.guide = {}
        theta = np.pi / 4
        self.target_pose =  np.array(
            [[ np.cos(theta), -np.sin(theta) , 0. ,6.13629426],
              [ np.sin(theta),  np.cos(theta) ,0. , -3.],
            [ 0.,  0. ,1. ,  3.059168  ],
            [ 0.       ,   0.       ,   0.       ,   1.        ]]
        )

        ### 나중에 삭제 ###
        self.grasp_pose = np.array(
            [[ 1., 0. ,0. ,6.13629426],
            [0.,  -1. , 0.,  -7.38735657],
            [0.,0., -1.,  1.],
            [ 0.       ,   0.   ,       0.       ,   1.        ]]
        )
    
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
    
    def get_guide_poses(self, seg):
        """
        RUN ICP for all guide objects
        """
        # initiate assembly pose
        self.guide = {}

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
            return True
        else:
            return False

    def get_assembly_pose(self, seg, idx):
        """
        RUN ICP for target object
        """

        # initiate assembly pose
        self.assembly_pose = None

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
        
        # 실패할 경우
        if self.assembly_pose is not None:
            return True
        else:
            return False

    def get_grasp(self):
        """
        RUN KETINET
        """
        if ketinet:
            self.grasp_pose = None # ketinet
        else:
            self.grasp_pose = None # 6d pose

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
    
    def calculate_trajectory(self):
        """
        Calculate trajectory. All poses are defined in base coordinate
        """

        trajectory = []

        "Home Pose"
        trajectory.append(self.assembly_pose)
        
        "Pre-Grasp pose"
        pre_grasp_pose = self.grasp_pose.copy()
        pre_grasp_pose[2,3] += 0.5
        trajectory.append(pre_grasp_pose)
        
        "Grasp pose"
        trajectory.append(self.grasp_pose)

        "Pre-Grasp pose"
        trajectory.append(pre_grasp_pose)

        "Pre-Placing pose"
        a_t_g = self.target_pose @ np.linalg.inv(self.assembly_pose)
        pre_placing_pose = a_t_g @ pre_grasp_pose
        trajectory.append(pre_placing_pose)

        "Placing pose"
        trajectory.append(self.target_pose)

        return trajectory

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
    
    def plot_poses(self, trajectory):
        """
        3개의 포즈를 3D 공간에서 시각화합니다.
        
        :param object_pose: 객체 포즈 (4x4 행렬)
        :param grasping_pose: 그리핑 포즈 (4x4 행렬)
        :param target_pose: 타겟 포즈 (4x4 행렬)
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        # 각 포즈를 그리기
        for traj in trajectory:
            print(traj)    
            self.plot_pose(ax, traj)

        # 축 범위 설정
        ax.set_xlim([-15, 0])
        ax.set_ylim([-15, 0])
        ax.set_zlim([-15, 5])
        
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

    def main(self):
        
        # Move to home pose
        self.move_to_home()

        # Move to guide top view

        # Get poses of guide objects
        print("Get poses of guide objects")
        guide_seg = self.get_segment()
        if self.get_guide_poses(guide_seg):
            print("DONE")
        else:
            print("0 GUIDE FOUND")
            return

        # Move to assembly top view

        # # Input target assembly idx and guide idx(TEMP ver)
        # user_input = input('Press target idx, q to quit...')
        # while True:
        #     user_input = input('Press target idx, q to quit...')
        #     if user_input.lower() == 'q':
        #         print("Exiting...")
        #         return
        #     else:
        #         target_idx = int(user_input)
        #         break

        # # Matching target assembly with guide
        # guide_idx = self.matching_guide(target_idx)
        # if guide_idx is None:
        #     print("MATCHING FAILED")
        #     return
        target_idx = None
        guide_idx = None

        # Get pose of target assembly
        print("Get pose of target assembly")
        guide_seg = self.get_segment()
        if self.get_assembly_pose(guide_seg, target_idx):
            print("DONE")
        else:
            print("POSE NOT FOUND")
            return
        
        # Get grasp point of target assembly
        self.grasp_pose =  self.get_grasp()
        if self.grasp_pose is None:
            print("GRASP POSE NOT FOUND")
            return
        
        # Create trajectory to grasping and placing
        trajectory = self.calculate_trajectory()
        if len(trajectory) == 6:
            return trajectory
        else:
            print("Trajectory generation failed")
            return
        

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
        user_input = input('q: quit / h: move to home pose / e: execute main loop')
        if user_input == 'q':
            break
        elif user_input == 'e':
            server.main()
        elif user_input == 'h':
            server.move_to_home()
