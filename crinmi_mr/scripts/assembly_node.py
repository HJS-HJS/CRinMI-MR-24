#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import os
import cv2
import yaml
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
from segment_interface.yolo_segment_interface import *
from assemble_interface.assemble_interface import AssembleInterface
from calibrate_interface.calibration import CalibrationInterface

class Test(object):
    
    def __init__(self):

        # get parameter
        config_file = os.path.abspath(os.path.join(rospkg.RosPack().get_path('crinmi_mr'),'config'))
        save_dir    = os.path.abspath(os.path.join(rospkg.RosPack().get_path('crinmi_mr'),'image'))
        self.workspace_config     = rospy.get_param("~robot")
        self.ip_config            = rospy.get_param("~robot_ip")[str(self.workspace_config)]
        self.pose_config          = rospy.get_param("~robot_pose")
        self.match_tf_config      = rospy.get_param("~match_pose")
        self.grip_tf_config       = rospy.get_param("~match_grip")

        self.gripper_offset = 0.02
        self.simulation = False
        self.scene = {"guide" : "0061", "assemble": "0064"}

        # ========= RB10 interface test =========
        if not self.simulation: 
            self.robot_server = RobotControlServer(self.ip_config["robot"])
        rospy.loginfo('Robot Control Server Ready')
        # ========= RB10 interface test =========
        if not self.simulation:
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
        # ========= IMPORT Calibration Interface =========
        self.calibration = CalibrationInterface()
        rospy.loginfo('Calibration Interface Ready')


        # self.gripper_server.GripperMove(100)
        # Generate TF msg
        # robot state for test
        rospy.sleep(1)
        if not self.simulation: 
            self.robot_state = self.robot_server.RecvRobotState()
            self.tf_interface.set_tf_pose(self.tf_interface.tf_base2eef, self.robot_state, m = True, deg = True)
            rospy.sleep(0.5)

            pcd = self.camera.pcd()
            self.vis.pub_pcd(pcd[np.arange(1,pcd.shape[0],10)])

            self.guide_top_view = np.array(self.pose_config[str(self.workspace_config)]['guide_capture_pose'])
            self.assembly_top_view = np.array(self.pose_config[str(self.workspace_config)]['assemble_capture_pose'])

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
    
    def set_tf(self, scene:str = None):
        # robot state for test
        rospy.sleep(1)
        if scene is None:
            robot_state = self.robot_server.RecvRobotState()
        else:
            robot_state = self.camera.temp_read_state(scene)
            self.camera.read_image(scene)
            
        self.tf_interface.set_tf_pose(self.tf_interface.tf_base2eef, robot_state, m = True, deg = True)
        rospy.sleep(1)
        pcd = self.camera.pcd()
        self.vis.pub_pcd(pcd[np.arange(1,pcd.shape[0],10)])
        self.vis.pub_mesh()

    def get_segment(self, data:str = None):
        """
        RUN Segment with YOLOv8
        """
        if data is not None:
            self.camera.read_image(data)
        color_img = self.camera.color_img
        self.segment_server.run(color_img)
        seg = self.segment_server.img2SegmentMask()
        # Visualization
        return seg
    
    def get_guide_poses(self):
        """
        RUN ICP for all guide objects
        """

        # initiate assembly pose
        self.guide = {}

        # Move to guide top view and set tf
        if not self.simulation:
            self.robot_server.SetVelocity(30)
            self.move_to_pose(self.guide_top_view)
            self.set_tf()
        else:
            self.set_tf(self.scene["guide"])


        # Get poses of guide objects
        print("Get poses of guide objects")
        if not self.simulation:
            seg = self.get_segment()
        else:
            seg = self.get_segment(self.scene['guide'])

        for obj in seg:
            obj_seg = cv2.erode(obj[0], None, iterations=2) # asset
            obj_depth = obj_seg * self.camera.depth_img
            obj_pcd = self.camera.depth2pcd(obj_depth, self.tf_interface.matrix("base_link", "camera_calibration"))
            self.vis.pub_target_pcd(obj_pcd[np.arange(1,obj_pcd.shape[0],5)])
            pose, test_pcd = self.assemble.get_pose(obj_pcd, obj[-1])
            self.tf_interface.add_stamp("base_link", "asset_" + str(obj[-1]), pose, m = True, deg = False)
            print("asset_" + str(obj[-1]))
            self.vis.pub_test_pcd(test_pcd)
            self.vis.pub_mesh()


            if pose is not None and obj[-1] not in self.guide:
                self.guide[obj[-1]] = pose
        if len(self.guide) == 11:
            print("GUIDE POSE ESTIMATED")
            return True
        else:
            print("GUIDE POSE NOT FOUND")
            return False
    
    def get_target(self):
        """
        RUN ICP for target object and generate grasp pose
        """

        # initiate assembly pose
        self.assembly_pose = None

        # Move to guide top view and set tf
        if not self.simulation:
            self.move_to_pose(self.assembly_top_view)
            rospy.sleep(1.0)
            self.set_tf()
        else:
            self.set_tf(self.scene["assemble"])

        print("Get poses of guide objects")

        # SEGMENT
        seg = self.get_segment()

        # TODO
        # TODO
        # TODO

        self.camera.vis_segment(seg)
        while True:
            user_input = input('Enter idx\n****** Must Be INT *******\n')
            if user_input == 'q':
                return
            else:
                try:
                    idx = int(user_input)
                    break
                except:
                    pass
                    

        obj = seg[idx]
        obj_seg = cv2.erode(obj[0], None, iterations=2) # asset
        obj_depth = obj_seg * self.camera.depth_img
        seg_instance = obj
        obj_pcd = self.camera.depth2pcd(obj_depth, self.tf_interface.matrix("base_link", "camera_calibration"))
        self.vis.pub_target_pcd(obj_pcd[np.arange(1,obj_pcd.shape[0],5)])
        pose, pcd = self.assemble.get_pose(obj_pcd, obj[-1])
        self.tf_interface.del_stamp('asset_' + str(obj[-1]))
        self.tf_interface.add_stamp("base_link", "asset_" + str(obj[-1]), pose, m = True, deg = False)
        print("asset_" + str(obj[-1]))
        self.vis.pub_test_pcd(pcd)
        self.vis.pub_mesh()

        return obj
    
    def get_grasp_pose(self, obj):
        print("get_grasp_pose")
        target_frame = 'asset_' + str(obj[-1])
        rospy.sleep(1)
        base2assemble = self.tf_interface.matrix('base_link', target_frame)
        offset = 0.001
        center = base2assemble[:3,3]
        center[2] -= offset

        # Point cloud processing
        obj_seg = obj[0]
        kernel_size_thin = 15  # 작은 커널 크기
        kernel_thin = np.ones((kernel_size_thin, kernel_size_thin), np.uint8)
        thin_dilated_mask = cv2.dilate(obj_seg, kernel_thin, iterations=1)

        # 두 번째 dilation (두껍게 확장)
        kernel_size_thick = 55  # 큰 커널 크기
        kernel_thick = np.ones((kernel_size_thick, kernel_size_thick), np.uint8)
        thick_dilated_mask = cv2.dilate(obj_seg, kernel_thick, iterations=1)

        # 차이 계산 (두껍게 확장된 부분에서 얇게 확장된 부분을 제외)
        difference_mask = cv2.subtract(thick_dilated_mask, thin_dilated_mask)
        obs_depth = difference_mask * self.camera.depth_img

        obs_pcd = self.camera.depth2pcd(obs_depth, self.tf_interface.matrix("base_link", "camera_calibration"))        
        obs_pcd = obs_pcd[obs_pcd[:, 2] <= 0.5]
    
        grip_points = np.array(self.grip_tf_config[str(obj[-1])]).reshape(-1, 3)
        print('grip_points')
        print(grip_points)
        grip_points = np.hstack([grip_points,np.ones((grip_points.shape[0], 1))])
        grip_points = (base2assemble @ grip_points.T).T

        is_parallel = grip_points[::2] - grip_points[1::2]
        is_parallel = 0.001 - np.abs(is_parallel)
        is_parallel = np.argwhere(is_parallel[:,2] > 0)
        print('is_parallel')
        print(is_parallel)
        parallel_points = np.squeeze(grip_points.reshape(-1, 2, 4)[is_parallel])
        self.vis.pub_test_pcd(obs_pcd)
        sorf_list = np.argsort(parallel_points[:,0,2])[::-1]
        for point_l, point_r in parallel_points[sorf_list]:
            if self.check_collision(point_l, obs_pcd) & self.check_collision(point_r, obs_pcd):
                print(point_l, point_r)
                diff = point_l - point_r
                angle = np.rad2deg((np.arctan2(diff[1], diff[0])))
                center = (point_l + point_r)/2
                print("grip angle:", angle)
                print("grip point:", center)
                print("success")
                grasp_point = self.grip_point2matrix(center, angle)
                self.tf_interface.add_stamp("base_link", "grasp_point", grasp_point, m = True, deg = False)
                return grasp_point, np.linalg.norm(point_l - point_r)

            else:
                print("Failed")
                continue

        return None

    def check_collision(self, point, pcd):
        
        diff = pcd - point[:3] # (n, 3)

        squared_distances = np.linalg.norm(diff[np.where(diff[:,2] > 0)], axis=1) # (n, )
        # finger tip size
        threshold = 0.01 + self.gripper_offset
        if len(squared_distances)==0:
               return True
        if np.any(squared_distances > threshold):
            return True
        else:
            return False

    def grip_point2matrix(self, point, pose):
        # return pose2matrix([point[0], point[1], point[2] + 0.246, 90, 0, 42.8 + pose])
        return pose2matrix([point[0], point[1], point[2] + 0.246, 90, 0, 44 + pose])

    def select_guide(self):
        while True:
            user_input = input("\n 6  \'g_L_shape_angle\'\n 7  \'g_bearing_holder_x\'\n 8  \'g_bearing_holder_y\'\n 9  \'g_cam_zig_x\'\n 10 \'g_cam_zig_y\'\n 11 \'g_hex_wrench_x\'\n 12 \'g_hex_wrench_y\'\n 13 \'g_rod_end_bearing_x\'\n 14 \'g_rod_end_bearing_y\'\n 15 \'g_roller_x\'\n 16 \'g_roller_y\'\n")
            if user_input == 'q':
                return
            else:
                try:
                    idx = int(user_input)
                    break
                except:
                    pass

        return idx

    def grasp(self):
        """
        GRASP
        """
        if self.grasp_pose is None:
            print("GRASP POSE NOT FOUND")
            return
        
        # initiate
        self.pre_grasp_pose = None

        # Calculate pre-grasp pose
        self.pre_grasp_pose = self.grasp_pose.copy()
        self.pre_grasp_pose[2,3] += 0.5
        

        if self.simulation:
            # visualiz
            pass
        else:
            # EXECUTE
            print("input grip width",self.grip_width)
            self.gripper_server.GripperMoveGrip()
            # self.gripper_server.GripperMove(self.grip_width)
            self.move_to_pose(self.pre_grasp_pose)
            test_pose = self.grasp_pose.copy()
            test_pose[2,3] += (0.246 + 0.01 + 0.05)
            self.move_to_pose(test_pose)
            # self.move_to_pose(self.grasp_pose)
            #### Gripper close ####
            rospy.sleep(10)
            # self.gripper_server.GripperMoveGrip()
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
    
    def place(self, assemble_idx, guide_idx, grasp_matrix):
        base2guide = self.tf_interface.matrix('base_link', 'asset_' + str(guide_idx))
        base2assemble = self.tf_interface.matrix('base_link', 'asset_' + str(assemble_idx))
        guide2place_set = np.array(self.match_tf_config[str(guide_idx)])

        # for guide2place in guide2place_set:
            
        #     print(grasp_matrix)
        #     print(base2assemble)
            
        #     assemble2grasp = np.linalg.inv(base2assemble) @ grasp_matrix
        #     base2assemble2 = base2guide@guide2place
        #     base2place = base2assemble2 @ assemble2grasp
        #     if base2place[2,3] > 0:
        #         self.tf_interface.add_stamp('base_link', 'place', base2place, m = True, deg = False)
        #         self.vis.pub_mesh()
        #         self.tf_interface.broadcast_once()
        #         rospy.sleep(1)
        #         break
        #         # return

        assemble2grasp = np.linalg.inv(base2assemble) @ grasp_matrix
        base2assemble2 = np.matmul(base2guide, guide2place_set)
        base2place = np.dot(base2assemble2, assemble2grasp)
        print("place candidate: ", base2place.shape)
        print("place candidate: ", base2place)

        arg = np.argmax(base2place[:,2,3])

        self.tf_interface.add_stamp('base_link', 'place', base2place[arg], m = True, deg = False)
        self.vis.pub_mesh()
        self.tf_interface.broadcast_once()
        rospy.sleep(1)

        # pick = grasp_matrix
        return base2place[arg]

        pick[2,3] -= 0.005
        place[2,3] += 0.03

        print("move to pick")
        self.robot_server.SetVelocity(10)
        self.move_to_pose(pick)
        rospy.sleep(5)
        self.gripper_server.GripperMoveGrip()
        print("move to home")
        self.robot_server.SetVelocity(40)
        self.move_to_home()
        rospy.sleep(5)
        print("move to place")
        self.robot_server.SetVelocity(10)
        self.move_to_pose(place)


        # if 
        #     print("GUIDE POSE NOT FOUND")
        #     return
        
        # if self.pre_grasp_pose is None:
        #     print("PRE-GRASP POSE NOT FOUND")
        #     return
        
        # # initiate
        # self.guide_pose = self.guide[assemble_idx]
        # self.pre_placing_pose = None
        # self.set_tf()

        # # calculate pre-placing pose
        # a_t_g = self.guide_pose @ np.linalg.inv(self.assembly_pose)
        # self.pre_placing_pose = a_t_g @ self.pre_grasp_pose

        # # EXECUTE
        # self.move_to_pose(self.pre_placing_pose)
        # self.move_to_pose(self.guide_pose)
        # #### Gripper open ####
        # self.gripper_server.GripperMoveRelease()
        # self.set_tf()
        

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
        self.robot_server.RobotMoveJ(self.pose_config[str(self.workspace_config)]["home_pose"])
        rospy.sleep(1)
        while not self.robot_server.wait:
            rospy.sleep(1)
        
        self.set_tf()

    def move_to_pose(self, pose, is_assemble:bool = True):
        rospy.sleep(1)
        rospy.loginfo('Move to target_pose pose using MoveL')
        offset, _temp = self.calibration.calculate_offset(pose[:3,3], assemble = is_assemble)
        pose[:2,3] += offset[:2]
        self.robot_server.RobotMoveL(pose)
        rospy.sleep(2)
        while not self.robot_server.wait:
            print("wait")
            rospy.sleep(1)

    def main(self):
        # Get guide poses
        self.get_guide_poses()

        # Get target pose
        obj = self.get_target()
        
        # Get grasp pose
        point, angle = self.get_grasp_pose(obj)

        # Grasp
        self.grasp()
        
        # Get pose of target matched guide
        guide_idx = None
        self.guide_pose = self.guide[guide_idx]
        
        # Place
        self.place(guide_idx)
        
    def test(self):
        
        # Move to home pose
        self.move_to_home()

        # Get guide poses
        self.get_guide_poses()
        print("\trecord: ", 
            self.data_save.save_data(
            "guide",
            self.camera.color_img_msg, 
            self.camera.depth_img_msg,
            self.camera.color_cam_info_msg,
            self.camera.depth_cam_info_msg,
            self.robot_state
            )
        )
        
        while True:
            user_input = input('enter enter\n')
            if user_input == 'q':
                return
            elif user_input == '':
                # Get target pose
                obj = self.get_target()
                
                # Get grasp pose
                grasp_matrix, grasp_width = self.get_grasp_pose(obj)

                print("\trecord: ", 
                    self.data_save.save_data(
                    "assemble",
                    self.camera.color_img_msg, 
                    self.camera.depth_img_msg,
                    self.camera.color_cam_info_msg,
                    self.camera.depth_cam_info_msg,
                    self.robot_state
                    )
                )
                guide_idx = self.select_guide()

                self.gripper_server.GripperMove(grasp_width + self.gripper_offset)

                pick = (grasp_matrix).copy()
                pick_prev = (grasp_matrix).copy()
                pick[2,3] -= 0.008
                pick_prev[2,3] += 0.20
                print("move to pick")
                self.robot_server.SetVelocity(45)
                self.move_to_pose(pick_prev)
                rospy.sleep(1)
                self.robot_server.SetVelocity(10)
                self.move_to_pose(pick)
                rospy.sleep(1)
                self.gripper_server.GripperMoveGrip()
                rospy.sleep(1)
                self.robot_server.SetVelocity(45)
                self.move_to_pose(pick_prev)
                rospy.sleep(1)

                # # Get pose of target matched guide
                place_matrix = self.place(obj[-1], guide_idx, grasp_matrix)
                place = (place_matrix).copy()
                place_prev = (place_matrix).copy()
                place[2,3] += 0.01
                place_prev[2,3] += 0.2
                print("move to place")
                self.robot_server.SetVelocity(45)
                self.move_to_pose(place_prev, is_assemble=False)
                rospy.sleep(1)
                self.robot_server.SetVelocity(10)
                self.move_to_pose(place, is_assemble=False)
                rospy.sleep(1)
                # self.gripper_server.GripperMove(grasp_width + self.gripper_offset)
                self.gripper_server.GripperMoveRelease()
                rospy.sleep(1)
                self.robot_server.SetVelocity(45)
                self.move_to_pose(place_prev, is_assemble=False)
                rospy.sleep(1)
                self.move_to_home()
            else:
                continue

    def aaaa(self):
        while True:
            user_input = input('enter enter\n')
            if user_input == 'q':
                return
            elif user_input == '':
                # Get target pose
                obj = self.get_target()
                
                # Get grasp pose
                grasp_matrix, grasp_width = self.get_grasp_pose(obj)
                
                self.gripper_server.GripperMove(grasp_width + self.gripper_offset)
                pick = (grasp_matrix).copy()
                pick_prev = (grasp_matrix).copy()
                # place = base2place[arg]

                pick[2,3] -= 0.005
                pick_prev[2,3] += 0.08

                print("move to pick")
                self.robot_server.SetVelocity(40)
                self.move_to_pose(pick_prev)
                rospy.sleep(1)
                self.robot_server.SetVelocity(10)
                self.move_to_pose(pick)
                rospy.sleep(1)
                self.gripper_server.GripperMoveGrip()
                print("move to release")
                pick[2,3] += 0.03
                self.move_to_pose(pick)
                rospy.sleep(1)
                print("move to place")
                self.gripper_server.GripperMoveRelease()
            else:
                continue
    

if __name__ == '__main__':
    rospy.init_node('crinmi_mr')
    server = Test()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        user_input = input('q: quit / h: move to home pose / a: getting target assembly pose and grasping pose / g: getting guide poses / m: grasp /p: place\n')
        if user_input == 'q':
            break
        elif user_input == 'm':
            server.main()
        elif user_input == 'test':
            server.test()
        elif user_input == 'h':
            server.move_to_home()
        elif user_input == 'g':
            server.get_guide_poses()
        elif user_input == 'a':
            # idx_input = 5
            # server.get_target(idx_input)
            server.aaaa()
        elif user_input == 'pick':
            server.grasp()
        elif user_input =='place':
            idx_input = 10
            server.place(idx_input)
        elif user_input =='w':
            server.gripper_server.GripperTest()

            # server.gripper_server.GripperMove(0.03)
            # rospy.sleep(5)
            # server.gripper_server.GripperMove(0.04)
            # rospy.sleep(5)
            # server.gripper_server.GripperMove(0.01)
            # rospy.sleep(5)
            # server.gripper_server.GripperMove(0.03)
            # rospy.sleep(5)
            # server.gripper_server.GripperMove(0.02)
            # rospy.sleep(5)
            # server.gripper_server.GripperMove(0.05)
        rate.sleep()