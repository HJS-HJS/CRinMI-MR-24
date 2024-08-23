#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import os
import cv2
import yaml
import numpy as np
import rospy
import rospkg
import matplotlib.pyplot as plt
import open3d as o3d
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
        # robot_server = RobotControlServer(self.ip_config["robot"])
        rospy.loginfo('Robot Control Server Ready')
        # ========= RB10 interface test =========
        # gripper_server = GripperControlServer(self.ip_config["gripper"], 502)
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
        # ========= tf marker posision test =========
        self.assemble = AssembleInterface()
        rospy.loginfo('Assemble Interface Ready')

        # Generate TF msg
        # robot state for test
        rospy.sleep(1)
        # read_num = "0063"
        read_num = "0069"
        robot_state = camera.temp_read_state(read_num)
        # robot_state = np.array(
        #     [
        #         [ 0.54718528,  0.05478036,  0.83521697,  0.20298141],
        #         [ 0.8358647 ,  0.01645447, -0.54868885, -0.65046209],
        #         [-0.04380043,  0.99836284, -0.03678533,  0.75620735],
        #         [ 0.        ,  0.        ,  0.        ,  1.        ],
        #         ]
        #     )
        # robot_state = robot_server.RecvRobotState()
        self.tf_interface.set_tf_pose(self.tf_interface.tf_base2eef, robot_state, m = True, deg = True)
        rospy.sleep(0.5)

        vis = VisualizeInterface()
        camera.read_image(read_num)
        pcd = camera.pcd()
        vis.pub_pcd(pcd[np.arange(1,pcd.shape[0],3)])
        vis.pub_mesh()
        color_img = camera.color_img
        segment_server = SegmentInterface()
        segment_server.run(color_img)
        seg = segment_server.img2SegmentMask()
        camera.vis_segment(seg)
        for idx in seg:
            obj_seg = idx[0]
            obj_depth = obj_seg * camera.depth_img
            obj_pcd = camera.depth2pcd(obj_depth, self.tf_interface.matrix("base_link", "camera_calibration"))
            # obj_pcd = obj_pcd[np.where(obj_pcd[:,2] > 0.01)]
            # obj_pcd = obj_pcd[np.where(obj_pcd[:,2] < 0.5)]
            # obj_pcd = obj_pcd[np.where(obj_pcd[:,2] > 0.5)]
            # print(obj_pcd)
            # obj_pcd = obj_pcd[np.where(pcd[:,2] < 0.03)]
            # vis.pub_target_pcd(obj_pcd[np.arange(1,obj_pcd.shape[0],5)])
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(obj_pcd)
            pcd = pcd.voxel_down_sample(voxel_size=0.001)
            cl, ind = pcd.remove_statistical_outlier(nb_neighbors=200, std_ratio=0.02)
            pcd = pcd.select_by_index(ind)
            pcd.translate([-pcd.get_center()[0],-pcd.get_center()[1], 0])
            pcd_points = np.asarray(pcd.points)
            image = self.point_cloud_to_image(pcd_points, (720, 1280))
            plt.figure(figsize=(10, 5))
            plt.subplot(1, 1, 1)
            plt.title("Original Image 1")
            plt.imshow(image, cmap="gray")
            plt.show()

            # pose, test_pcd = self.assemble.get_pose(obj_pcd, idx[-1])
            # self.tf_interface.add_stamp("base_link", "asset_" + str(idx[-1]), pose, m = True, deg = False)
            # print("asset_" + str(idx[-1]))
            # vis.pub_test_pcd(test_pcd)
            # vis.pub_mesh()

            # camera.vis_pcd(obj_pcd, reduction_ratio=1) # visualize with matplotlib
        
        rospy.sleep(1)
    
    def point_cloud_to_image(self, point_cloud, image_size):
        # 이미지 크기 (height, width)
        height, width = image_size
        
        # 이미지의 중심 좌표
        center_x, center_y = width // 2, height // 2
        
        # 빈 이미지 생성 (검은색)
        image = np.zeros((height, width), dtype=np.uint8)
        point_cloud = 1000*point_cloud
        # print(np.min(point_cloud[:, 0].astype(int)))
        # Point Cloud의 x, y 좌표를 픽셀 좌표로 변환
        # x, y 좌표의 0, 0 위치를 이미지의 중심으로 설정
        x_coords = np.clip(center_x - (point_cloud[:, 0]).astype(int), 0, width - 1)
        y_coords = np.clip(center_y + (point_cloud[:, 1]).astype(int), 0, height - 1)

        # 해당 좌표를 하얀색으로 설정
        image[y_coords, x_coords] = 255
        print("Return")
        return image
    
if __name__ == '__main__':
    rospy.init_node('crinmi_mr')
    server = Test()
    rospy.spin()
    # rate = rospy.Rate(10)  # 20hz
    # while not rospy.is_shutdown():
    #     server.SpinOnce()
    #     rate.sleep()

    try:
        pass
    except KeyboardInterrupt:
        robot_server.Disconnect()
        gripper_server.Disconnect()
        rospy.loginfo("Disconnect robot & gripper server")

