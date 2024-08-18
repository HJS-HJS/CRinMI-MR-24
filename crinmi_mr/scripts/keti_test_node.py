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
from segment_interface.yolo_segment_interface import *
from ketinet_interface.ketinet_interface import *

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
        # ========= ketinet test =========
        self.ketinet = KetinetInterface()
        rospy.loginfo('Ketinet Interface Ready')

        # Generate TF msg
        # robot state for test
        rospy.sleep(1)
        # robot_state = robot_server.RecvRobotState()
        robot_state = np.array(self.pose_config["assemble_capture_pose"][1])
        self.tf_interface.set_tf_pose(self.tf_interface.tf_base2eef, robot_state, m = True, deg = True)
        rospy.sleep(0.5)

        vis = VisualizeInterface()
        camera.read_image("0070")
        # camera.vis_image()
        pcd = camera.pcd()
        vis.pub_pcd(pcd[np.arange(1,pcd.shape[0],1)])

        while True:
            user_input = input('Press enter to record, q to quit...')
            if user_input == 'q':
                break
            elif user_input == '':
                color_img = camera.color_img
                segment_server = SegmentInterface()
                segment_server.run(color_img)
                seg = segment_server.img2SegmentMask()
                camera.vis_segment(seg)

                for idx, seg_instance in enumerate(seg):
                    obj_seg = seg_instance[0]
                    obj_depth = obj_seg * camera.keti_depth_img
                    # cv2.imshow("test", obj_seg)
                    # cv2.waitKey(0)

                    self.ketinet.set_workspace(seg_instance[1], scale_factor=1.2)
                    grip_candidate, best_grip_candidate, keti_img = self.ketinet.run_kitinet(camera.color_img, camera.keti_depth_img)
                    # grip_candidate, best_grip_candidate, img_keti = self.ketinet.run_kitinet(camera.color_img, obj_depth)
                
                    for i in best_grip_candidate:
                        xc = int(grip_candidate[i][0])
                        yc = int(grip_candidate[i][1])
                        zc = int(grip_candidate[i][2])
                        
                        if obj_seg[yc][xc] == 1:
                            print("Found grip point")
                            grip_width = grip_candidate[i][3]
                            angle, grip_xl, grip_yl, grip_xr, grip_yr = grip_candidate[i][5:10]
                            grip_xl = int(grip_xl)
                            grip_yl = int(grip_yl)
                            grip_xr = int(grip_xr)
                            grip_yr = int(grip_yr)
                            
                            # print(angle, grip_xl, grip_yl, grip_xr, grip_yr)
                    
                            # # Specify the coordinates for the point (x, y)
                            # # Draw the point on the image
                            # cv2.circle(keti_img, (xc, yc), 10, (0, 255, 255), -1)

                            # # Display the modified image in a window
                            # cv2.imshow('Image with Red Point', keti_img)

                            # # Wait for a key press indefinitely or for a specified amount of time in milliseconds
                            # cv2.waitKey(0)
                            break
                    
                    x, y, z = camera.pixel_to_3d(xc, yc)

                    # print("xyz")
                    # print(x, y, z)
                    # print("base to camera_calibration matrix")
                    # print(self.tf_interface.matrix('base_link', 'camera_calibration'))
                    cam2assemble = np.eye(4)
                    cam2assemble[:3, :3] = np.transpose(self.tf_interface.matrix('base_link', 'camera_calibration')[:3, :3])
                    cam2assemble[:, 3] = x, y, zc/1000, 1
                    # print("camera link to assemble ")
                    # print(cam2assemble)
                
                    self.tf_interface.set_tf_pose(self.tf_interface.tf_cam2assemble, cam2assemble, m = True, deg = True)
                    rospy.sleep(0.5)

                    fig = plt.figure()
                    image = fig.add_subplot(1,1,1)
                    image.imshow(cv2.circle(keti_img, (xc, yc), 10, (0, 255, 255), -1))
                    plt.show()

                    base2cam = self.tf_interface.matrix("base_link", "camera_calibration")
                    base2assemble = self.tf_interface.matrix("base_link", "assemble_part")
                    # print(base2assemble)

                    # convert to camera coordinate xy
                    grip_xl, grip_yl, dummy = camera.pixel_to_3d(grip_xl, grip_yl)
                    grip_xr, grip_yr, dummy = camera.pixel_to_3d(grip_xr, grip_yr)

                    # convert to base coordinate xy
                    grip_l = base2cam * np.array([grip_xl, grip_yl, zc/1000, 1])
                    grip_r = base2cam * np.array([grip_xr, grip_yr, zc/1000, 1])
                    
                    dx = grip_l[0, 0] - grip_r[0, 0]
                    dy = grip_l[0, 1] - grip_r[0, 1]

                    grip_angle = math.atan2(dy, dx)
                    grip_angle = math.degrees(grip_angle)
                    pose = pose2matrix([base2assemble[3, 0], base2assemble[3, 1], base2assemble[3, 2] + 0.5, 0, 0, grip_angle], mm=False)
                
                    # ================== ROBOT MOVE CODE ========================
                    
                    # gripper_server.GripperMove(grip_width)
                    # robot_server.RobotMoveL(pose)
                    
                    # ================== ROBOT MOVE CODE ========================
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
        # robot_server.Disconnect()
        # gripper_server.Disconnect()
        rospy.loginfo("Disconnect robot & gripper server")

