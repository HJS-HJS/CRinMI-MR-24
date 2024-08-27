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


        # Generate TF msg
        # robot state for test
        # rospy.sleep(1)
        # robot_state = robot_server.RecvRobotState()
        # # robot_state =np.array([[-6.69130606e-01, -3.12223171e-15,  7.43144825e-01, -4.64933217e-01],
        # #                         [ 7.43144825e-01,  3.37607931e-15,  6.69130606e-01, -7.68679976e-01],
        # #                         [-4.59809667e-15,  1.00000000e+00,  6.12323400e-17,  6.60972357e-01],
        # #                         [ 0.00000000e+00, 0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

        # # robot_state = np.array(
        # #     [
        # #         [ 0.54718528,  0.05478036,  0.83521697,  0.20298141],
        # #         [ 0.8358647 ,  0.01645447, -0.54868885, -0.65046209],
        # #         [-0.04380043,  0.99836284, -0.03678533,  0.75620735],
        # #         [ 0.        ,  0.        ,  0.        ,  1.        ],
        # #         ]
        # #     )
        # self.tf_interface.set_tf_pose(self.tf_interface.tf_base2eef, robot_state, m = True, deg = True)

        # marker_5_pose = np.array([  0.11435308,  0.08167744,  0.46246641, 0, 0, 0])
        # marker_30_pose = np.array([ -0.17843222,  0.08005709,  0.48505111, 0, 0, 0])
        # marker_21_pose = np.array([-0.0316321,  -0.03495728,  0.48229999, 0, 0, 0])
        # marker_37_pose = np.array([ 0.11205553, -0.15022158,  0.46633627, 0, 0, 0])
        # marker_4_pose = np.array([-0.17848817, -0.15740559,  0.50243064, 0, 0, 0])


        # self.tf_interface.add_stamp("camera_color_optical_frame", "marker_37", marker_37_pose, m = True, deg = False)
        # # target_marker = base2cam @ cam2target

        # rospy.sleep(2)
        # target_marker =  self.tf_interface.matrix(target="base_link", source="marker_37")
        # print(target_marker)
        # gripper_server.GripperMoveGrip()
        # robot_server.SetVelocity(10)
        # robot_server.RobotMoveL(self.guide_top_view)
        # vis = VisualizeInterface()
        # pcd = camera.pcd(np.eye(4))
        # vis.pub_pcd(pcd[np.arange(1,pcd.shape[0],1)])
        # camera.vis_image()


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

        cam_matrix = np.array([
            [-0.07356074,  0.98395957, -0.16251887, -0.20279966],
            [ 0.99547739,  0.06262279, -0.07143634, -0.68058853],
            [-0.06011309, -0.16703877, -0.98411608,  0.63329831],
            [ 0.        ,  0.        ,  0.        ,  1.        ],
            ])
        



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

