#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import json
import numpy as np

import rospy
import rospkg
import tf2_ros
import tf.transformations as tf
from geometry_msgs.msg import TransformStamped

class Crinmi_TransformStamped(TransformStamped):
    def __init__(self, parent_id, child_id, transform, rotation, mm = True, deg = True):
        super(Crinmi_TransformStamped, self).__init__()
        self.header.frame_id = parent_id
        self.child_frame_id = child_id
        self.header.stamp = rospy.Time.now()
        self.transform.translation.x = transform[0]
        self.transform.translation.y = transform[1]
        self.transform.translation.z = transform[2]
        self.t(transform, mm)
        self.r(rotation, deg)
    
    def t(self, transform, mm = True):
        if not mm:
            transform = np.array(transform) / 1000
        self.transform.translation.x = transform[0]
        self.transform.translation.y = transform[1]
        self.transform.translation.z = transform[2]

    def r(self, rotation, deg = True):
        if deg:
            rotation = np.deg2rad(np.array(rotation))
        if len(rotation) == 3:
            rotation = tf.quaternion_from_euler(rotation[0], rotation[1], rotation[2], axes = 'rxyz')
        self.transform.rotation.x = rotation[0]
        self.transform.rotation.y = rotation[1]
        self.transform.rotation.z = rotation[2]
        self.transform.rotation.w = rotation[3]

    @property
    def matrix(self):
        return

class TFInterface(object):
    def __init__(self, robot_id:int = 1):
        rospy.loginfo("calibration interface imported")
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        json_file = os.path.abspath(os.path.join(rospkg.RosPack().get_path('crinmi_mr'),'config', 'robot_camera_calibration', 'calibration{}.json'.format(robot_id)))
        # Load configuation file
        with open(json_file,'r') as f:
            cfg = json.load(f)

        # camera matrix from eef
        self.m_gripper2cam = np.eye(4)
        self.m_gripper2cam[:3,3] = np.array(cfg["calibration_data"] ) * -1 / 1000

        self.m_eef2gripper = np.array([
          [-1.0, 0.0, 0.0, 0.0],
          [0.0, 0.0, -1.0, 0.0],
          [0.0, -1.0, 0.0, 0.0],
          [0.0, 0.0, 0.0, 1.0],  
        ])

        # tf
        self.tf_base2eef  = Crinmi_TransformStamped(
            "base_link", "eef", 
            [0, 0, 0], 
            [0, 0, 0],
            mm = True, deg = True
            )
        self.tf_cam2point = Crinmi_TransformStamped(
            "camera_base", "point",
            [0, 0, 0], [0, 0, 0],
            mm = True, deg = True
            )
        # static tf
        self.tf_eef2gripper = Crinmi_TransformStamped(
            "eef", "gripper", 
            [0, 0, 0], 
            [90, 0, 37],
            mm = True, deg = True
            )
        self.tf_gripper2cam = Crinmi_TransformStamped(
            "gripper", "camera_base", 
            np.array(cfg["calibration_data"] ) * -1, [0, 0, 0], 
            mm = False, deg = True
            )
        
        self.static_broadcaster.sendTransform([self.tf_eef2gripper, self.tf_gripper2cam])

    def broadcast(self):
        self.broadcaster.sendTransform([self.tf_base2eef, self.tf_cam2point])
    
    def base2eef(self, pose):
        self.tf_base2eef.t(pose[0:3])
        self.tf_base2eef.r(pose[3:6])

    def cam2point(self, pose):
        self.tf_cam2point.t(pose[0:3])
        self.tf_cam2point.r(pose[3:6])

if __name__ == '__main__':
    rospy.init_node("tf2_broadcaster")
    module = TFInterface(1)
