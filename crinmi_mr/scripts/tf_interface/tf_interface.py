#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import json
import numpy as np

import rospy
import rospkg
import tf.transformations as tf
import tf2_ros
from geometry_msgs.msg import TransformStamped

class Crinmi_TransformStamped(TransformStamped):
    def __init__(self, parent_id, child_id, transform = [0, 0, 0], rotation = [0, 0, 0], m = True, deg = True):
        super(Crinmi_TransformStamped, self).__init__()
        self.header.frame_id = parent_id
        self.child_frame_id = child_id
        self.header.stamp = rospy.Time.now()
        self.transform.translation.x = transform[0]
        self.transform.translation.y = transform[1]
        self.transform.translation.z = transform[2]
        self.t(transform, m)
        self.r(rotation, deg)
    
    def t(self, transform, m = True):
        if not m:
            transform = np.array(transform) / 1000
        if len(transform) != 3:
            transform = np.array(transform)[:3,3]
        self.transform.translation.x = transform[0]
        self.transform.translation.y = transform[1]
        self.transform.translation.z = transform[2]

    def r(self, rotation, deg = True):
        if deg:
            rotation = np.deg2rad(np.array(rotation))
        if len(rotation) == 3:
            rotation = tf.quaternion_from_euler(rotation[0], rotation[1], rotation[2], axes = 'rxyz')
        elif rotation.size == 16:
            rotation = tf.quaternion_from_matrix(rotation)
        else:
            pass
        rotation = rotation / np.linalg.norm(rotation)
        self.transform.rotation.x = rotation[0]
        self.transform.rotation.y = rotation[1]
        self.transform.rotation.z = rotation[2]
        self.transform.rotation.w = rotation[3]

    @property
    def matrix(self):
        return

class TFInterface(object):
    def __init__(self, robot_id:int = 1):
        rospy.loginfo("tf interface imported")
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.stamp_list = []
        
        json_file = os.path.abspath(os.path.join(rospkg.RosPack().get_path('crinmi_mr'),'config', 'robot_camera_calibration', 'calibration{}.json'.format(robot_id)))
        # Load configuation file
        with open(json_file,'r') as f:
            cfg = json.load(f)
            
        # tf
        self.tf_base2eef  = Crinmi_TransformStamped(
            parent_id = "base_link", 
            child_id = "eef", 
            transform = [0, 0, 0], 
            rotation = [0, 0, 0],
            m = False, deg = True
            )
        # static tf
        self.tf_eef2gripper = Crinmi_TransformStamped(
            parent_id = "eef", 
            child_id = "gripper", 
            transform = [0, -246, 0], 
            rotation = [90, 0, cfg["home_angle"]],
            m = False, deg = True
            )
        t_gripper2cam = np.array(cfg["calibration_data"])
        t_gripper2cam = t_gripper2cam * np.array([1, 1, -1])
        self.tf_gripper2cam = Crinmi_TransformStamped(
            parent_id = "gripper", 
            child_id = "camera_calibration", 
            transform = t_gripper2cam, 
            rotation = [0, 0, -180], 
            m = False, deg = True
            )
        self.tf_cam2cam_link = Crinmi_TransformStamped(
            parent_id = "camera_calibration",
            child_id = "camera_link",
            transform = [0, 0, 0],
            rotation = np.array([-0.5, 0.49999, -0.5, -0.5000001]),
            m = False, deg = False
            )
        
        self.tf_cam2assemble  = Crinmi_TransformStamped(
            parent_id = "camera_calibration", 
            child_id = "assemble_part", 
            transform = [0, 0, 0], 
            rotation = [0, 0, 0],
            m = False, deg = True
            )
        
        self.static_broadcaster.sendTransform([self.tf_eef2gripper, self.tf_gripper2cam, self.tf_cam2cam_link])
        duration = rospy.Duration(0.1)
        timer = rospy.Timer(duration, self.broadcast)

        self.buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.buffer)

    def broadcast_once(self):
        self.broadcaster.sendTransform([self.tf_base2eef])
        self.broadcaster.sendTransform([self.tf_cam2assemble])
        self.broadcaster.sendTransform(self.stamp_list)

    def broadcast(self, event):
        self.tf_base2eef.header.stamp = rospy.Time.now()
        self.tf_cam2assemble.header.stamp = rospy.Time.now()
        for id in self.stamp_list:
            id.header.stamp = rospy.Time.now()
        self.broadcaster.sendTransform([self.tf_base2eef])
        self.broadcaster.sendTransform([self.tf_cam2assemble])
        return self.broadcaster.sendTransform(self.stamp_list)

    def add_stamp(self, parent_id, child_id, pose, m = True, deg = True):
        is_exist = False
        for id in self.stamp_list:
            if id.child_frame_id == child_id:
                is_exist = True
                _temp_stamp = id
                break
        if is_exist is not True:
            _temp_stamp = Crinmi_TransformStamped(
                parent_id = parent_id, 
                child_id = child_id,
                m = False, deg = True
                )
            self.stamp_list.append(_temp_stamp)
        self.set_tf_pose(_temp_stamp, pose, m = m, deg = deg)
        
    def del_stamp(self, child_id):
        is_exist = False
        for id in self.stamp_list:
            if id.child_frame_id == child_id:
                is_exist = True
                self.stamp_list.remove(id)
                break
        return is_exist

    def check_stamp(self):
        list(map(lambda x: print(x[0], x[1].child_frame_id), enumerate(self.stamp_list)))
    
    def set_tf_pose(self, stampe, pose, m = True, deg = True):
        if len(pose) >= 6:
            stampe.t(pose[0:3], m)
            stampe.r(pose[3:len(pose)], deg)
        else:
            stampe.t(pose, m)
            stampe.r(pose, False)

    
    def matrix(self, target, source):
        _stamp = self.buffer.lookup_transform(target_frame=target, source_frame=source, time=rospy.Time())
        _matrix = tf.quaternion_matrix([
            _stamp.transform.rotation.x,
            _stamp.transform.rotation.y,
            _stamp.transform.rotation.z,
            _stamp.transform.rotation.w,
        ])
        _matrix[:3,3] = np.array([
            _stamp.transform.translation.x,
            _stamp.transform.translation.y,
            _stamp.transform.translation.z
            ])
        return _matrix

if __name__ == '__main__':
    rospy.init_node("tf2_broadcaster")
    module = TFInterface(1)
    rospy.sleep(2)

    
