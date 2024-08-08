#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import json
import numpy as np

import rospy
import rospkg

class TFInterface(object):
    def __init__(self, robot_id:int = 1):
        rospy.loginfo("calibration interface imported")

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

if __name__ == '__main__':
    module = TFInterface(1)
