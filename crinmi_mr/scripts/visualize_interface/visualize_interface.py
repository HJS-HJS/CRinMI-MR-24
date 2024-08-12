#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import struct
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

class VisualizeInterface(object):
    def __init__(self):
        rospy.loginfo("[Visualize Interface] Imported")
        self.pcd_publisher = rospy.Publisher('mr_vis/pcd_raw', PointCloud2, queue_size=2)

    def pub_pcd(self, pcd):
        _header = Header()
        _header.frame_id = "base_link"
        _header.stamp = rospy.Time.now()
        
        fields =[
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32,1),
            ]

        points = []
        for point in pcd:
            # print(point)
            rgb = struct.unpack('I', struct.pack('BBBB', 255, 255, 255, 255))[0]
            points.append([point[0], point[1], point[2], rgb])

        pcd_msg = point_cloud2.create_cloud(_header, fields, points)

        self.pcd_publisher.publish(pcd_msg)
