#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import struct
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

import rospy
import rospkg
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray, Marker

CLASS = {
    'a_L_shape_angle'    : '0',
    'a_bearing_holder'   : '1',
    'a_cam_zig'          : '2',
    'a_hex_wrench'       : '3',
    'a_rod_end_bearing'  : '4',
    'a_roller'           : '5',
    'g_L_shape_angle'    : '6',
    'g_bearing_holder_x' : '7',
    'g_bearing_holder_y' : '8',
    'g_cam_zig_x'        : '9',
    'g_cam_zig_y'        : '10',
    'g_hex_wrench_x'     : '11',
    'g_hex_wrench_y'     : '12',
    'g_rod_end_bearing_x': '13',
    'g_rod_end_bearing_y': '14',
    'g_roller_x'         : '15',
    'g_roller_y'         : '16',
}

class VisualizeInterface(object):
    def __init__(self):
        rospy.loginfo("[Visualize Interface] Imported")
        self.pcd_publisher = rospy.Publisher('mr_vis/pcd_raw', PointCloud2, queue_size=2)
        self.target_pcd_publisher = rospy.Publisher('mr_vis/target_pcd_raw', PointCloud2, queue_size=2)
        self.mesh_publisher = rospy.Publisher('mr_vis/meshes', MarkerArray, queue_size=2)
        self.test_publisher = rospy.Publisher('mr_vis/test', PointCloud2, queue_size=2)

        mesh_dir    = os.path.abspath(os.path.join(rospkg.RosPack().get_path('crinmi_mr'),'mesh'))
        self.markerArray = MarkerArray()

        files = [file for file in os.listdir(mesh_dir) if file.endswith(('.obj', '.stl'))]
        for marker_id, file in enumerate(files):
            rospy.loginfo('Loading file: %s', file)
            marker = Marker()
            marker.id = marker_id
            marker.mesh_resource =  "package://crinmi_mr/mesh/" + file
            marker.type = marker.MESH_RESOURCE
            marker.header.frame_id = "asset_" + str(CLASS[os.path.splitext(file)[0]])
            marker.scale.x = 0.001
            marker.scale.y = 0.001
            marker.scale.z = 0.001
            marker.color.a = 1.0
            marker.color.r = 0.8
            marker.color.g = 0.8
            marker.color.b = 0.8
            marker.pose.orientation.w = 1.0
            self.markerArray.markers.append(marker)

    def pub_pcd(self, pcd):
        _header = Header()
        _header.frame_id = "camera_calibration"
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

    def pub_test_pcd(self, pcd):
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

        self.test_publisher.publish(pcd_msg)


    def pub_target_pcd(self, pcd):
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

        self.target_pcd_publisher.publish(pcd_msg)
    
    def pub_mesh(self):
        # for asset in self.markerArray.markers:
            # asset.header.stamp = rospy.Time.now()
        self.mesh_publisher.publish(self.markerArray)