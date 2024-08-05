#!/usr/bin/env python3
# coding: utf-8
import rospy
import std_msgs
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np

def publish_pc2(pc, obj):
    """Publisher of PointCloud data"""
    pub = rospy.Publisher("/points_raw", PointCloud2, queue_size=1000000)
    rospy.init_node("pc2_publisher")
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "rgb_camera_link"
    points = pc2.create_cloud_xyz32(header, pc[:, :3])

    pub2 = rospy.Publisher("/points_raw1", PointCloud2, queue_size=1000000)
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "velodyne"
    points2 = pc2.create_cloud_xyz32(header, obj)

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        print("publishing")
        pub.publish(points)
        print("published")
        pub2.publish(points2)
        r.sleep() 

if __name__ == '__main__':
    
    pc = np.load('/home/kkw0418/query/p3d/xyzs.npy')
    scale = 7.37
    #T2
    # R = np.array([[ 0.93510613,  0.21354667, -0.28279737],
    #               [-0.33399425,  0.79780082, -0.50195786],
    #               [ 0.11842454,  0.56383657,  0.81735179]])
    # t = np.array([10.7640849,25.70171796,-8.65324132])
    
    #Query6
    R = np.array([[ 0.56668027,  0.54096057, -0.62147818],
                  [-0.73452436,  0.67341407, -0.08359103],
                  [ 0.3732927,   0.50386024, 0.77895919]])
    t = np.array([22.66769172, 11.6522564, -2.8668691])
    
    pc = pc*scale
    j = 0
    for i in pc:
        pc_temp = R.T@((i-t)/100)
        pc[j] = pc_temp
        j = j+1
    obj = pc
    publish_pc2(pc,obj)