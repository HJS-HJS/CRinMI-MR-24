#!/usr/bin/env python3
# coding: utf-8
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import pickle

if __name__ == "__main__":
    rospy.init_node('tutorial', anonymous=True)
    
    with open("/home/kkw0418/query/pcd/image36.pkl","rb") as fr:
        data = pickle.load(fr)

    pub = rospy.Publisher("/depth_points", PointCloud2, queue_size=1)
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        print("publishing")
        pub.publish(data)
        print("published")
        r.sleep()
    
    rospy.spin()