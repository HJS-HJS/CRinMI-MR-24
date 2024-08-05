#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import pickle
import rospkg
import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
# OpenCV2 for saving an image
from cv_bridge import CvBridge, CvBridgeError
import cv2
# PCL for saving a point cloud
# import sensor_msgs.point_cloud2 as pc2
# import pcl


class Recorder:
    """
    Save images and camera parameters.
    Please keep the camera static during recording.

    Usage:
        1. Run Azure Kinect node: roslaunch azure_kinect_ros_driver kinect_rgbd.launch
        2. Run this node: rosrun bundle_adjustment record_samples.py _img_id:=0
    """

    def __init__(self, img_id: int):
        # Save directory
        pkg_dir = "/home/kkw0418/"
        self.save_dir_img = os.path.join(pkg_dir, 'query/image')
        self.save_dir_pcd = os.path.join(pkg_dir, 'query/pcd')
        self.status = {
            'rgb': False,
            'rgb_info': False,
            'pc': False,
            'pc_info': False
        }
        self.img_id = img_id

        # Azure Kinect Subscribers
        rospy.Subscriber('/rgb/image_raw', Image, self.callback_rgb_raw)
        # rospy.Subscriber('/rgb/image_rect_color', Image, self.callback_rgb)
        # rospy.Subscriber('/rgb/camera_info', CameraInfo, self.callback_rgb_info)
        rospy.Subscriber('/depth_to_rgb/points', PointCloud2, self.callback_pc)
        # rospy.Subscriber('/depth_to_rgb/camera_info', CameraInfo, self.callback_pc_info)

    @property
    def is_done(self):
        return all(self.status.values())

    def callback_rgb_raw(self, data: Image):
        # Skip if already saved
        if self.status['rgb']:
            return
        # Save as jpg
        fname = os.path.join(self.save_dir_img, f'image{self.img_id}.jpg')
        try:
            cv_image = CvBridge().imgmsg_to_cv2(data, 'bgr8')
            cv2.imwrite(fname, cv_image)
        except CvBridgeError as e:
            rospy.logerr(e)
        self.status['rgb'] = True

    # def callback_rgb(self, data: Image):
    #     # Skip if already saved
    #     if self.status['rgb']:
    #         return
    #     # Save as jpg
    #     fname = os.path.join(self.save_dir, f'rgb+image_rect_color+{self.img_id}.jpg')
    #     try:
    #         cv_image = CvBridge().imgmsg_to_cv2(data, 'bgr8')
    #         cv2.imwrite(fname, cv_image)
    #     except CvBridgeError as e:
    #         rospy.logerr(e)
    #     self.status['rgb'] = True

    # def callback_rgb_info(self, data: CameraInfo):
    #     # Skip if file already exists
    #     fname = os.path.join(self.save_dir, 'rgb+camera_info.pkl')
    #     if os.path.exists(fname):
    #         self.status['rgb_info'] = True
    #         return
    #     # Save as pickle
    #     with open(fname, 'wb') as f:
    #         pickle.dump(data, f)
    #     self.status['rgb_info'] = True

    def callback_pc(self, data: PointCloud2):
        # Skip if already saved
        if self.status['pc']:
            return
        # Save as pickle
        fname = os.path.join(self.save_dir_pcd, f'image{self.img_id}.pkl')
        with open(fname, 'wb') as f:
            pickle.dump(data, f)
        # Save as pcd
        # fname = os.path.join(self.save_dir, f'depth_to_rgb+points+{self.img_id}.pcd')
        # TODO
        self.status['pc'] = True

    # def callback_pc_info(self, data: CameraInfo):
    #     # Skip if file already exists
    #     fname = os.path.join(self.save_dir, 'depth_to_rgb+camera_info.pkl')
    #     if os.path.exists(fname):
    #         self.status['pc_info'] = True
    #         return
    #     # Save as pickle
    #     with open(fname, 'wb') as f:
    #         pickle.dump(data, f)
    #     self.status['pc_info'] = True


if __name__ == '__main__':
    rospy.init_node('record_samples')
    # Get image ID
    if not rospy.has_param('~img_id'):
        rospy.logerr('Please specify image ID by rosparam: _img_id:=0')
        exit()
    img_id = int(rospy.get_param('~img_id'))
    rospy.loginfo(f'Image ID: {img_id}')

    # Run recorder
    recorder = Recorder(img_id)
    t = 0.1
    wait = 0
    while not rospy.is_shutdown():
        if recorder.is_done:
            break
        if wait > 5.0:
            rospy.logerr('Timeout')
            exit()
        rospy.sleep(t)
        wait += t
    rospy.loginfo('Done')