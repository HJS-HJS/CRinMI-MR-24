#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
from cv_bridge import CvBridge
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import pickle

import rospy
from sensor_msgs.msg import Image, CameraInfo

class CameraInterface(object):
    def __init__(self):
        rospy.loginfo("camera interface imported")
        self.cv_bridge = CvBridge()
        rospy.Subscriber('/camera/color/image_raw', Image, self.read_color_cb)
        rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.read_color_cam_info_cb)
        rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.read_depth_cb)
        rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.read_depth_cam_info_cb)
        self.color_img_msg = Image()
        self.depth_img_msg = Image()
        self.color_cam_info_msg = CameraInfo()
        self.depth_cam_info_msg = CameraInfo()

    def read_color_cb(self, msg: Image):
        """Color image ROS subscriber callback.

        Args:
            msg (`sensor_msgs/Image`): Color image msg.
        """
        self.color_img_msg = msg

    def read_color_cam_info_cb(self, msg: CameraInfo):
        """Color camera information subscriber callback.
        
        Args:
            msg (`sensor_msgs/CameraInfo`): Color camera information msg.
        """
        self.color_cam_info_msg = msg

    def read_depth_cb(self, msg: Image):
        """Depth image ROS subcriber callback.

        Args:
            msg (`sensor_msgs/Image`): Depth image msg with `32FC1` or `16UC1`.
        """
        self.depth_img_msg = msg

    def read_depth_cam_info_cb(self, msg: CameraInfo):
        """Depth camera information ROS subscriber callback.

        Args:
            msg (`sensor_msgs/CameraInfo`): Depth camera information msg.
        """
        self.depth_cam_info_msg = msg

    @property
    def color_img(self) -> np.ndarray:
        """Converted color image with numpy array from the subscribed color image topic.

        Returns:
            `numpy.ndarray`: (H, W, C) with `uint8` color image.
        """
        return self.cv_bridge.imgmsg_to_cv2(self.color_img_msg, "rgb8")
    
    @property
    def depth_img(self) -> np.ndarray:
        """Depth image from the subscribed depth image topic.

        Returns:
            `numpy.ndarray`: (H, W) with `float32` depth image.
        """
        if self.depth_img_msg.encoding == '32FC1':
            img = self.cv_bridge.imgmsg_to_cv2(self.depth_img_msg)
        elif self.depth_img_msg.encoding == '16UC1':
            img = self.cv_bridge.imgmsg_to_cv2(self.depth_img_msg)
            img = (img/1000.).astype(np.float32)
        else:
            img = self.cv_bridge.imgmsg_to_cv2(self.depth_img_msg)

        # none to zero
        img = np.nan_to_num(img)

        # depth hole filling
        inpaint_mask = np.zeros(img.shape, dtype='uint8')
        inpaint_mask[img == 0] = 255
        restored_depth_image = cv2.inpaint(
            img,
            inpaint_mask,
            inpaintRadius=15,
            flags=cv2.INPAINT_NS
            )
        return restored_depth_image
    
    @property
    def color_cam_intr(self) -> np.ndarray:
        """Color camera intrinsic matrix.

        Returns:
            numpy.ndarray: (3, 3) camera intrinsic matrix.
        """
        intr = np.array(self.color_cam_info_msg.K)
        return intr.reshape(3, 3)

    @property
    def depth_cam_intr(self) -> np.ndarray:
        """Depth camera intrinsic matrix.

        Returns:
            numpy.ndarray: (3, 3) camera intrinsic matrix.
        """
        intr = np.array(self.depth_cam_info_msg.K)
        return intr.reshape(3, 3)

    def vis_image(self):
        """_summary_
        """
        fig = plt.figure()
        image = fig.add_subplot(211)
        image.imshow(self.color_img)
        depth = fig.add_subplot(212)
        depth.imshow(self.depth_img)
        plt.show()

    def show_intrinsic(self):
        """_summary_
        """
        rospy.loginfo(self.color_cam_intr)
        rospy.loginfo(self.depth_cam_intr)

    def save_image(self, name):
        cv2.imwrite('../../image/' + name + '_image.jpg', self.color_img)
        cv2.imwrite('../../image/' + name + '_depth.jpg', self.depth_img)
        with open('../../image/' + name + '_topic.p', 'wb') as f:
            pickle.dump([self.color_img_msg,
                         self.depth_img_msg,
                         self.color_cam_info_msg,
                         self.depth_cam_info_msg,
                         ], f)
        
    def read_image(self, name):
        with open('../../image/' + name + '_topic.p', 'rb') as f:
            topic_list = pickle.load(f)
            self.color_img_msg = topic_list[0]
            self.depth_img_msg = topic_list[1]
            self.color_cam_info_msg = topic_list[2]
            self.depth_cam_info_msg = topic_list[3]



if __name__ == '__main__':
    rospy.init_node('camera_interface')
    module = CameraInterface()
    rospy.loginfo('[Crinmi MR] Save/Read Image')
    name = "1"
    rospy.spin()
    module.save_image(name)
    # module.read_image(name)
    rospy.loginfo('[Crinmi MR] vis Image')
    module.vis_image()
    rospy.loginfo('[Crinmi MR] Show Intrinsic Data')
    module.show_intrinsic()
    rospy.loginfo('[Crinmi MR] Node Closed')
