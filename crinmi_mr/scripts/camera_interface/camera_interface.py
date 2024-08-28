#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import cv2
from cv_bridge import CvBridge
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import pickle

import rospy
import rospkg

from sensor_msgs.msg import Image, CameraInfo

class CameraInterface(object):
    def __init__(self):
        rospy.loginfo("camera interface imported")
        self.cv_bridge = CvBridge()
        rospy.Subscriber('/camera/color/image_raw', Image, self.read_color_cb)
        rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.read_color_cam_info_cb)
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.read_depth_cb)
        rospy.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo, self.read_depth_cam_info_cb)
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
        return self.cv_bridge.imgmsg_to_cv2(self.color_img_msg, "bgr8")
    
    @property
    def save_color_img(self) -> np.ndarray:
        """Converted color image with numpy array from the subscribed color image topic.

        Returns:
            `numpy.ndarray`: (H, W, C) with `uint8` color image.
        """
        return self.cv_bridge.imgmsg_to_cv2(self.color_img_msg, "bgr8")

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
            # img = self.cv_bridge.imgmsg_to_cv2(self.depth_img_msg, desired_encoding="passthrough")
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
    def keti_depth_img(self) -> np.ndarray:
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
        return (restored_depth_image*1000).astype(np.int16)

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

    def save_image(self, name):
        save_dir = os.path.abspath(os.path.join(rospkg.RosPack().get_path('crinmi_mr'),'image'))
        cv2.imwrite(save_dir + '/' + name + '_image.jpg', self.save_color_img)
        cv2.imwrite(save_dir + '/' + name + '_depth.jpg', self.depth_img)
        with open(save_dir + name + '_topic.p', 'wb') as f:
            pickle.dump([self.color_img_msg,
                         self.depth_img_msg,
                         self.color_cam_info_msg,
                         self.depth_cam_info_msg,
                         ], f)
        
    def read_image(self, name, vis:bool = False):
        save_dir = os.path.abspath(os.path.join(rospkg.RosPack().get_path('crinmi_mr'),'image'))
        with open(save_dir + '/' + name + '.p', 'rb') as f:
            topic_list = pickle.load(f)
            self.color_img_msg = topic_list[0]
            self.depth_img_msg = topic_list[1]
            self.color_cam_info_msg = topic_list[2]
            self.depth_cam_info_msg = topic_list[3]
        if vis: self.vis_image()
    
    def temp_read_state(self, name, vis:bool = False):
        save_dir = os.path.abspath(os.path.join(rospkg.RosPack().get_path('crinmi_mr'),'image'))
        with open(save_dir + '/' + name + '.p', 'rb') as f:
            topic_list = pickle.load(f)
            return topic_list[4]
    

    def depth2pcd(self, depth: np.array, extr: np.array = np.eye(4)) ->np.array:
        """Convert depth image to pointcloud data.

        Args:
            depth_image (np.array): (H, W) depth image to convert.
            intr (np.array): (3, 3) camera intrinsic matrix.
            extr (np.array): (3, 3) camera extrinsic matrix.

        Returns:
            np.array: (N, 3) pointcloud data array converted from depth image
        """
        height, width = depth.shape
        row_indices = np.arange(height)
        col_indices = np.arange(width)
        pixel_grid = np.meshgrid(col_indices, row_indices)
        pixels = np.c_[pixel_grid[0].flatten(), pixel_grid[1].flatten()].T
        pixels_homog = np.r_[pixels, np.ones([1, pixels.shape[1]])]
        depth_arr = np.tile(depth.flatten(), [3, 1])
        point_cloud = depth_arr * np.linalg.inv(self.depth_cam_intr).dot(pixels_homog)
        point_cloud = point_cloud.transpose()

        return (np.matmul(extr[:3,:3], point_cloud[:,:3].T) + extr[:3,3].reshape(3,1)).T

    def pcd(self, extr: np.array = np.eye(4)):
        """_summary_
        """
        return self.depth2pcd(self.depth_img, extr)
        
    def pixel_to_3d(self, pixel_x, pixel_y):
        """
        Transform 2d pixel coordinate points to 3d camera coordinate points
        """
        
        if self.depth_img is None or self.color_cam_intr is None:
            rospy.logwarn("Depth image or camera matrix not available")
            return None
            
        depth = self.depth_img[pixel_y, pixel_x]

        fx = self.color_cam_intr[0, 0]
        fy = self.color_cam_intr[1, 1]
        cx = self.color_cam_intr[0, 2]
        cy = self.color_cam_intr[1, 2]

        x = (pixel_x - cx) * depth / fx
        y = (pixel_y - cy) * depth / fy
        z = depth

        return x, y, z

    @staticmethod
    def hsv_segment_h(image, segment):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        h = h * segment
        threshold2 = np.mean(h[np.where(s > 0)])
        thresh_h = cv2.threshold(h, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
        return (thresh_h.astype(float)/255).astype(np.uint8)

    @staticmethod
    def hsv_segment_s(image, segment):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        s = s * segment
        threshold2 = np.mean(s[np.where(s > 0)])
        thresh_h = cv2.threshold(s, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
        return (thresh_h.astype(float)/255).astype(np.uint8)

    @staticmethod
    def hsv_segment_v(image, segment):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        v = v * segment
        threshold2 = np.mean(v[np.where(s > 0)])
        thresh_h = cv2.threshold(v, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
        return (thresh_h.astype(float)/255).astype(np.uint8)

    def show_intrinsic(self):
        """_summary_
        """
        rospy.loginfo(self.color_cam_intr)
        rospy.loginfo(self.depth_cam_intr)

    def vis_image(self):
        """_summary_
        """
        fig = plt.figure()
        image = fig.add_subplot(211)
        image.imshow(self.color_img)
        depth = fig.add_subplot(212)
        depth.imshow(self.depth_img)
        plt.show()

    def vis_pcd(self, pcd:np.array, extr:np.array = np.eye(4), reduction_ratio: int = 50):
        """_summary_
        """
        pcd_s = pcd[np.arange(1,pcd.shape[0], reduction_ratio)]
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(pcd_s[:, 0], pcd_s[:, 1], pcd_s[:, 2])
        ax.axis("equal")
        plt.show()
        # open3d.visualization.draw_geometries([pcd],
        #                           zoom=0.3412,
        #                           front=[0.4257, -0.2125, -0.8795],
        #                           lookat=[2.6172, 2.0475, 1.532],
        #                           up=[-0.0694, -0.9768, 0.2024])

    @staticmethod
    def vis_segment(segment_list):
        fig = plt.figure()
        axis = int(np.ceil(np.sqrt(len(segment_list))))
        for idx, seg in enumerate(segment_list):
            image = fig.add_subplot(axis, axis, idx + 1)
            image.imshow(seg[0])
        plt.show()

    @staticmethod
    def vis_image_segment(segment_list, image):
        axis = int(np.ceil(np.sqrt(len(segment_list))))
        
        fig = plt.figure()
        image_axis = fig.add_subplot(1, 1, 1)
        image_axis.imshow(image)
        plt.show()

        fig = plt.figure()
        for idx, seg in enumerate(segment_list):
            h_img = CameraInterface.hsv_segment_h(image, seg[0])
            s_img = CameraInterface.hsv_segment_s(image, seg[0])
            image_axis = fig.add_subplot(axis, axis, idx + 1)
            image_axis.imshow(h_img * s_img)
        plt.show()

        
if __name__ == '__main__':
    rospy.init_node('camera_interface')
    module = CameraInterface()

    import sys
    import os
    current_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    sys.path.append(current_dir)
    from segment_interface.yolo_segment_interface import SegmentInterface
    segment_server = SegmentInterface()

    # name = "guide_0060"
    # name = "guide_0106"
    # name = "guide_0119"
    # name = "guide_0142"
    name = "depth_0061"
    module.read_image(name)
    # module.vis_image()
    segment_server.run(module.color_img, False)
    seg = segment_server.img2SegmentMask()
    module.vis_image_segment(seg, module.color_img)

