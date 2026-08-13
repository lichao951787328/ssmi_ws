#!/usr/bin/env python3
from __future__ import division
from __future__ import print_function

import sys
import rospy
import numpy as np
import cv2
import message_filters
import time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import PointCloud2
from ssmi_sensors.semantic_sensor import PointType, SemanticPclGenerator


class SemanticCloud:
    """
    Class for ros node to take in a color image (bgr) and a semantic segmentation image (bgr)
    Then produce point cloud based on depth information
    """
    def __init__(self):
        robot_name = '/' + rospy.get_param("~agent_name", 'husky')
        
        # Get point type
        point_type = rospy.get_param(robot_name + '/semantic_pcl/point_type')
        if point_type == 0:
            self.point_type = PointType.SEMANTIC
            print('Generate semantic point cloud.')
        else:
            print("Invalid point type.")
            return
        # Get Unit conversion factor
        unit_conversion = rospy.get_param(robot_name + '/semantic_pcl/unit_conversion')
        # Get image size
        self.img_width, self.img_height = rospy.get_param(robot_name + '/camera/width'), rospy.get_param(robot_name + '/camera/height')
        # Set up ROS
        self.bridge = CvBridge() # CvBridge to transform ROS Image message to OpenCV image
        # Set up ros image subscriber
        # Set buff_size to average msg size to avoid accumulating delay
        # Point cloud frame id
        frame_id = rospy.get_param(robot_name + '/semantic_pcl/frame_id')
        # Camera intrinsic matrix
        fx = rospy.get_param(robot_name + '/camera/fx')
        fy = rospy.get_param(robot_name + '/camera/fy')
        cx = rospy.get_param(robot_name + '/camera/cx')
        cy = rospy.get_param(robot_name + '/camera/cy')
        intrinsic = np.matrix([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype = np.float32)
        dynamic_rgb = rospy.get_param(robot_name + '/semantic_pcl/dynamic_rgb', [255, 0, 255])
        self.depth_edge_filter_enabled = rospy.get_param(
            robot_name + '/semantic_pcl/depth_edge_filter/enabled', True)
        self.depth_edge_absolute_threshold = rospy.get_param(
            robot_name + '/semantic_pcl/depth_edge_filter/absolute_threshold', 0.15)
        self.depth_edge_relative_threshold = rospy.get_param(
            robot_name + '/semantic_pcl/depth_edge_filter/relative_threshold', 0.03)
        self.reject_semantic_boundaries = rospy.get_param(
            robot_name + '/semantic_pcl/depth_edge_filter/reject_semantic_boundaries', True)
        
        self.pcl_pub = rospy.Publisher(rospy.get_param(robot_name + '/octomap/pointcloud_topic'), PointCloud2, queue_size = 1)

        # increase buffer size to avoid delay (despite queue_size = 1)
        self.color_sub = message_filters.Subscriber(rospy.get_param(robot_name + '/semantic_pcl/color_image_topic'), Image,
                                                    queue_size = 1, buff_size = 30*self.img_width*self.img_height)
        self.semantic_sub = message_filters.Subscriber(rospy.get_param(robot_name + '/semantic_pcl/semantic_image_topic'), Image,
                                                       queue_size = 1, buff_size = 30*self.img_width*self.img_height)
        self.depth_sub = message_filters.Subscriber(rospy.get_param(robot_name + '/semantic_pcl/depth_image_topic'), Image,
                                                    queue_size = 1, buff_size = 30*self.img_width*self.img_height)

        # Take in color image, semantic image, and depth image with a limited time gap between message time stamps
        synchronization_slop = rospy.get_param(
            robot_name + '/semantic_pcl/synchronization_slop', 0.02)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.semantic_sub, self.depth_sub],
            queue_size = 5, slop = synchronization_slop)
        self.ts.registerCallback(self.color_semantic_depth_callback)
        self.cloud_generator = SemanticPclGenerator(intrinsic, self.img_width,self.img_height, unit_conversion, frame_id,
                                                    self.point_type, dynamic_rgb)
        print('Semantic point cloud ready!')

    def filter_depth_edges(self, depth_img, semantic_img):
        """Reject mixed foreground/background pixels without smoothing depth."""
        depth = depth_img.copy()
        valid = np.isfinite(depth) & (depth > 0.0)
        reject = np.zeros(depth.shape, dtype=bool)

        def reject_discontinuity(a, b, valid_a, valid_b, reject_a, reject_b):
            threshold = np.maximum(
                self.depth_edge_absolute_threshold,
                self.depth_edge_relative_threshold * np.minimum(a, b))
            discontinuity = valid_a & valid_b & (np.abs(a - b) > threshold)
            reject_a |= discontinuity
            reject_b |= discontinuity

        reject_discontinuity(
            depth[:, :-1], depth[:, 1:], valid[:, :-1], valid[:, 1:],
            reject[:, :-1], reject[:, 1:])
        reject_discontinuity(
            depth[:-1, :], depth[1:, :], valid[:-1, :], valid[1:, :],
            reject[:-1, :], reject[1:, :])

        if self.reject_semantic_boundaries:
            horizontal_boundary = np.any(
                semantic_img[:, :-1] != semantic_img[:, 1:], axis=2)
            vertical_boundary = np.any(
                semantic_img[:-1, :] != semantic_img[1:, :], axis=2)
            reject[:, :-1] |= horizontal_boundary
            reject[:, 1:] |= horizontal_boundary
            reject[:-1, :] |= vertical_boundary
            reject[1:, :] |= vertical_boundary

        depth[~valid | reject] = np.nan
        return depth

    def color_semantic_depth_callback(self, color_img_ros, semantic_img_ros , depth_img_ros):
        """
        Callback function to produce point cloud registered with semantic class color based
        on input color image and depth image
        """
        # Convert ros Image message to numpy array
        try:
            color_img = self.bridge.imgmsg_to_cv2(color_img_ros, "bgr8")
            semantic_img = self.bridge.imgmsg_to_cv2(semantic_img_ros, "bgr8")
            depth_img = self.bridge.imgmsg_to_cv2(depth_img_ros, "32FC1")
        except CvBridgeError as e:
            print(e)

        # Resize depth
        if depth_img.shape[0] != self.img_height or depth_img.shape[1] != self.img_width:
            depth_img = cv2.resize(depth_img, (self.img_width, self.img_height), interpolation = cv2.INTER_NEAREST)
            depth_img = depth_img.astype(np.float32)

        # Resize semantic
        if semantic_img.shape[0] != self.img_height or semantic_img.shape[1] != self.img_width:
            semantic_img = cv2.resize(semantic_img, (self.img_width, self.img_height), interpolation = cv2.INTER_NEAREST)
            semantic_img = semantic_img.astype(np.uint8)

        if self.depth_edge_filter_enabled:
            depth_img = self.filter_depth_edges(depth_img, semantic_img)
        
        if self.point_type == PointType.SEMANTIC:
            cloud_ros = self.cloud_generator.generate_cloud_semantic(color_img, semantic_img, depth_img,
                                                                     depth_img_ros.header.stamp)
        else:
            print('Point type not supported!')

        # Publish point cloud
        self.pcl_pub.publish(cloud_ros)
        

def main(args):
    rospy.init_node('semantic_cloud', anonymous=True)
    sem_cloud = SemanticCloud()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Semantic cloud shutting down!")


if __name__ == '__main__':
    main(sys.argv)
