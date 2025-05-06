#!/usr/bin/env python3
"""
This ROS2 node detects ArUco markers using OpenCV and publishes their IDs and 3D poses.

Subscriptions:
   /camera/image_raw (sensor_msgs.msg.Image)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)

Published Topics:
    /aruco_poses (geometry_msgs.msg.PoseArray) — Pose of all detected markers (for RViz visualization)
    /aruco_markers (aruco_interfaces.msg.ArucoMarkers) — Array of marker IDs and corresponding poses
    /aruco_image (sensor_msgs.msg.Image) — Annotated image with marker bounding boxes

Parameters:
    marker_size (meters)
    aruco_dictionary_id (default: DICT_4X4_50)
    image_topic, depth_image_topic, camera_info_topic
    detected_markers_topic, markers_visualization_topic, output_image_topic
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray
from aruco_interfaces.msg import ArucoMarkers
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import numpy as np
import cv2
import time
import message_filters

from aruco_pose_estimation.utils import ARUCO_DICT
from aruco_pose_estimation.pose_estimation import pose_estimation


class ArucoNode(Node):
    def __init__(self):
        super().__init__("aruco_node")
        self.last_detection_time = 0  # Throttle detection frequency
        self.bridge = CvBridge()
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        self.initialize_parameters()

        try:
            dictionary_id = cv2.aruco.__getattribute__(self.dictionary_id_name)
            if dictionary_id not in ARUCO_DICT.values():
                raise AttributeError
        except AttributeError:
            self.get_logger().error(f"Invalid aruco_dictionary_id: {self.dictionary_id_name}")
            options = "\n".join([s for s in ARUCO_DICT])
            self.get_logger().error(f"Valid options:\n{options}")

        # Set up camera info subscriber
        self.info_sub = self.create_subscription(
            CameraInfo, self.info_topic, self.info_callback, qos_profile_sensor_data)

        # RGB-only mode
        if not self.use_depth_input:
            self.image_sub = self.create_subscription(
                Image, self.image_topic, self.image_callback, qos_profile_sensor_data)
        else:
            self.image_sub = message_filters.Subscriber(self, Image, self.image_topic, qos_profile_sensor_data)
            self.depth_image_sub = message_filters.Subscriber(self, Image, self.depth_image_topic, qos_profile_sensor_data)
            self.synchronizer = message_filters.ApproximateTimeSynchronizer(
                [self.image_sub, self.depth_image_sub], queue_size=10, slop=0.05)
            self.synchronizer.registerCallback(self.rgb_depth_sync_callback)

        self.poses_pub = self.create_publisher(PoseArray, self.markers_visualization_topic, 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, self.detected_markers_topic, 10)
        self.image_pub = self.create_publisher(Image, self.output_image_topic, 10)

        # 创建字典
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)

        # 使用自定义参数
        parameters = cv2.aruco.DetectorParameters()

        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        parameters.adaptiveThreshWinSizeMin = 3
        parameters.adaptiveThreshWinSizeMax = 23
        parameters.adaptiveThreshWinSizeStep = 10
        parameters.minMarkerPerimeterRate = 0.01
        parameters.maxMarkerPerimeterRate = 4.0
        parameters.polygonalApproxAccuracyRate = 0.03

        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, parameters)

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)

        self.get_logger().info("✅ Camera info received")
        self.get_logger().info(f"Intrinsic matrix:\n{self.intrinsic_mat}")
        self.get_logger().info(f"Distortion: {self.distortion}")
        self.get_logger().info(f"Resolution: {self.info_msg.width}x{self.info_msg.height}")

        self.destroy_subscription(self.info_sub)

    def image_callback(self, img_msg):
        if self.info_msg is None:
            return

        now = time.time()
        if now - self.last_detection_time < 1.0:
            return
        self.last_detection_time = now

        image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="rgb8")
        pose_array = PoseArray()
        markers = ArucoMarkers()

        pose_array.header.stamp = img_msg.header.stamp
        markers.header.stamp = img_msg.header.stamp
        pose_array.header.frame_id = self.camera_frame
        markers.header.frame_id = self.camera_frame

        frame, pose_array, markers = pose_estimation(
            rgb_frame=image,
            depth_frame=None,
            aruco_detector=self.aruco_detector,
            marker_size=self.marker_size,
            matrix_coefficients=self.intrinsic_mat,
            distortion_coefficients=self.distortion,
            pose_array=pose_array,
            markers=markers,
        )

        if len(markers.marker_ids) > 0:
            self.get_logger().info(f"[Detected] Marker IDs: {markers.marker_ids}")
            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))

    def rgb_depth_sync_callback(self, rgb_msg, depth_msg):
        # identical to image_callback, just include depth_frame
        pass  # 可选实现

    def initialize_parameters(self):
        self.declare_parameter("marker_size", 0.0625,
                               ParameterDescriptor(description="Marker size in meters."))
        self.declare_parameter("aruco_dictionary_id", "DICT_4X4_50")
        self.declare_parameter("use_depth_input", False)
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_image_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("detected_markers_topic", "/aruco_markers")
        self.declare_parameter("markers_visualization_topic", "/aruco_poses")
        self.declare_parameter("output_image_topic", "/aruco_image")

        self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value
        self.dictionary_id_name = self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        self.use_depth_input = self.get_parameter("use_depth_input").get_parameter_value().bool_value
        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.depth_image_topic = self.get_parameter("depth_image_topic").get_parameter_value().string_value
        self.info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        self.detected_markers_topic = self.get_parameter("detected_markers_topic").get_parameter_value().string_value
        self.markers_visualization_topic = self.get_parameter("markers_visualization_topic").get_parameter_value().string_value
        self.output_image_topic = self.get_parameter("output_image_topic").get_parameter_value().string_value

        self.get_logger().info(f"Marker size: {self.marker_size}")
        self.get_logger().info(f"Dictionary: {self.dictionary_id_name}")
        self.get_logger().info(f"Image topic: {self.image_topic}")
        self.get_logger().info(f"Depth input: {self.use_depth_input}")


def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
