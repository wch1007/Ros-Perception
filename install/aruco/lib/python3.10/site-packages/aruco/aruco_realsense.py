#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseArray, Vector3
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout
from cv_bridge import CvBridge
import cv2
import numpy as np
import cv2.aruco as aruco
import time
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        
        # Create CV bridge
        self.bridge = CvBridge()
        
        # Marker size in meters
        self.marker_size = 0.2
        
        # Reduce processing frequency with timer
        self.timer_period = 0.2  # 5Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.last_process_time = time.time()
        
        # Create publishers
        self.marker_pose_pub = self.create_publisher(PoseStamped, '/aruco/marker_pose', 10)
        self.marker_array_pub = self.create_publisher(PoseArray, '/aruco/marker_array', 10)
        
        # Publisher for custom pose format with RPY (x, y, z, roll, pitch, yaw)
        self.marker_pose_rpy_pub = self.create_publisher(Float64MultiArray, '/aruco/marker_pose_rpy', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to color image topic
        self.color_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.color_callback,
            10)
        # Subscribe to depth image topic
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10)
        # Subscribe to camera info topic
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            10)
        # Initialize variables
        self.color_image = None
        self.depth_image = None
        self.camera_matrix = None
        self.dist_coeffs = None
        # Initialize ArUco detector
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)
        # Create windows
        cv2.namedWindow('RGB Image', cv2.WINDOW_NORMAL)
        cv2.namedWindow('ArUco Detection', cv2.WINDOW_NORMAL)
        self.get_logger().info('Aruco detector node started.')
        
        # 3D points of the marker in marker coordinate system
        self.obj_points = self.create_marker_object_points()
    
    def create_marker_object_points(self):
        """Create 3D points for the marker (four corners)"""
        half_size = self.marker_size / 2.0
        
        # Define points in clockwise order
        objPoints = np.zeros((4, 1, 3), dtype=np.float32)
        objPoints[0, 0, 0] = -half_size  # Top-left
        objPoints[0, 0, 1] = half_size
        objPoints[0, 0, 2] = 0
        
        objPoints[1, 0, 0] = half_size   # Top-right
        objPoints[1, 0, 1] = half_size
        objPoints[1, 0, 2] = 0
        
        objPoints[2, 0, 0] = half_size   # Bottom-right
        objPoints[2, 0, 1] = -half_size
        objPoints[2, 0, 2] = 0
        
        objPoints[3, 0, 0] = -half_size  # Bottom-left
        objPoints[3, 0, 1] = -half_size
        objPoints[3, 0, 2] = 0
        
        return objPoints
    
    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (Roll, Pitch, Yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Convert to degrees
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        
        return roll, pitch, yaw, roll_deg, pitch_deg, yaw_deg
    
    def camera_info_callback(self, msg):
        """Handle camera intrinsic info"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Received camera intrinsics.')
    
    def color_callback(self, msg):
        """Handle color image"""
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Show original RGB image
            cv2.imshow('RGB Image', self.color_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error processing color image: {str(e)}')
    
    def depth_callback(self, msg):
        """Handle depth image"""
        try:
            # Depth image is usually uint16, convert to float32 for processing
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            # For visualization, normalize depth image to 0-255
            if self.depth_image is not None:
                depth_vis = np.clip(self.depth_image, 0, 4000)  # up to 4 meters
                depth_vis = (depth_vis / 4000.0 * 255).astype(np.uint8)
                cv2.imshow('Depth Image', depth_vis)
                cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')
    
    def timer_callback(self):
        """Process frames at reduced frequency"""
        current_time = time.time()
        if current_time - self.last_process_time >= self.timer_period:
            self.process_frame()
            self.last_process_time = current_time
    
    def process_frame(self):
        """Process image frame and detect ArUco markers"""
        if self.color_image is None or self.depth_image is None or self.camera_matrix is None:
            self.get_logger().debug('Waiting for all data (color, depth, camera_info)...')
            return
        
        detection_image = self.color_image.copy()
        # Detect ArUco markers
        corners, ids, rejected = self.detector.detectMarkers(detection_image)
        
        # Prepare PoseArray message
        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        pose_array_msg.header.frame_id = "camera_color_optical_frame"
        
        if ids is not None:
            aruco.drawDetectedMarkers(detection_image, corners, ids)
            
            # Store rotation and translation vectors for all markers
            rvecs = []
            tvecs = []
            
            for i, corner in enumerate(corners):
                # Reshape corners to match solvePnP input format
                corners_reshape = corner.reshape(4, 1, 2)
                
                # Estimate pose using solvePnP
                retval, rvec, tvec = cv2.solvePnP(
                    self.obj_points, corners_reshape, 
                    self.camera_matrix, self.dist_coeffs)
                    
                rvecs.append(rvec)
                tvecs.append(tvec)
                
                # Draw coordinate axes
                cv2.drawFrameAxes(detection_image, self.camera_matrix, 
                                self.dist_coeffs, rvec, tvec, 0.1)
                
                # Get marker center and check bounds
                marker_center = np.mean(corners[i][0], axis=0).astype(int)
                y, x = marker_center[1], marker_center[0]
                h, w = self.depth_image.shape[:2]
                
                # Marker ID
                marker_id = ids[i][0]
                
                # Convert rotation vector to quaternion
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                rotation_matrix_3x3 = np.eye(4)
                rotation_matrix_3x3[:3, :3] = rotation_matrix
                
                # Convert rotation matrix to quaternion
                qw = np.sqrt(1 + rotation_matrix[0, 0] + rotation_matrix[1, 1] + rotation_matrix[2, 2]) / 2
                qx = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / (4 * qw)
                qy = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / (4 * qw)
                qz = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / (4 * qw)
                
                # Convert quaternion to Euler angles (Roll, Pitch, Yaw)
                roll, pitch, yaw, roll_deg, pitch_deg, yaw_deg = self.quaternion_to_euler(qx, qy, qz, qw)
                
                # Create PoseStamped message
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "camera_color_optical_frame"
                
                # Set position
                pose_msg.pose.position.x = tvec[0][0]
                pose_msg.pose.position.y = tvec[1][0]
                pose_msg.pose.position.z = tvec[2][0]
                
                # Set orientation (still using quaternion for message)
                pose_msg.pose.orientation.x = qx
                pose_msg.pose.orientation.y = qy
                pose_msg.pose.orientation.z = qz
                pose_msg.pose.orientation.w = qw
                
                # Publish individual marker pose
                self.marker_pose_pub.publish(pose_msg)
                
                # Create and publish RPY format message (x, y, z, roll, pitch, yaw)
                pose_rpy_msg = Float64MultiArray()
                
                # Add dimensions information to make it easier to interpret
                pose_rpy_msg.layout.dim = [
                    MultiArrayDimension(
                        label="pose_with_rpy",
                        size=7,  # ID + x,y,z + roll,pitch,yaw
                        stride=7
                    )
                ]
                
                # Set data: marker_id, x, y, z, roll, pitch, yaw (in radians)
                pose_rpy_msg.data = [
                    float(marker_id),
                    tvec[0][0],   # x
                    tvec[1][0],   # y
                    tvec[2][0],   # z
                    roll,         # roll (radians)
                    pitch,        # pitch (radians)
                    yaw           # yaw (radians)
                ]
                
                # Publish pose with RPY
                self.marker_pose_rpy_pub.publish(pose_rpy_msg)
                
                # Add to pose array
                pose_array_msg.poses.append(pose_msg.pose)
                
                # Publish TF transform
                transform = TransformStamped()
                transform.header.stamp = self.get_clock().now().to_msg()
                transform.header.frame_id = "camera_color_optical_frame"
                transform.child_frame_id = f"aruco_marker_{marker_id}"
                
                # Set transform position
                transform.transform.translation.x = tvec[0][0]
                transform.transform.translation.y = tvec[1][0]
                transform.transform.translation.z = tvec[2][0]
                
                # Set transform orientation
                transform.transform.rotation.x = qx
                transform.transform.rotation.y = qy
                transform.transform.rotation.z = qz
                transform.transform.rotation.w = qw
                
                # Broadcast transform
                self.tf_broadcaster.sendTransform(transform)
                
                # Check if within depth image bounds and get depth info
                depth_info = "N/A"
                if 0 <= y < h and 0 <= x < w:
                    depth = self.depth_image[y, x]
                    depth_info = f"{depth/1000:.3f}m"
                
                # Display information on image
                text = f"ID: {marker_id}, Depth: {depth_info}"
                cv2.putText(detection_image, text, 
                          (int(corners[i][0][0][0]), int(corners[i][0][0][1])-10),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Print detailed marker information in terminal
                self.get_logger().info(f"\nMarker ID: {marker_id} Position: "
                                     f"x={tvec[0][0]:.3f}m, y={tvec[1][0]:.3f}m, z={tvec[2][0]:.3f}m, "
                                     f"Depth: {depth_info}, "
                                     f"Orientation (RPY): Roll={roll_deg:.2f}°, Pitch={pitch_deg:.2f}°, Yaw={yaw_deg:.2f}°")
            
            # Publish pose array
            self.marker_array_pub.publish(pose_array_msg)
        else:
            self.get_logger().debug('No markers detected.')
        
        # Show detection result
        cv2.imshow('ArUco Detection', detection_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main() 