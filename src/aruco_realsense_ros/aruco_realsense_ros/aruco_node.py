#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
import math
import time
from geometry_msgs.msg import TransformStamped
import random

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        
        # 标记尺寸 (以米为单位)
        self.marker_size = 0.05
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize ArUco detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        
        # 预定义标记的3D点（用于姿态估计）
        self.objPoints = self.create_marker_object_points()
        
        # Create publishers
        self.marker_pub = self.create_publisher(Marker, '/aruco/marker', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco/pose', 10)
        self.marker_pose_rpy_pub = self.create_publisher(Float64MultiArray, '/aruco/marker_pose_rpy', 10)
        self.camera_pose_pub = self.create_publisher(PoseStamped, '/aruco/camera_pose', 10)
        self.camera_pose_rpy_pub = self.create_publisher(Float64MultiArray, '/aruco/camera_pose_rpy', 10)
        self.tf_matrix_pub = self.create_publisher(Float64MultiArray, '/aruco/tf', 10)
        self.mimic_point_pub = self.create_publisher(Float64MultiArray, '/camera/mimic_point', 10)
        self.arm_mimic_point_pub = self.create_publisher(Float64MultiArray, '/arm/mimic_xyzrpt', 10)
        
        # Create transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 设置marker在arm坐标系中的位置
        self.marker_in_arm = {
            'x': 0.1,
            'y': -0.2,
            'z': 0.05,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0
        }
        
        # 创建定时器，用于发布模拟点
        self.create_timer(1.0, self.publish_mimic_point)  # 每秒发布一次
        
        # Subscribe to color image topic
        self.color_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.color_callback,
            10)
            
        # Subscribe to camera info topic
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            10)
            
        # Initialize variables
        self.color_image = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.current_tf_matrix = None
        self.marker_detected = False
        
        self.get_logger().info('ArUco detector node has been started')
    
    def publish_mimic_point(self):
        """发布模拟点数据"""
        try:
            # 生成随机点
            x = random.uniform(-0.5, 0.5)
            y = random.uniform(-0.5, 0.5)
            z = random.uniform(0.0, 1.0)
            roll = random.uniform(-math.pi/4, math.pi/4)
            pitch = random.uniform(-math.pi/4, math.pi/4)
            yaw = random.uniform(-math.pi/4, math.pi/4)
            
            # 发布模拟点
            mimic_msg = Float64MultiArray()
            mimic_msg.data = [x, y, z, roll, pitch, yaw]
            self.mimic_point_pub.publish(mimic_msg)
            
            # 只有在检测到marker时才进行坐标转换
            if self.marker_detected and self.current_tf_matrix is not None:
                # 创建点的变换矩阵
                point_matrix = self.create_transform_matrix(x, y, z, roll, pitch, yaw)
                
                # 进行坐标变换
                transformed_matrix = self.current_tf_matrix @ point_matrix
                
                # 从变换矩阵中提取位置和姿态
                transformed_position = transformed_matrix[:3, 3]
                transformed_rotation = transformed_matrix[:3, :3]
                
                # 计算欧拉角
                sy = math.sqrt(transformed_rotation[0,0] * transformed_rotation[0,0] + 
                             transformed_rotation[1,0] * transformed_rotation[1,0])
                singular = sy < 1e-6
                if not singular:
                    roll = math.atan2(transformed_rotation[2,1], transformed_rotation[2,2])
                    pitch = math.atan2(-transformed_rotation[2,0], sy)
                    yaw = math.atan2(transformed_rotation[1,0], transformed_rotation[0,0])
                else:
                    roll = math.atan2(-transformed_rotation[1,2], transformed_rotation[1,1])
                    pitch = math.atan2(-transformed_rotation[2,0], sy)
                    yaw = 0
                
                # 发布转换后的点
                arm_msg = Float64MultiArray()
                arm_msg.data = [float(transformed_position[0]), 
                              float(transformed_position[1]), 
                              float(transformed_position[2]), 
                              roll, pitch, yaw]
                self.arm_mimic_point_pub.publish(arm_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error publishing mimic point: {str(e)}')
    
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
    
    def create_transform_matrix(self, x, y, z, roll, pitch, yaw):
        """创建4x4变换矩阵"""
        # 创建旋转矩阵
        Rx = np.array([[1, 0, 0],
                      [0, np.cos(roll), -np.sin(roll)],
                      [0, np.sin(roll), np.cos(roll)]])
        
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                      [0, 1, 0],
                      [-np.sin(pitch), 0, np.cos(pitch)]])
        
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                      [np.sin(yaw), np.cos(yaw), 0],
                      [0, 0, 1]])
        
        R = Rz @ Ry @ Rx
        
        # 创建4x4变换矩阵
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        
        return T
    
    def calculate_transform_matrix(self, marker_pose):
        """计算从camera_link到arm_link的变换矩阵"""
        try:
            # 创建marker在camera_link中的变换矩阵
            marker_in_camera = self.create_transform_matrix(
                marker_pose[0],  # x
                marker_pose[1],  # y
                marker_pose[2],  # z
                marker_pose[3],  # roll
                marker_pose[4],  # pitch
                marker_pose[5]   # yaw
            )
            
            # 创建marker在arm_link中的变换矩阵
            marker_in_arm = self.create_transform_matrix(
                self.marker_in_arm['x'],
                self.marker_in_arm['y'],
                self.marker_in_arm['z'],
                self.marker_in_arm['roll'],
                self.marker_in_arm['pitch'],
                self.marker_in_arm['yaw']
            )
            
            # 计算camera_link到arm_link的变换矩阵
            # camera_to_arm = arm_to_marker * marker_to_camera
            camera_to_arm = marker_in_arm @ np.linalg.inv(marker_in_camera)
            
            return camera_to_arm
            
        except Exception as e:
            self.get_logger().error(f'Error calculating transform matrix: {str(e)}')
            return None
    
    def publish_transform_matrix(self, marker_id, marker_pose):
        """发布从camera_link到arm_link的变换矩阵"""
        try:
            # 计算变换矩阵
            camera_to_arm = self.calculate_transform_matrix(marker_pose)
            if camera_to_arm is None:
                return
            
            # 保存当前变换矩阵
            self.current_tf_matrix = camera_to_arm
            
            # 将4x4矩阵展平为一维数组
            tf_matrix_msg = Float64MultiArray()
            tf_matrix_msg.data = camera_to_arm.flatten().tolist()
            
            # 发布变换矩阵
            self.tf_matrix_pub.publish(tf_matrix_msg)
            
            # 发布TF变换
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = "arm_link"
            transform.child_frame_id = "camera_color_optical_frame"
            
            # 从变换矩阵中提取位置和旋转
            transform.transform.translation.x = float(camera_to_arm[0, 3])
            transform.transform.translation.y = float(camera_to_arm[1, 3])
            transform.transform.translation.z = float(camera_to_arm[2, 3])
            
            # 从旋转矩阵计算四元数
            rotation_matrix = camera_to_arm[:3, :3]
            qw = np.sqrt(1 + rotation_matrix[0,0] + rotation_matrix[1,1] + rotation_matrix[2,2]) / 2
            qx = (rotation_matrix[2,1] - rotation_matrix[1,2]) / (4 * qw)
            qy = (rotation_matrix[0,2] - rotation_matrix[2,0]) / (4 * qw)
            qz = (rotation_matrix[1,0] - rotation_matrix[0,1]) / (4 * qw)
            
            transform.transform.rotation.x = float(qx)
            transform.transform.rotation.y = float(qy)
            transform.transform.rotation.z = float(qz)
            transform.transform.rotation.w = float(qw)
            
            self.tf_broadcaster.sendTransform(transform)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing transform matrix: {str(e)}')
    
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
            self.process_frame()
        except Exception as e:
            self.get_logger().error(f'Error processing color image: {str(e)}')
    
    def process_frame(self):
        """Process image frame and detect ArUco markers"""
        if self.color_image is None or self.camera_matrix is None:
            return
            
        detection_image = self.color_image.copy()
        
        # Detect ArUco markers
        corners, ids, rejected = self.detector.detectMarkers(detection_image)
        
        if ids is not None:
            self.marker_detected = True
            # Draw detected markers
            cv2.aruco.drawDetectedMarkers(detection_image, corners, ids)
            
            # Process each detected marker
            for i, corner in enumerate(corners):
                try:
                    # Reshape corners to match solvePnP input format
                    corners_reshape = corner.reshape(4, 1, 2)
                    
                    # Estimate pose using solvePnP
                    retval, rvec, tvec = cv2.solvePnP(
                        self.objPoints, corners_reshape, 
                        self.camera_matrix, self.dist_coeffs)
                    
                    # Draw axis
                    cv2.drawFrameAxes(detection_image, self.camera_matrix, 
                                    self.dist_coeffs, rvec, tvec, 0.1)
                    
                    # 计算欧拉角
                    rotation_matrix, _ = cv2.Rodrigues(rvec)
                    sy = math.sqrt(rotation_matrix[0,0] * rotation_matrix[0,0] +  rotation_matrix[1,0] * rotation_matrix[1,0])
                    singular = sy < 1e-6
                    if not singular:
                        roll = math.atan2(rotation_matrix[2,1], rotation_matrix[2,2])
                        pitch = math.atan2(-rotation_matrix[2,0], sy)
                        yaw = math.atan2(rotation_matrix[1,0], rotation_matrix[0,0])
                    else:
                        roll = math.atan2(-rotation_matrix[1,2], rotation_matrix[1,1])
                        pitch = math.atan2(-rotation_matrix[2,0], sy)
                        yaw = 0
                    
                    # 创建marker位姿数组
                    marker_pose = [float(tvec[0]), float(tvec[1]), float(tvec[2]), roll, pitch, yaw]
                    
                    # Publish marker pose
                    self.publish_marker_pose(rvec, tvec, ids[i][0])
                    
                    # Publish marker visualization
                    self.publish_marker_visualization(tvec, ids[i][0])
                    
                    # 发布RPY
                    rpy_msg = Float64MultiArray()
                    rpy_msg.data = marker_pose
                    self.marker_pose_rpy_pub.publish(rpy_msg)
                    
                    # 计算camera相对于marker的位置
                    self.calculate_camera_pose(rvec, tvec, ids[i][0])
                    
                    # 发布从camera_link到arm_link的变换矩阵
                    self.publish_transform_matrix(ids[i][0], marker_pose)
                    
                except Exception as e:
                    self.get_logger().error(f'Error processing marker {i}: {str(e)}')
        else:
            self.marker_detected = False
        
        # Show detection result
        cv2.imshow('ArUco Detection', detection_image)
        cv2.waitKey(1)
    
    def calculate_camera_pose(self, rvec, tvec, marker_id):
        """计算camera相对于marker的位置"""
        try:
            # 创建marker到camera的变换矩阵
            marker_to_camera = np.eye(4)
            rotation_matrix, _ = cv2.Rodrigues(rvec)
            marker_to_camera[:3, :3] = rotation_matrix
            marker_to_camera[:3, 3] = tvec.reshape(3)
            
            # 计算camera到marker的变换矩阵（逆变换）
            camera_to_marker = np.linalg.inv(marker_to_camera)
            
            # 提取位置和旋转
            camera_position = camera_to_marker[:3, 3]
            camera_rotation = camera_to_marker[:3, :3]
            
            # 计算欧拉角
            sy = math.sqrt(camera_rotation[0,0] * camera_rotation[0,0] + camera_rotation[1,0] * camera_rotation[1,0])
            singular = sy < 1e-6
            if not singular:
                roll = math.atan2(camera_rotation[2,1], camera_rotation[2,2])
                pitch = math.atan2(-camera_rotation[2,0], sy)
                yaw = math.atan2(camera_rotation[1,0], camera_rotation[0,0])
            else:
                roll = math.atan2(-camera_rotation[1,2], camera_rotation[1,1])
                pitch = math.atan2(-camera_rotation[2,0], sy)
                yaw = 0
            
            # 发布camera位姿
            camera_pose_msg = PoseStamped()
            camera_pose_msg.header.stamp = self.get_clock().now().to_msg()
            camera_pose_msg.header.frame_id = f"aruco_marker_{marker_id}"
            camera_pose_msg.pose.position.x = float(camera_position[0])
            camera_pose_msg.pose.position.y = float(camera_position[1])
            camera_pose_msg.pose.position.z = float(camera_position[2])
            
            # 转换旋转矩阵到四元数
            qw = np.sqrt(1 + camera_rotation[0,0] + camera_rotation[1,1] + camera_rotation[2,2]) / 2
            qx = (camera_rotation[2,1] - camera_rotation[1,2]) / (4 * qw)
            qy = (camera_rotation[0,2] - camera_rotation[2,0]) / (4 * qw)
            qz = (camera_rotation[1,0] - camera_rotation[0,1]) / (4 * qw)
            
            camera_pose_msg.pose.orientation.x = float(qx)
            camera_pose_msg.pose.orientation.y = float(qy)
            camera_pose_msg.pose.orientation.z = float(qz)
            camera_pose_msg.pose.orientation.w = float(qw)
            
            self.camera_pose_pub.publish(camera_pose_msg)
            
            # 发布RPY格式的camera位姿
            camera_rpy_msg = Float64MultiArray()
            camera_rpy_msg.data = [float(camera_position[0]), float(camera_position[1]), float(camera_position[2]), 
                                 roll, pitch, yaw]
            self.camera_pose_rpy_pub.publish(camera_rpy_msg)
            
            # 发布TF变换
            transform = TransformStamped()
            transform.header = camera_pose_msg.header
            transform.child_frame_id = "camera_color_optical_frame"
            transform.transform.translation.x = camera_pose_msg.pose.position.x
            transform.transform.translation.y = camera_pose_msg.pose.position.y
            transform.transform.translation.z = camera_pose_msg.pose.position.z
            transform.transform.rotation = camera_pose_msg.pose.orientation
            self.tf_broadcaster.sendTransform(transform)
            
        except Exception as e:
            self.get_logger().error(f'Error calculating camera pose: {str(e)}')
    
    def publish_marker_pose(self, rvec, tvec, marker_id):
        """Publish marker pose as PoseStamped message"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "camera_color_optical_frame"
        
        # Set position
        pose_msg.pose.position.x = float(tvec[0])
        pose_msg.pose.position.y = float(tvec[1])
        pose_msg.pose.position.z = float(tvec[2])
        
        # Convert rotation vector to quaternion
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        qw = np.sqrt(1 + rotation_matrix[0, 0] + rotation_matrix[1, 1] + rotation_matrix[2, 2]) / 2
        qx = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / (4 * qw)
        qy = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / (4 * qw)
        qz = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / (4 * qw)
        
        # Set orientation
        pose_msg.pose.orientation.x = float(qx)
        pose_msg.pose.orientation.y = float(qy)
        pose_msg.pose.orientation.z = float(qz)
        pose_msg.pose.orientation.w = float(qw)
        
        self.pose_pub.publish(pose_msg)
        
        # Broadcast transform
        transform = TransformStamped()
        transform.header = pose_msg.header
        transform.child_frame_id = f"aruco_marker_{marker_id}"
        transform.transform.translation.x = pose_msg.pose.position.x
        transform.transform.translation.y = pose_msg.pose.position.y
        transform.transform.translation.z = pose_msg.pose.position.z
        transform.transform.rotation = pose_msg.pose.orientation
        self.tf_broadcaster.sendTransform(transform)
    
    def publish_marker_visualization(self, tvec, marker_id):
        """Publish marker visualization as Marker message"""
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "camera_color_optical_frame"
        marker.ns = "aruco_markers"
        marker.id = int(marker_id)
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Set marker pose
        marker.pose.position.x = float(tvec[0])
        marker.pose.position.y = float(tvec[1])
        marker.pose.position.z = float(tvec[2])
        marker.pose.orientation.w = 1.0
        
        # Set marker scale
        marker.scale.x = self.marker_size
        marker.scale.y = self.marker_size
        marker.scale.z = 0.01
        
        # Set marker color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        
        self.marker_pub.publish(marker)

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