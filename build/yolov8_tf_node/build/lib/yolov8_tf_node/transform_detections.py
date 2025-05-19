#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from yolo_msgs.msg import DetectionArray

class TFTransformer(Node):
    def __init__(self):
        super().__init__('yolo_tf_transformer')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            DetectionArray,
            '/yolo/detections_3d',
            self.callback,
            10
        )

    def callback(self, msg):
        for detection in msg.detections:
            pt = PointStamped()
            pt.header.frame_id = msg.header.frame_id  # 通常为 camera_link
            pt.header.stamp = self.get_clock().now().to_msg()
            pt.point = detection.bbox3d.center.position  # 关键改动

            try:
                transformed = self.tf_buffer.transform(pt, 'map', timeout=rclpy.duration.Duration(seconds=1.0))
                self.get_logger().info(
                    f"{detection.class_name} in map: "
                    f"({transformed.point.x:.2f}, {transformed.point.y:.2f}, {transformed.point.z:.2f})"
                )
            except Exception as e:
                self.get_logger().warn(f"TF transform failed: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = TFTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
