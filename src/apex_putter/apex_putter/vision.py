"""
Vision module for Apex Putter. This nodes is responsible for detecting the golf ball and publishes transformation for robot base.
Author: Zhengyang Kris Weng
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge  
import tf2_ros
import tf_transformations
from geometry_msgs.msg import Transform, TransformStamped
import apex_putter.transform_operations as transOps
from apex_putter_interfaces.msg import Detection2D, DetectionArray


class Vision(Node):
    def __init__(self):
        super().__init__('vision')

        self.bridge = CvBridge()

        # Known transform from apriltag to robot base
        self.atag_to_rbf_matrix = np.array([
            [0, 1, 0, 0.180],
            [0, 0, 1, -0.085],
            [1, 0, 0, -0.009],
            [0, 0, 0, 1]
        ])

        self.atag_to_rbf_transform = transOps.htm_to_transform(self.atag_to_rbf_matrix)

        # TF listener
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.rbf_publisher = self.create_publisher(
            TransformStamped,
            '/robot_base_frame',
            10
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.ball_detection_sub = self.create_subscription(
            DetectionArray,
            '/ball_detections',
            self.ball_detection_callback,
            10
        )

        self.ball_xy_image = None

        self.timer = self.create_timer(1, self.timer_callback)

        self.get_logger().info('Vision node started')
    
    def publish_rbf(self):
        """Publish robot base frame"""
        robot_base_transform = TransformStamped()
        robot_base_transform.header.stamp = self.get_clock().now().to_msg()
        # robot_base_transform.header.frame_id = 'tag36h11:9'
        robot_base_transform.header.frame_id = 'robot_base_tag'
        robot_base_transform.child_frame_id = 'robot_base_frame'
        robot_base_transform.transform = self.atag_to_rbf_transform

        self.rbf_publisher.publish(robot_base_transform)
        self.tf_broadcaster.sendTransform(robot_base_transform)

    def ball_detection_callback(self, msg):
        """Callback for ball detection"""
        # Initialize empty array with 2 columns for x,y coordinates
        balls_detected = np.empty((0, 2))
        
        for detection in msg.detections:  # Note: using msg.points based on your earlier message definition
            self.get_logger().debug(f"Detected ball at ({detection.x}, {detection.y})")
            # Create a 1x2 array for the current detection
            ball_detected = np.array([[detection.x, detection.y]])
            # Append as a new row
            balls_detected = np.vstack((balls_detected, ball_detected))
        
        self.ball_xy_image = balls_detected
        self.get_logger().info(f"Ball detected at {self.ball_xy_image}")

    def timer_callback(self):
        self.publish_rbf()

def main(args=None):
    rclpy.init(args=args)

    vision = Vision()

    rclpy.spin(vision)

    vision.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()