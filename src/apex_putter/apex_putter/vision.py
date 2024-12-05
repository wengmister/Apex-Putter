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


class Vision(Node):
    def __init__(self):
        super().__init__('vision')

        self.bridge = CvBridge()

        # Known transform from apriltag to robot base
        self.atag_to_rbf_matrix = np.array([
            [0, 1, 0, 0.156],
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

        self.timer = self.create_timer(1, self.timer_callback)

        self.get_logger().info('Vision node started')
    
    def publish_rbf(self):
        """Publish robot base frame"""
        robot_base_transform = TransformStamped()
        robot_base_transform.header.stamp = self.get_clock().now().to_msg()
        robot_base_transform.header.frame_id = 'tag36h11:9'
        robot_base_transform.child_frame_id = 'robot_base_frame'
        robot_base_transform.transform = self.atag_to_rbf_transform

        self.rbf_publisher.publish(robot_base_transform)
        self.tf_broadcaster.sendTransform(robot_base_transform)

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