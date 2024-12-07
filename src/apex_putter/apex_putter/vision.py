"""
Vision module for Apex Putter. This nodes is responsible for detecting the golf ball and publishes transformation for robot base.
Author: Zhengyang Kris Weng
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import tf_transformations
from geometry_msgs.msg import Transform, TransformStamped
import apex_putter.transform_operations as transOps
from apex_putter_interfaces.msg import Detection2D, DetectionArray


class Vision(Node):
    def __init__(self):
        super().__init__('vision')

        self.bridge = CvBridge()
        
        # RS parameters
        self.intrinsics = None
        self._depth_info_topic = "/camera/camera/color/camera_info"
        self._depth_image_topic = "/camera/camera/aligned_depth_to_color/image_raw"
        self._colored_image_topic = "/camera/camera/color/image_raw"

        self.sub_depth = self.create_subscription(
            Image, self._depth_image_topic, self.imageDepthCallback, 1
        )
        self.sub_info = self.create_subscription(
            CameraInfo, self._depth_info_topic, self.imageDepthInfoCallback, 1
        )
        self.sub1 = self.create_subscription(
            Image, self._colored_image_topic, self.get_latest_frame, 1
        )

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

        self.balls_detected_array = None

        self.timer = self.create_timer(1, self.timer_callback)

        self.get_logger().info('Vision node started')

    def imageDepthCallback(self, data):
        """
        Obtain latest depth image.

        Args:
        ----
            data (Image): Depth image message.

        Returns
        -------
            None

        """

        try:

            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            self._latest_depth_img = cv_image
        except CvBridgeError as e:
            self.get_logger().error("CvBridgeError in imageDepthCallback: {}".format(e))

        except ValueError as e:
            self.get_logger().error("ValueError in imageDepthCallback: {}".format(e))
            return

    
    def publish_rbf(self):
        """Publish robot base frame"""
        robot_base_transform = TransformStamped()
        robot_base_transform.header.stamp = self.get_clock().now().to_msg()
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
        
        self.balls_detected_array = balls_detected
        self.get_logger().info(f"Ball detected at {self.balls_detected_array}")

    def deproject_ball_xy(self):
        positions = self.rsViewer.get_ball_positions()
        self.get_logger().info(f"Ball positions: {positions}")


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