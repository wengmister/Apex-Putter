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
from transforms3d.quaternions import quat2mat, mat2quat
from transforms3d.affines import compose, decompose

def combine_transforms(known_matrix: np.array, tag_transform: TransformStamped) -> Transform:
    """
    Combines a known transform matrix with a TF2 transform in ROS2
    
    Args:
        known_transform (4x4 np.array): A known homogeneous transformation matrix.
        tf2_transform (TransformStamped): Transform from TF2
        
    Returns:
        Transform: The resulting combined transform
    """

    # Convert TF2 transform to matrix
    tf2_translation = np.array([
        tag_transform.transform.translation.x,
        tag_transform.transform.translation.y,
        tag_transform.transform.translation.z
    ])
    tf2_rotation = quat2mat([
        tag_transform.transform.rotation.w,
        tag_transform.transform.rotation.x,
        tag_transform.transform.rotation.y,
        tag_transform.transform.rotation.z
    ])
    tf2_scale = np.ones(3)
    tf2_matrix = compose(tf2_translation, tf2_rotation, tf2_scale)
    
    # Combine transforms through matrix multiplication
    result_matrix = np.matmul(known_matrix, tf2_matrix)
    
    # Decompose the result
    translation, rotation, scale = decompose(result_matrix)
    quaternion = mat2quat(rotation)  # Returns w,x,y,z
    
    # Create and populate result transform
    result = Transform()
    result.translation.x = float(translation[0])
    result.translation.y = float(translation[1])
    result.translation.z = float(translation[2])
    result.rotation.w = float(quaternion[0])  # transforms3d returns w,x,y,z
    result.rotation.x = float(quaternion[1])
    result.rotation.y = float(quaternion[2])
    result.rotation.z = float(quaternion[3])
    
    return result

class Vision(Node):
    def __init__(self):
        super().__init__('vision')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.bridge = CvBridge()

        # Known transform from apriltag to robot base
        self.atag_to_rbf = np.array([
            [0, 1, 0, 0.156],
            [0, 0, 1, -0.085],
            [1, 0, 0, -0.009],
            [0, 0, 0, 1]
        ])

        # TF listener
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.rbf_publisher = self.create_publisher(
            TransformStamped,
            '/robot_base_frame',
            10
        )

        self.timer = self.create_timer(1, self.timer_callback)

        self.get_logger().info('Vision node started')

    # def image_callback(self, msg):
    #     # Convert ROS image message to OpenCV image
    #     np_arr = np.frombuffer(msg.data, np.uint8)
    #     cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    #     # Detect the golf ball


    def update_robot_base_pos(self):
        """Find robot base position based on transform from apriltag."""
        try:
            robot_atag_transform = self.tf_buffer.lookup_transform(
                'camera_color_optical_frame', 'tag36h11:9', rclpy.time.Time())
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'Failed to get transform: {e}')
            self.x_robot = 0.0
            self.y_robot = 0.0
            return

        robot_base_frame = combine_transforms(self.atag_to_rbf, robot_atag_transform)

        return robot_base_frame
    
    def publish_rbf(self):
        """Publish robot base frame"""
        robot_base_frame = self.update_robot_base_pos()
        robot_base_transform = TransformStamped()
        robot_base_transform.header.stamp = self.get_clock().now().to_msg()
        robot_base_transform.header.frame_id = 'camera_color_optical_frame'
        robot_base_transform.child_frame_id = 'robot_base_frame'
        robot_base_transform.transform = robot_base_frame

        self.rbf_publisher.publish(robot_base_transform)

    def timer_callback(self):
        self.publish_rbf()