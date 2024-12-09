import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Quaternion, TransformStamped, Point
from std_srvs.srv import Empty
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import math
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker
import time

from apex_putter.MotionPlanningInterface import MotionPlanningInterface
import apex_putter.transform_operations as transOps

from tf_transformations import quaternion_from_euler

class DemoNode(Node):
    def __init__(self):
        super().__init__('demo_node')

        # Parameter to toggle simulation vs real-world
        self.declare_parameter('simulation_mode', True)
        self.use_simulation_mode = self.get_parameter('simulation_mode').get_parameter_value().bool_value

        # Declare parameters for frames
        self.declare_parameter('ball_tag_frame', 'ball')
        self.declare_parameter('hole_tag_frame', 'tag_15')
        self.declare_parameter('base_frame', 'robot_base_frame')
        self.declare_parameter('camera_frame', 'camera_link')

        self.ball_tag_frame = self.get_parameter('ball_tag_frame').get_parameter_value().string_value
        self.hole_tag_frame = self.get_parameter('hole_tag_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value

        # Known offsets:
        self.putter_length = 22.85 * 0.0254  # About 0.58 m
        self.putter_offset = 0.18 * 0.0254   # About 0.00457 m

        # Dimensions (in meters)
        self.puttface_dim = [0.1,0.02,0.028]
        self.goal_ball_radius = 2.03
        self.putface_ee_transform =  np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, -0.53],
            [0, 0, 0, 1]
        ])

        # Motion planning interface
        self.MPI = MotionPlanningInterface(
            node=self,
            base_frame=self.base_frame,
            end_effector_frame='fer_link8'
        )

        # Default fallback positions if TF not available yet
        self.hole_position = np.array([0.72347078, 0.29201838, 0.05362293])
        self.ball_position = np.array([0.60659744, -0.04259332, 0.05297366])

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Dynamic transform broadcaster for putter-face
        self.tf_dynamic_broadcaster = TransformBroadcaster(self)
        self.transform_base_ee = TransformStamped()
        self.transform_base_ee.header.stamp = self.get_clock().now().to_msg()

        # Publishers for markers
        self.ball_marker_pub = self.create_publisher(Marker, 'ball_marker', 10)
        self.hole_marker_pub = self.create_publisher(Marker, 'hole_marker', 10)
        self.vector_marker_pub = self.create_publisher(Marker, 'vector_marker', 10)

        # Create markers
        self.ball_marker = Marker()
        self.hole_marker = Marker()
        self.vector_marker = Marker()

        # Setup markers with default properties
        self.setup_ball_marker()
        self.setup_hole_marker()
        self.setup_vector_marker()

        # Republish markers at 1 Hz
        self.marker_timer = self.create_timer(1.0, self.publish_markers)

        # Services
        self.ready_srv = self.create_service(
            Empty, 'ready', self.ready_callback, callback_group=MutuallyExclusiveCallbackGroup())
        self.home_srv = self.create_service(
            Empty, 'home_robot', self.home_callback, callback_group=MutuallyExclusiveCallbackGroup())
        self.putt_srv = self.create_service(
            Empty, 'putt', self.putt_callback, callback_group=MutuallyExclusiveCallbackGroup())

        # Timer for optional tasks
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("DemoNode initialized. Use '/ready' to set up the putter and '/putt' to execute the putt.")

    async def home_callback(self, request, response):
        """Make the robot go to home pose"""
        self.get_logger().info("Home requested.")
        await self.MPI.move_arm_joints([0.0, -0.4, 0.0, -1.6, 0.0, 1.57, 0.0])
        return response

    def dynamic_brodcaster(self,transform_base_ee: TransformStamped):
        T_be = transOps.transform_to_htm(transform_base_ee.transform)
        T_ep = self.putface_ee_transform
        T_pb = np.dot(np.linalg.inv(T_ep),np.linalg.inv(T_be))
        T = transOps.htm_to_transform(T_pb)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'fer_link8'
        t.child_frame_id = 'putt_face'
        t.transform = T
        self.tf_dynamic_broadcaster.sendTransform(t)

    def look_up_fer8_frame(self):
        """Look up the ee position in the base frame"""
        self.get_logger().info("Looking up ee position in base frame.")
        try:
            self.transform_base_ee = self.tf_buffer.lookup_transform(self.base_frame, 'fer_link8', rclpy.time.Time())
            self.get_logger().info(f"Transform from 'fer_link8' to {self.base_frame}: {self.transform_base_ee}")
        except Exception as e:
            self.get_logger().error(f"Failed to look up the end-effector transform: {e}")

    def look_up_ball_in_base_frame(self):
        """Look up the ball position in the base frame"""
        self.get_logger().info("Looking up ball position in base frame.")
        try:
            transform_base_ball = self.tf_buffer.lookup_transform(self.base_frame, self.ball_tag_frame, rclpy.time.Time())
            htm_base_ball = transOps.transform_to_htm(transform_base_ball.transform)
            self.ball_position = htm_base_ball[:3, 3]
            self.get_logger().info(f"Ball position: {self.ball_position}")
        except Exception as e:
            self.ball_position = np.array([0.60659744, -0.04259332, 0.05297366])
            self.get_logger().error(f"Failed to look up ball position, using fallback: {self.ball_position}")

    def look_up_hole_in_base_frame(self):
        """Look up the hole position in the base frame"""
        self.get_logger().info("Looking up hole position in base frame.")
        try:
            transform_base_hole = self.tf_buffer.lookup_transform(self.base_frame, self.hole_tag_frame, rclpy.time.Time())
            htm_base_hole = transOps.transform_to_htm(transform_base_hole.transform)
            self.hole_position = htm_base_hole[:3, 3]
            self.get_logger().info(f"Hole position: {self.hole_position}")
        except Exception as e:
            self.hole_position = np.array([0.72347078, 0.29201838, 0.05362293])
            self.get_logger().error(f"Failed to look up hole position, using fallback: {self.hole_position}")

    def calculate_ball_and_hole_positions(self):
        self.look_up_ball_in_base_frame()
        self.look_up_hole_in_base_frame()

        # Update ball and hole markers with the latest positions
        self.ball_marker.pose.position.x = float(self.ball_position[0])
        self.ball_marker.pose.position.y = float(self.ball_position[1])
        self.ball_marker.pose.position.z = float(self.ball_position[2])

        self.hole_marker.pose.position.x = float(self.hole_position[0])
        self.hole_marker.pose.position.y = float(self.hole_position[1])
        self.hole_marker.pose.position.z = float(self.hole_position[2])

    def setup_ball_marker(self):
        self.ball_marker.ns = "ball_marker_ns"
        self.ball_marker.id = 0
        self.ball_marker.header.frame_id = self.base_frame
        self.ball_marker.type = Marker.SPHERE
        self.ball_marker.action = Marker.ADD
        self.ball_marker.scale.x = 0.03
        self.ball_marker.scale.y = 0.03
        self.ball_marker.scale.z = 0.03
        self.ball_marker.color.r = 1.0
        self.ball_marker.color.g = 1.0
        self.ball_marker.color.b = 1.0
        self.ball_marker.color.a = 1.0
        self.ball_marker.pose.orientation.w = 1.0
        # Use fallback positions initially
        self.ball_marker.pose.position.x = float(self.ball_position[0])
        self.ball_marker.pose.position.y = float(self.ball_position[1])
        self.ball_marker.pose.position.z = float(self.ball_position[2])

    def setup_hole_marker(self):
        self.hole_marker.ns = "hole_marker_ns"
        self.hole_marker.id = 0
        self.hole_marker.header.frame_id = self.base_frame
        self.hole_marker.type = Marker.CYLINDER
        self.hole_marker.action = Marker.ADD
        self.hole_marker.scale.x = 0.1
        self.hole_marker.scale.y = 0.1
        self.hole_marker.scale.z = 0.01
        self.hole_marker.color.r = 0.0
        self.hole_marker.color.g = 0.0
        self.hole_marker.color.b = 0.0
        self.hole_marker.color.a = 1.0
        self.hole_marker.pose.orientation.w = 1.0
        # Use fallback positions initially
        self.hole_marker.pose.position.x = float(self.hole_position[0])
        self.hole_marker.pose.position.y = float(self.hole_position[1])
        self.hole_marker.pose.position.z = float(self.hole_position[2])

    def setup_vector_marker(self):
        # We'll use an ARROW marker
        self.vector_marker.ns = "vector_marker_ns"
        self.vector_marker.id = 0
        self.vector_marker.header.frame_id = self.base_frame
        self.vector_marker.type = Marker.ARROW
        self.vector_marker.action = Marker.ADD
        self.vector_marker.scale.x = 0.01  # shaft diameter
        self.vector_marker.scale.y = 0.02  # head diameter
        self.vector_marker.scale.z = 0.0

        self.vector_marker.color.r = 0.0
        self.vector_marker.color.g = 1.0
        self.vector_marker.color.b = 0.0
        self.vector_marker.color.a = 1.0
        # Points will be updated in publish_markers()

    def publish_markers(self):
        now = self.get_clock().now().to_msg()
        self.ball_marker.header.stamp = now
        self.hole_marker.header.stamp = now
        self.vector_marker.header.stamp = now

        # Update vector marker:
        # Look up putt_face in base_frame
        try:
            putt_face_tf = self.tf_buffer.lookup_transform(self.base_frame, 'putt_face', rclpy.time.Time())
            putt_face_pos = np.array([
                putt_face_tf.transform.translation.x,
                putt_face_tf.transform.translation.y,
                putt_face_tf.transform.translation.z
            ])
        except Exception:
            # If not available, fallback to ball position
            self.get_logger().warn("Could not look up putt_face transform, using ball position for vector start.")
            putt_face_pos = self.ball_position

        start_point = Point(x=float(putt_face_pos[0]),
                            y=float(putt_face_pos[1]),
                            z=float(putt_face_pos[2]))
        end_point = Point(x=float(self.hole_position[0]),
                          y=float(self.hole_position[1]),
                          z=float(self.hole_position[2]))

        self.vector_marker.points = [start_point, end_point]

        self.ball_marker_pub.publish(self.ball_marker)
        self.hole_marker_pub.publish(self.hole_marker)
        self.vector_marker_pub.publish(self.vector_marker)

    async def ready_callback(self, request, response):
        self.get_logger().info("Ready requested.")
        self.calculate_ball_and_hole_positions()

        direction = self.hole_position - self.ball_position
        direction[2] = 0.0
        dist = np.linalg.norm(direction)
        if dist < 1e-6:
            self.get_logger().error("Ball and hole overlap, cannot set orientation.")
            return response
        direction = direction / dist

        # Yaw from direction
        yaw = math.atan2(direction[1], direction[0])
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
        orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        # Place fer_link8 behind the ball by 0.1m at ball height minus 0.53
        ready_pos = self.ball_position - 0.1 * direction
        ready_pos[2] = self.ball_position[2] - 0.53

        ready_pose = Pose()
        ready_pose.position.x = ready_pos[0]
        ready_pose.position.y = ready_pos[1]
        ready_pose.position.z = ready_pos[2]
        ready_pose.orientation = orientation

        self.get_logger().info(f"Ready pose: pos={ready_pose.position}, ori={ready_pose.orientation}")

        time.sleep(2)
        result = await self.MPI.move_arm_pose(ready_pose, max_velocity_scaling_factor=0.5, max_acceleration_scaling_factor=0.5)
        if result:
            self.get_logger().info("Moved to ready pose.")
        else:
            self.get_logger().error("Failed to move to ready pose.")
        return response

    async def putt_callback(self, request, response):
        self.get_logger().info("Putt requested.")
        self.calculate_ball_and_hole_positions()

        direction = self.hole_position - self.ball_position
        direction[2] = 0.0
        dist = np.linalg.norm(direction)
        if dist < 1e-6:
            self.get_logger().error("Ball and hole overlap, cannot putt.")
            return response
        direction = direction / dist

        yaw = math.atan2(direction[1], direction[0])
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
        orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        # Start 0.2m behind the ball line
        putt_start_pos = self.ball_position - 0.2 * direction
        putt_start_pos[2] = self.ball_position[2] - 0.53

        # End 0.2m ahead of the ball (total 0.4m movement)
        putt_end_pos = self.ball_position + 0.2 * direction
        putt_end_pos[2] = self.ball_position[2] - 0.53

        start_pose = Pose()
        start_pose.position.x = putt_start_pos[0]
        start_pose.position.y = putt_start_pos[1]
        start_pose.position.z = putt_start_pos[2]
        start_pose.orientation = orientation

        end_pose = Pose()
        end_pose.position.x = putt_end_pos[0]
        end_pose.position.y = putt_end_pos[1]
        end_pose.position.z = putt_end_pos[2]
        end_pose.orientation = orientation

        self.get_logger().info(f"Putt start pose: pos={start_pose.position}, ori={start_pose.orientation}")
        self.get_logger().info(f"Putt end pose: pos={end_pose.position}, ori={end_pose.orientation}")

        result = await self.MPI.move_arm_cartesian(
            waypoints=[start_pose, end_pose],
            avoid_collisions=False,
            max_velocity_scaling_factor=0.8,
            max_acceleration_scaling_factor=0.8
        )

        if result:
            self.get_logger().info("Putt action executed successfully.")
        else:
            self.get_logger().error("Putt action failed. Check orientation or constraints.")
        return response

    def timer_callback(self):
        self.dynamic_brodcaster(self.transform_base_ee)

def main(args=None):
    rclpy.init(args=args)
    node = DemoNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
