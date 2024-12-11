import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Quaternion
from std_srvs.srv import Empty
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import TransformStamped
import tf2_ros
from apex_putter.MotionPlanningInterface import MotionPlanningInterface
import apex_putter.transform_operations as transOps
from time import sleep

from tf_transformations import quaternion_from_euler


class DemoNode(Node):
    def __init__(self):
        super().__init__('demo_node')

        # Parameter to toggle simulation vs real-world
        self.declare_parameter('simulation_mode', True)
        self.use_simulation_mode = self.get_parameter(
            'simulation_mode').get_parameter_value().bool_value

        # Declare parameters for frames
        self.declare_parameter('ball_tag_frame', 'ball_compensated')
        self.declare_parameter('hole_tag_frame', 'target')
        self.declare_parameter('base_frame', 'robot_base_frame')
        self.declare_parameter('camera_frame', 'camera_link')

        self.ball_tag_frame = self.get_parameter(
            'ball_tag_frame').get_parameter_value().string_value
        self.hole_tag_frame = self.get_parameter(
            'hole_tag_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter(
            'base_frame').get_parameter_value().string_value
        self.camera_frame = self.get_parameter(
            'camera_frame').get_parameter_value().string_value

        # Known offsets:
        self.putter_length = 22.85 * 0.0254  # About 0.58 m
        self.putter_offset = 0.18 * 0.0254  # About 0.00457 m

        # Motion planning interface
        self.MPI = MotionPlanningInterface(
            node=self,
            base_frame=self.base_frame,
            end_effector_frame='fer_link8'
        )

        self.hole_position = None
        self.ball_position = None
        self.v_h2b = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Static broadcaster for ball and club face
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Services
        self.ready_srv = self.create_service(Empty, 'ready', self.ready_callback, callback_group=MutuallyExclusiveCallbackGroup())
        self.home_srv = self.create_service(Empty, 'home_robot', self.home_callback, callback_group=MutuallyExclusiveCallbackGroup())
        self.putt_srv = self.create_service(Empty, 'putt', self.putt_callback, callback_group=MutuallyExclusiveCallbackGroup())
        self.swing_srv = self.create_service(Empty, 'swing', self.swing_callback, callback_group=MutuallyExclusiveCallbackGroup())

        # Timer for optional tasks
        # self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("DemoNode initialized. Use '/simulate' or '/real_putt'.")

    async def home_callback(self, request, response):
        """Make the robot go to home pose"""
        self.get_logger().info("Home requested.")
        await self.MPI.move_arm_joints(joint_values=[-0.4, -0.4, 0.0, -1.6, 0.0, 1.57, 0.0], max_velocity_scaling_factor=0.2, max_acceleration_scaling_factor=0.2)
        self.v_h2b = self.calculate_hole_to_ball_vector()
        self.goal_club_tf()
        self.goal_ee_tf()
        return response

    def look_up_ball_in_base_frame(self):
        """Look up the ball position in the base frame"""
        self.get_logger().info("Looking up ball position in base frame.")
        try:
            transform_base_ball = self.tf_buffer.lookup_transform(
                self.base_frame, self.ball_tag_frame, rclpy.time.Time())
            self.get_logger().info(f"Transform from {self.ball_tag_frame} to {
                self.base_frame}: {transform_base_ball}")
            htm_base_ball = transOps.transform_to_htm(
                transform_base_ball.transform)
            self.ball_position = htm_base_ball[:3, 3]
            self.get_logger().info(f"Ball position: {self.ball_position}")
        except Exception as e:
            self.get_logger().error(f"Failed to look up ball position: {e}")

    def look_up_hole_in_base_frame(self):
        """Look up the hole position in the base frame"""
        self.get_logger().info("Looking up hole position in base frame.")
        try:
            transform_base_hole = self.tf_buffer.lookup_transform(
                self.base_frame, self.hole_tag_frame, rclpy.time.Time())
            self.get_logger().info(f"Transform from {self.hole_tag_frame} to {
                self.base_frame}: {transform_base_hole}")
            htm_base_hole = transOps.transform_to_htm(
                transform_base_hole.transform)
            self.hole_position = htm_base_hole[:3, 3]
            self.get_logger().info(f"Hole position: {self.hole_position}")
        except Exception as e:
            self.get_logger().error(f"Failed to look up hole position: {e}")

    def calculate_hole_to_ball_vector(self):
        """Calculate the vector from the hole to the ball"""
        self.look_up_ball_in_base_frame()
        self.look_up_hole_in_base_frame()
        # flatten the hole position on z-axis
        self.hole_position[2] = self.ball_position[2]
        return self.ball_position - self.hole_position  # vector: hole to ball

    def goal_club_tf(self):
        radius = 0.045
        ball_hole_vec = -self.calculate_hole_to_ball_vector()
        theta_hole_ball = np.arctan2(ball_hole_vec[1], ball_hole_vec[0])
        ball_hole_mag = np.linalg.norm(ball_hole_vec)
        ball_hole_unit = ball_hole_vec / ball_hole_mag
        club_face_position = -radius * ball_hole_unit
        club_face_orientation = quaternion_from_euler(
            0.0, 0.0, theta_hole_ball)
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base'
        t.child_frame_id = 'goal_face'

        t.transform.translation.x = self.ball_position[0] + 0.08 * self.v_h2b[0]
        t.transform.translation.y = self.ball_position[1] + 0.08 * self.v_h2b[1]
        t.transform.translation.z = self.ball_position[2]
        t.transform.rotation.x = club_face_orientation[0]
        t.transform.rotation.y = club_face_orientation[1]
        t.transform.rotation.z = club_face_orientation[2]
        t.transform.rotation.w = club_face_orientation[3]

        self.tf_static_broadcaster.sendTransform(t)

    def goal_ee_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'goal_face'
        t.child_frame_id = 'goal_ee'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.54

        dummy_orientation = quaternion_from_euler(np.pi, 0.0, 0.0)
        t.transform.rotation.x = dummy_orientation[0]
        t.transform.rotation.y = dummy_orientation[1]
        t.transform.rotation.z = dummy_orientation[2]
        t.transform.rotation.w = dummy_orientation[3]

        self.tf_static_broadcaster.sendTransform(t)

    async def ready_callback(self, request, response):
        """Prepare the robot for putting"""
        self.get_logger().info("Ready requested.")
        self.get_logger().info("=============================================================")

        # Look up the ideal ee transform first
        ideal_ee_transform = self.tf_buffer.lookup_transform(
            self.base_frame, 'goal_ee', rclpy.time.Time())
        ideal_pose = Pose()
        ideal_pose.position.x = ideal_ee_transform.transform.translation.x
        ideal_pose.position.y = ideal_ee_transform.transform.translation.y
        ideal_pose.position.z = ideal_ee_transform.transform.translation.z
        ideal_pose.orientation = ideal_ee_transform.transform.rotation
        ball_tf = await self.MPI.get_transform('base', 'ball')
        # await self.MPI.add_box('ball', (0.042, 0.042, 0.042), (ball_tf.pose.position.x, ball_tf.pose.position.y, ball_tf.pose.position.z))
        await self.MPI.move_arm_pose(ideal_pose, max_velocity_scaling_factor=0.2, max_acceleration_scaling_factor=0.2)
        # await self.MPI.remove_box('ball')
        return response

    def calculate_putt_strength(self):
        scaling_factor = 0.5
        distance = np.linalg.norm(self.v_h2b)
        output = scaling_factor * distance + 0.2
        if output > 0.8:
            output = 0.8
        return output


    async def putt_callback(self, request, response):
        """Putt the fucking ball"""
        self.get_logger().info("Putt requested.")
        self.get_logger().info("=============================================================")
        ideal_ee_transform = self.tf_buffer.lookup_transform(self.base_frame, 'goal_ee', rclpy.time.Time())
        ideal_pose = Pose()
        ideal_pose.position.x = ideal_ee_transform.transform.translation.x - 0.2 * self.v_h2b[0]
        ideal_pose.position.y = ideal_ee_transform.transform.translation.y - 0.2 * self.v_h2b[1]
        ideal_pose.position.z = ideal_ee_transform.transform.translation.z
        ideal_pose.orientation = ideal_ee_transform.transform.rotation

        traj_vec = self.v_h2b
        traj_mag = np.linalg.norm(traj_vec)
        traj_unit = traj_vec / traj_mag 

        def contruct_putt_pose(vector, ideal_pose, scaling):
            pose = Pose()
            pose.position.x = ideal_pose.position.x - scaling * vector[0]
            pose.position.y = ideal_pose.position.y - scaling * vector[1]
            pose.position.z = ideal_pose.position.z
            pose.orientation = ideal_pose.orientation
            return pose
        
        putt_pose_2 = contruct_putt_pose(traj_unit, ideal_pose, 0.11)
        strength = self.calculate_putt_strength()

        self.get_logger().info(f"=====================Putt strength: {strength}=====================")    

        await self.MPI.move_arm_pose(putt_pose_2, max_velocity_scaling_factor=strength, max_acceleration_scaling_factor=strength * 0.8)
        return response
    
    async def swing_callback(self, request, response):
        """Swing 'em!!"""
        self.get_logger().info("Swing requested.")
        self.get_logger().info("=============================================================")

        # Look up the current joint configuration
        current_robot_state = self.MPI.RobotState.get_robot_state()
        current_joint_values = current_robot_state.joint_state.position

        swung_joint_values = current_joint_values
        swung_joint_values[4] = swung_joint_values[4] + np.pi/6

        # Swing the putter
        await self.MPI.move_arm_joints(joint_values=swung_joint_values, max_velocity_scaling_factor=0.6, max_acceleration_scaling_factor=0.6)
        return response

    def offset_ball_position(self, z):
        """Offset the ball position by z"""
        self.ball_position[2] += z


def main(args=None):
    rclpy.init(args=args)
    node = DemoNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
