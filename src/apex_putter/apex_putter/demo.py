import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Quaternion
from std_srvs.srv import Empty
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
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
        self.use_simulation_mode = self.get_parameter('simulation_mode').get_parameter_value().bool_value

        # Declare parameters for frames
        self.declare_parameter('ball_tag_frame', 'ball_compensated')
        self.declare_parameter('hole_tag_frame', 'tag_15')
        self.declare_parameter('base_frame', 'robot_base_frame')
        self.declare_parameter('camera_frame', 'camera_link')

        self.ball_tag_frame = self.get_parameter('ball_tag_frame').get_parameter_value().string_value
        self.hole_tag_frame = self.get_parameter('hole_tag_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value

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

        # Create a Marker publisher
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)

        # Initialize ball and hole markers
        self.ball_marker = Marker()
        self.hole_marker = Marker()

        # Create a timer that updates the ball marker at 10 Hz
        self.timer = self.create_timer(0.1, self.update_ball_marker_timer_callback)

        # New attributes for simulation
        self.ball_in_motion = False
        self.sim_start_time = None
        self.initial_ball_velocity = 0.0
        self.direction_to_hole = None
        self.friction_coeff = 0.26
        self.gravity = 9.81

        self.get_logger().info("DemoNode initialized. Use '/simulate' or '/real_putt'.")

    async def home_callback(self, request, response):
        """Make the robot go to home pose"""
        self.get_logger().info("Home requested.")
        await self.MPI.move_arm_joints(joint_values=[-0.4, -0.4, 0.0, -1.6, 0.0, 1.57, 0.0], max_velocity_scaling_factor=0.2, max_acceleration_scaling_factor=0.2)
        self.v_h2b = self.calculate_hole_to_ball_vector()
        self.goal_club_tf()
        self.goal_ee_tf()
        return response
    
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
        if self.ball_position is not None:
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
        if self.hole_position is not None:
            self.hole_marker.pose.position.x = float(self.hole_position[0])
            self.hole_marker.pose.position.y = float(self.hole_position[1])
            self.hole_marker.pose.position.z = float(self.hole_position[2])
    
    def look_up_ball_in_base_frame(self):
        """Look up the ball position in the base frame"""
        self.get_logger().info("Looking up ball position in base frame.")
        try:
            transform_base_ball = self.tf_buffer.lookup_transform(self.base_frame, self.ball_tag_frame, rclpy.time.Time())
            self.get_logger().info(f"Transform from {self.ball_tag_frame} to {self.base_frame}: {transform_base_ball}")
            htm_base_ball = transOps.transform_to_htm(transform_base_ball.transform)
            self.ball_position = htm_base_ball[:3, 3]
            self.get_logger().info(f"Ball position: {self.ball_position}")

            # If we haven't set the initial ball position yet, set it now
            if not hasattr(self, 'ball_position_initial'):
                self.ball_position_initial = np.copy(self.ball_position)

            # Update and publish the ball marker
            self.setup_ball_marker()
            self.ball_marker.header.stamp = self.get_clock().now().to_msg()
            self.marker_pub.publish(self.ball_marker)

        except Exception as e:
            self.get_logger().error(f"Failed to look up ball position: {e}")

    def look_up_hole_in_base_frame(self):
        """Look up the hole position in the base frame"""
        self.get_logger().info("Looking up hole position in base frame.")
        try:
            transform_base_hole = self.tf_buffer.lookup_transform(self.base_frame, self.hole_tag_frame, rclpy.time.Time())
            self.get_logger().info(f"Transform from {self.hole_tag_frame} to {self.base_frame}: {transform_base_hole}")
            htm_base_hole = transOps.transform_to_htm(transform_base_hole.transform)
            self.hole_position = htm_base_hole[:3, 3]
            self.get_logger().info(f"Hole position: {self.hole_position}")

            # Update and publish the hole marker
            self.setup_hole_marker()
            self.hole_marker.header.stamp = self.get_clock().now().to_msg()
            self.marker_pub.publish(self.hole_marker)

        except Exception as e:
            self.get_logger().error(f"Failed to look up hole position: {e}")

    def calculate_hole_to_ball_vector(self):
        """Calculate the vector from the hole to the ball"""
        self.look_up_ball_in_base_frame()
        self.look_up_hole_in_base_frame()
        self.hole_position[2] = self.ball_position[2] # flatten the hole position on z-axis
        return self.ball_position - self.hole_position # vector: hole to ball
    
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
        t.transform.translation.z = 0.58

        dummy_orientation = quaternion_from_euler(np.pi, 0.0, 0.0)
        t.transform.rotation.x = dummy_orientation[0]
        t.transform.rotation.y = dummy_orientation[1]
        t.transform.rotation.z = dummy_orientation[2]
        t.transform.rotation.w = dummy_orientation[3]

        self.tf_static_broadcaster.sendTransform(t)

    def update_ball_marker_timer_callback(self):
        if self.ball_in_motion and self.ball_position is not None:
            current_time = self.get_clock().now()
            elapsed = (current_time.nanoseconds - self.sim_start_time.nanoseconds) * 1e-9

            a = self.friction_coeff * self.gravity  # m/s^2
            v = self.initial_ball_velocity - a * elapsed
            if v <= 0:
                v = 0
                self.ball_in_motion = False
                self.get_logger().info("Ball stopped.")

            if self.ball_in_motion:
                displacement = self.initial_ball_velocity * elapsed - 0.5 * a * (elapsed ** 2)
                new_position = self.ball_position_initial + displacement * self.direction_to_hole
                self.ball_position = new_position

                # Check if ball reached/passed hole
                hole_displacement = self.hole_position - self.ball_position_initial
                dist_to_hole = np.linalg.norm(hole_displacement)
                dist_ball_traveled = np.linalg.norm(self.ball_position - self.ball_position_initial)
                if dist_ball_traveled >= dist_to_hole:
                    self.ball_position = self.hole_position
                    self.ball_in_motion = False
                    self.get_logger().info("Ball reached the hole!")

            # Update marker
            self.setup_ball_marker()
            self.ball_marker.header.stamp = self.get_clock().now().to_msg()
            self.marker_pub.publish(self.ball_marker)
        
    async def ready_callback(self, request, response):
        """Prepare the robot for putting"""
        self.get_logger().info("Ready requested.")
        self.get_logger().info("=============================================================")

        # Look up the ideal ee transform first
        ideal_ee_transform = self.tf_buffer.lookup_transform(self.base_frame, 'goal_ee', rclpy.time.Time())
        ideal_pose = Pose()
        ideal_pose.position.x = ideal_ee_transform.transform.translation.x
        ideal_pose.position.y = ideal_ee_transform.transform.translation.y
        ideal_pose.position.z = ideal_ee_transform.transform.translation.z
        ideal_pose.orientation = ideal_ee_transform.transform.rotation

        await self.MPI.move_arm_pose(ideal_pose, max_velocity_scaling_factor=0.2, max_acceleration_scaling_factor=0.2)
        return response
    
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
        
        # Test to get waypoints for cartesian paths
        start_scale = -0.15 # Negative Scale first / could probably start at 0
        end_scale = 0.11

        num_waypoints = 5
        scaling_waypoints = np.linspace(start_scale, end_scale, num=num_waypoints)

        waypoints = []
        for s in scaling_waypoints:
            w_pose = contruct_putt_pose(traj_unit, ideal_pose, s)
            waypoints.append(w_pose)

        self.get_logger().info(f"Waypoints Planned:{waypoints}")
        self.get_logger().info("Attempting to Putt in Cartesian Path")

        # putt_pose_1 = contruct_putt_pose(traj_unit, ideal_pose, -0.15)

        putt_pose_2 = contruct_putt_pose(traj_unit, ideal_pose, 0.11)

        # self.get_logger().info(f"putt_pose_1.{putt_pose_1}")
        self.get_logger().info(f"putt_pose_2.{putt_pose_2}")

        self.get_logger().info("Moving arm to putt.")

        # await self.MPI.move_arm_pose(putt_pose_1, max_velocity_scaling_factor=0.15, max_acceleration_scaling_factor=0.15)

        # self.get_logger().info("Putt the ball.")
        # sleep(0.8)
        # await self.MPI.move_arm_pose(putt_pose_2, max_velocity_scaling_factor=0.5, max_acceleration_scaling_factor=0.4)

        await self.MPI.move_arm_cartesian(waypoints, max_velocity_scaling_factor=0.2, max_acceleration_scaling_factor=0.2)

        # After robot finishes the putt motion:
        self.get_logger().info("Starting ball simulation.")

        # Ensure we have a fresh initial position for the ball
        self.ball_position_initial = np.copy(self.ball_position)

        # We want the ball to move towards the hole:
        direction_ball_to_hole = self.hole_position - self.ball_position_initial
        self.direction_to_hole = direction_ball_to_hole / np.linalg.norm(direction_ball_to_hole)

        self.initial_ball_velocity = 1.0 
        self.ball_in_motion = True
        self.sim_start_time = self.get_clock().now()

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
