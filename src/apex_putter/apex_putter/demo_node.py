import math
import numpy as np
import rclpy
from rclpy.node import Node
from enum import Enum, auto

from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from visualization_msgs.msg import Marker
from std_srvs.srv import Empty
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import tf2_ros
from tf2_geometry_msgs import PoseStamped as Tf2PoseStamped

from apex_putter.MotionPlanningInterface import MotionPlanningInterface
from apex_putter.BallTrajectoryCalc import BallTrajectoryCalculator
import apex_putter.transform_operations as transOps

from tf_transformations import quaternion_from_euler

class State(Enum):
    START = auto()
    IDLE = auto()
    TASK = auto()

class DemoNode(Node):
    def __init__(self):
        super().__init__('demo_node')

        # Parameter to toggle simulation vs real-world mode
        self.declare_parameter('simulation_mode', True)
        self.use_simulation_mode = self.get_parameter('simulation_mode').get_parameter_value().bool_value

        # Declare parameters for frames used in real world mode
        self.declare_parameter('ball_tag_frame', 'ball')
        self.declare_parameter('hole_tag_frame', 'hole')
        self.declare_parameter('robot_base_tag_frame', 'robot_base_tag')
        self.declare_parameter('base_frame', 'base')
        self.declare_parameter('camera_frame', 'camera_link')

        self.ball_tag_frame = self.get_parameter('ball_tag_frame').get_parameter_value().string_value
        self.hole_tag_frame = self.get_parameter('hole_tag_frame').get_parameter_value().string_value
        self.robot_base_tag_frame = self.get_parameter('robot_base_tag_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value

        # Known offsets:
        self.putter_length = 22.85 * 0.0254  # ~0.58 m
        self.putter_offset = 0.18 * 0.0254   # ~0.00457 m

        # Initialize motion planning interface
        self.MPI = MotionPlanningInterface(
            node=self,
            base_frame=self.base_frame,
            end_effector_frame='fer_link8'
        )

        # Hard-coded hole position (simulation mode)
        self.hole_position = np.array([0.8, 0.0, 0.0])
        # Ball position is updated in real world mode
        self.ball_position = np.array([0.5, 0.0, 0.0])

        self.trajectory_calculator = BallTrajectoryCalculator(self.ball_position, self.hole_position)

        # Start pose
        self.start_pose = Pose()
        self.start_pose.position.x = 0.4
        self.start_pose.position.y = 0.0
        self.start_pose.position.z = 0.4
        self.start_pose.orientation = Quaternion(w=1.0)

        # TF setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Compute initial waypoints
        self.waypoints = self.compute_waypoints()

        # State machine
        self.state = State.START
        self.task_step = 0

        # Markers for ball and hole
        self.ball_marker_pub = self.create_publisher(Marker, 'ball_marker', 10)
        self.hole_marker_pub = self.create_publisher(Marker, 'hole_marker', 10)
        self.ball_marker = Marker()
        self.hole_marker = Marker()
        self.setup_ball_marker()
        self.setup_hole_marker()
        self.ball_animation_timer = None

        # Republish markers at 1 Hz
        self.marker_timer = self.create_timer(1.0, self.publish_markers)

        # Services
        self.sim_srv = self.create_service(Empty, 'simulate', self.sim_callback, callback_group=MutuallyExclusiveCallbackGroup())
        self.real_putt_srv = self.create_service(Empty, 'real_putt', self.real_putt_callback, callback_group=MutuallyExclusiveCallbackGroup())

        # Timer for initialization or background tasks
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("DemoNode initialized. Use '/simulate' or '/real_putt' for sequences.")

    def update_real_world_positions(self):
        """
        Update ball and hole positions by looking up their TF frames relative to base.
        The Vision node provides transforms for ball and hole into the TF tree.
        """
        try:
            # Lookup hole position in base frame
            hole_tf = self.tf_buffer.lookup_transform(self.base_frame, self.hole_tag_frame, rclpy.time.Time())
            self.hole_position = np.array([hole_tf.transform.translation.x,
                                           hole_tf.transform.translation.y,
                                           hole_tf.transform.translation.z])

            # Lookup ball position in base frame
            ball_tf = self.tf_buffer.lookup_transform(self.base_frame, self.ball_tag_frame, rclpy.time.Time())
            ball_offset = np.array([0.0, 0.0, 0.0])  # Adjust if needed
            self.ball_position = np.array([
                ball_tf.transform.translation.x,
                ball_tf.transform.translation.y,
                ball_tf.transform.translation.z
            ]) + ball_offset

            self.get_logger().info(f"Real world positions updated: Ball at {self.ball_position}, Hole at {self.hole_position}")

        except Exception as e:
            self.get_logger().error(f"Could not update real world positions: {e}")

    def apply_putter_offset_in_base(self, p: Pose):
        # In simulation mode, apply offset directly in base frame
        x = p.position.x
        y = p.position.y
        z = p.position.z

        # Adjust the sign as needed. If putter must go down, we subtract.
        z -= self.putter_length
        # Move X by putter_offset (adjust sign if needed)
        x -= self.putter_offset

        p.position.x = x
        p.position.y = y
        p.position.z = z
        return p

    def compute_waypoints(self):
        direction = self.hole_position - self.ball_position
        distance = np.linalg.norm(direction)
        if distance < 1e-6:
            self.get_logger().error("Ball and hole overlap. Using default waypoints.")
            return self.default_waypoints()

        unit_dir = direction / distance
        behind_ball_pos = self.ball_position - unit_dir * 0.05
        at_ball_pos = self.ball_position
        beyond_hole_pos = self.hole_position + unit_dir * 0.05

        # Orientation: If needed, flip end-effector 180 degrees vertically
        roll = math.pi
        pitch = 0.0
        yaw = 0.0
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)

        def make_pose(x, y, z):
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            p.position.z = float(z)
            p.orientation.x = qx
            p.orientation.y = qy
            p.orientation.z = qz
            p.orientation.w = qw
            return p

        z_height = 0.4
        wp1 = make_pose(behind_ball_pos[0], behind_ball_pos[1], z_height)
        wp2 = make_pose(at_ball_pos[0], at_ball_pos[1], z_height)
        wp3 = make_pose(beyond_hole_pos[0], beyond_hole_pos[1], z_height)

        if self.use_simulation_mode:
            # Simulation mode (previously we applied putter offsets in base)
            # Commenting out offset lines since URDF now places fer_link8 at putter head:
            # wp1 = self.apply_putter_offset_in_base(wp1)
            # wp2 = self.apply_putter_offset_in_base(wp2)
            # wp3 = self.apply_putter_offset_in_base(wp3)
            pass
        else:
            # Real world mode (also previously applied offsets)
            # Commenting out offset lines for the same reason:
            # wp1 = self.apply_putter_offset_in_base(wp1)
            # wp2 = self.apply_putter_offset_in_base(wp2)
            # wp3 = self.apply_putter_offset_in_base(wp3)
            pass

        return [wp1, wp2, wp3]


    def default_waypoints(self):
        # don't worry these change!
        w1 = Pose()
        w1.position.x = 0.45
        w1.position.y = 0.0
        w1.position.z = 0.4
        w1.orientation.w = 1.0

        w2 = Pose()
        w2.position.x = 0.5
        w2.position.y = 0.0
        w2.position.z = 0.4
        w2.orientation.w = 1.0

        w3 = Pose()
        w3.position.x = 0.55
        w3.position.y = 0.0
        w3.position.z = 0.4
        w3.orientation.w = 1.0
        return [w1, w2, w3]

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
        self.hole_marker.pose.position.x = float(self.hole_position[0])
        self.hole_marker.pose.position.y = float(self.hole_position[1])
        self.hole_marker.pose.position.z = float(self.hole_position[2])

    def publish_markers(self):
        now = self.get_clock().now().to_msg()
        self.ball_marker.header.stamp = now
        self.hole_marker.header.stamp = now
        self.ball_marker_pub.publish(self.ball_marker)
        self.hole_marker_pub.publish(self.hole_marker)

    async def sim_callback(self, request, response):
        if not self.use_simulation_mode:
            self.get_logger().warn("simulate service called, but simulation_mode is False.")
        self.get_logger().info("Simulation started.")
        self.waypoints = self.compute_waypoints()
        success = await self.run_motion_sequence()
        if success:
            self.get_logger().info("Simulation done.")
        else:
            self.get_logger().error("Simulation failed.")
        return response

    async def real_putt_callback(self, request, response):
        if self.use_simulation_mode:
            self.get_logger().info("Real putt requested, but simulation_mode is True. Switch simulation_mode=False.")
            return response

        self.get_logger().info("Real-world putt requested.")
        # Update positions from TF (no subscribers needed, Vision node publishes TF)
        self.update_real_world_positions()
        self.waypoints = self.compute_waypoints()

        # Before executing the motion, align club face
        # Create Pose objects for ball and hole for alignment function
        ball_pose = Pose()
        ball_pose.position.x = float(self.ball_position[0])
        ball_pose.position.y = float(self.ball_position[1])
        ball_pose.position.z = float(self.ball_position[2])
        ball_pose.orientation.w = 1.0

        hole_pose = Pose()
        hole_pose.position.x = float(self.hole_position[0])
        hole_pose.position.y = float(self.hole_position[1])
        hole_pose.position.z = float(self.hole_position[2])
        hole_pose.orientation.w = 1.0

        # Align the club face asynchronously (assuming MPI has this method)
        # Make sure you've implemented align_club_face in MPI
        await self.MPI.align_club_face(ball_pose, hole_pose)

        self.get_logger().info("Attempting full real-world motion sequence...")
        success = await self.run_motion_sequence()
        if success:
            self.get_logger().info("Real-world putt done.")
        else:
            self.get_logger().error("Real-world putt failed.")
        return response

    async def run_motion_sequence(self):
        self.get_logger().info("Planning and executing Cartesian path...")
        result = await self.MPI.move_arm_cartesian(
            waypoints=self.waypoints,
            avoid_collisions=False
        )
        if result:
            self.get_logger().info("Trajectory executed successfully. Robot should have 'hit' the ball now.")
            self.get_logger().info("Animating the ball movement...")
            unit_direction, distance = self.calculate_ball_trajectory()
            self.animate_ball_movement(unit_direction, distance)
            return True
        else:
            self.get_logger().error("Planning or execution failed.")
            return False

    def calculate_ball_trajectory(self):
        unit_direction, distance = self.trajectory_calculator.calculate_trajectory()
        return unit_direction, distance

    def animate_ball_movement(self, unit_direction, distance):
        mu = 0.26
        g = 9.81
        a = -mu * g
        v0 = math.sqrt(2 * mu * g * distance)

        self.ball_animation_time = 0.0
        self.ball_animation_dt = 0.05
        self.ball_animation_end_time = -v0 / a
        self.ball_direction = unit_direction
        self.ball_initial_velocity = v0
        self.ball_acceleration = a

        if self.ball_animation_timer is not None:
            self.destroy_timer(self.ball_animation_timer)
        self.ball_animation_timer = self.create_timer(self.ball_animation_dt, self.ball_timer_callback)

    def ball_timer_callback(self):
        t = self.ball_animation_time
        if t > self.ball_animation_end_time:
            self.destroy_timer(self.ball_animation_timer)
            self.ball_animation_timer = None
            return

        s = self.ball_initial_velocity * t + 0.5 * self.ball_acceleration * (t**2)
        if s < 0:
            self.destroy_timer(self.ball_animation_timer)
            self.ball_animation_timer = None
            return

        position = self.ball_position + self.ball_direction * s
        self.ball_marker.pose.position.x = float(position[0])
        self.ball_marker.pose.position.y = float(position[1])
        self.ball_marker.pose.position.z = float(position[2])
        self.ball_marker.header.stamp = self.get_clock().now().to_msg()
        self.ball_marker_pub.publish(self.ball_marker)

        self.ball_animation_time += self.ball_animation_dt

    def timer_callback(self):
        if self.state == State.START:
            self.state = State.IDLE

def main(args=None):
    rclpy.init(args=args)
    node = DemoNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
