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

# Import quaternion_from_euler to set orientation (currently not used)
from tf_transformations import quaternion_from_euler

class State(Enum):
    START = auto()
    IDLE = auto()
    TASK = auto()

class DemoNode(Node):
    def __init__(self):
        super().__init__('demo_node')

        # Parameter to toggle simulation vs real-world
        self.declare_parameter('simulation_mode', True)
        self.use_simulation_mode = self.get_parameter('simulation_mode').get_parameter_value().bool_value

        # Declare parameters for frames
        self.declare_parameter('ball_tag_frame', 'ball_tag')
        self.declare_parameter('hole_tag_frame', 'hole_tag')
        self.declare_parameter('robot_base_tag_frame', 'robot_base_tag')
        self.declare_parameter('base_frame', 'base')
        self.declare_parameter('camera_frame', 'camera_link')

        self.ball_tag_frame = self.get_parameter('ball_tag_frame').get_parameter_value().string_value
        self.hole_tag_frame = self.get_parameter('hole_tag_frame').get_parameter_value().string_value
        self.robot_base_tag_frame = self.get_parameter('robot_base_tag_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value

        # Known offsets:
        # Putter length ~22.85 in = 0.58 m
        # Offset from shaft: 0.18 in = ~0.00457 m
        self.putter_length = 22.85 * 0.0254
        self.putter_offset = 0.18 * 0.0254

        # Motion planning interface
        self.MPI = MotionPlanningInterface(
            node=self,
            base_frame=self.base_frame,
            end_effector_frame='fer_link8'
        )

        # Hard-coded hole (used in sim mode)
        self.hole_position = np.array([0.8, 0.0, 0.0])
        self.ball_position = np.array([0.5, 0.0, 0.0])  # Updated in real mode

        self.trajectory_calculator = BallTrajectoryCalculator(self.ball_position, self.hole_position)

        # Start pose
        self.start_pose = Pose()
        self.start_pose.position.x = 0.4
        self.start_pose.position.y = 0.0
        self.start_pose.position.z = 0.4
        self.start_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Initial waypoints
        self.waypoints = self.compute_waypoints()

        # TF setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State machine
        self.state = State.START
        self.task_step = 0

        # Markers
        self.ball_marker_pub = self.create_publisher(Marker, 'ball_marker', 10)
        self.hole_marker_pub = self.create_publisher(Marker, 'hole_marker', 10)
        self.ball_marker = Marker()
        self.hole_marker = Marker()
        self.setup_ball_marker()
        self.setup_hole_marker()
        self.ball_animation_timer = None

        # 1 Hz
        self.marker_timer = self.create_timer(1.0, self.publish_markers)

        # Services
        self.sim_srv = self.create_service(Empty, 'simulate', self.sim_callback, callback_group=MutuallyExclusiveCallbackGroup())
        self.real_putt_srv = self.create_service(Empty, 'real_putt', self.real_putt_callback, callback_group=MutuallyExclusiveCallbackGroup())

        # Timer for optional tasks
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("DemoNode initialized. Use '/simulate' for simulation or '/real_putt' for real-world mode.")

    def update_real_world_positions(self):
        try:
            # If direct transforms from base->tag are available, uncomment these:
            # hole_tf = self.tf_buffer.lookup_transform(self.base_frame, self.hole_tag_frame, rclpy.time.Time())
            # self.hole_position = np.array([
            #     hole_tf.transform.translation.x,
            #     hole_tf.transform.translation.y,
            #     hole_tf.transform.translation.z
            # ])
            #
            # ball_tf = self.tf_buffer.lookup_transform(self.base_frame, self.ball_tag_frame, rclpy.time.Time())
            # self.ball_position = np.array([
            #     ball_tf.transform.translation.x,
            #     ball_tf.transform.translation.y,
            #     ball_tf.transform.translation.z
            # ])

            # Otherwise, use matrix approach:
            base_cam_tf = self.tf_buffer.lookup_transform(self.base_frame, self.camera_frame, rclpy.time.Time())
            cam_hole_tf = self.tf_buffer.lookup_transform(self.camera_frame, self.hole_tag_frame, rclpy.time.Time())
            cam_ball_tf = self.tf_buffer.lookup_transform(self.camera_frame, self.ball_tag_frame, rclpy.time.Time())

            T_base_cam = transOps.transform_to_htm(base_cam_tf.transform)
            T_cam_hole = transOps.transform_to_htm(cam_hole_tf.transform)
            T_cam_ball = transOps.transform_to_htm(cam_ball_tf.transform)

            T_base_hole = T_base_cam @ T_cam_hole
            T_base_ball = T_base_cam @ T_cam_ball

            ball_offset = np.array([0.0, 0.0, 0.0])
            self.hole_position = T_base_hole[0:3, 3]
            self.ball_position = T_base_ball[0:3, 3] + ball_offset

            self.get_logger().info(f"Real world positions updated: Ball at {self.ball_position}, Hole at {self.hole_position}")
        except Exception as e:
            self.get_logger().error(f"Could not update real world positions: {e}")

    # Commenting out the putter offset application for testing
    # def apply_putter_offset(self, waypoint_pose):
    #     x = waypoint_pose.position.x
    #     y = waypoint_pose.position.y
    #     z = waypoint_pose.position.z

    #     z = z + self.putter_length
    #     x = x - self.putter_offset

    #     waypoint_pose.position.x = x
    #     waypoint_pose.position.y = y
    #     waypoint_pose.position.z = z
    #     return waypoint_pose

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

        def make_pose(x, y, z):
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            p.position.z = float(z)
            # Keep orientation as identity, assuming fer_link8 is already oriented correctly
            p.orientation.w = 1.0
            p.orientation.x = 0.0
            p.orientation.y = 0.0
            p.orientation.z = 0.0
            return p

        z_height = 0.4
        wp1 = make_pose(behind_ball_pos[0], behind_ball_pos[1], z_height)
        wp2 = make_pose(at_ball_pos[0], at_ball_pos[1], z_height)
        wp3 = make_pose(beyond_hole_pos[0], beyond_hole_pos[1], z_height)

        # Commenting out putter offset for testing
        # if not self.use_simulation_mode:
        #     wp1 = self.apply_putter_offset(wp1)
        #     wp2 = self.apply_putter_offset(wp2)
        #     wp3 = self.apply_putter_offset(wp3)

        return [wp1, wp2, wp3]

    def default_waypoints(self):
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
        success = await self.run_motion_sequence()
        if success:
            self.get_logger().info("Simulation done.")
        else:
            self.get_logger().error("Simulation failed.")
        return response

    async def real_putt_callback(self, request, response):
        if self.use_simulation_mode:
            self.get_logger().info("Real putt requested, but simulation_mode is True. Switch to False.")
            return response

        self.get_logger().info("Real-world putt requested.")
        # Update real-world positions from TF
        self.update_real_world_positions()
        # Recompute waypoints
        self.waypoints = self.compute_waypoints()

        self.get_logger().info("Executing real-world motion sequence...")
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
            self.get_logger().info("Trajectory executed. Robot has 'hit' the ball.")
            self.get_logger().info("Animating ball movement...")
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
        # You can leave this empty if no periodic tasks are needed
        if self.state == State.START:
            self.state = State.IDLE

def main(args=None):
    rclpy.init(args=args)
    node = DemoNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
