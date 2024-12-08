import math
import numpy as np
import rclpy
from rclpy.node import Node
from enum import Enum, auto

from geometry_msgs.msg import Pose, Quaternion
from visualization_msgs.msg import Marker
from std_srvs.srv import Empty
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import tf2_ros

from apex_putter.MotionPlanningInterface import MotionPlanningInterface
import apex_putter.transform_operations as transOps

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
        self.declare_parameter('ball_tag_frame', 'ball')
        self.declare_parameter('hole_tag_frame', 'tag_15')
        self.declare_parameter('robot_base_tag_frame', 'robot_base_tag')
        self.declare_parameter('base_frame', 'base')
        self.declare_parameter('camera_frame', 'camera_link')

        self.ball_tag_frame = self.get_parameter('ball_tag_frame').get_parameter_value().string_value
        self.hole_tag_frame = self.get_parameter('hole_tag_frame').get_parameter_value().string_value
        self.robot_base_tag_frame = self.get_parameter('robot_base_tag_frame').get_parameter_value().string_value
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

        # Start pose
        self.start_pose = Pose()
        self.start_pose.position.x = 0.4
        self.start_pose.position.y = 0.0
        self.start_pose.position.z = 0.4
        self.start_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Now we can safely compute waypoints since tf_buffer is ready
        self.waypoints = self.compute_waypoints()

        # State machine
        self.state = State.START
        self.task_step = 0

        # Markers
        # self.hole_marker_pub = self.create_publisher(Marker, 'hole_marker', 10)
        # self.hole_marker = Marker()
        # self.setup_hole_marker()
        # self.ball_animation_timer = None

        # 1 Hz
        # self.marker_timer = self.create_timer(0.001, self.publish_markers)

        # Services
        self.ready_srv = self.create_service(Empty, 'ready', self.ready_callback, callback_group=MutuallyExclusiveCallbackGroup())
        self.home_srv = self.create_service(Empty, 'home', self.home_callback, callback_group=MutuallyExclusiveCallbackGroup())
        self.putt_srv = self.create_service(Empty, 'putt', self.putt_callback, callback_group=MutuallyExclusiveCallbackGroup())

        # Timer for optional tasks
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("DemoNode initialized. Use '/simulate' or '/real_putt'.")

    async def home_callback(self, request, response):
        """Make the robot go to home pose"""
        self.get_logger().info("Home requested.")
        await self.MPI.plan_to_named_configuration('home')
        return response

    def update_real_world_positions(self):
        try:
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

    def transform_point_to_fer_link8_and_apply_offset(self, target_point_in_base):
        fer_link8_tf = self.tf_buffer.lookup_transform(self.base_frame, 'fer_link8', rclpy.time.Time())
        T_base_fer = transOps.transform_to_htm(fer_link8_tf.transform)
        T_fer_base = np.linalg.inv(T_base_fer)

        target_hom = np.array([target_point_in_base[0], target_point_in_base[1], target_point_in_base[2], 1.0])
        target_in_fer = T_fer_base @ target_hom

        x_fer = target_in_fer[0]
        y_fer = target_in_fer[1]
        z_fer = target_in_fer[2]

        # Apply putter offset in fer_link8 frame
        fer_link8_x = x_fer - self.putter_offset
        fer_link8_y = y_fer
        fer_link8_z = z_fer + self.putter_length

        fer_link8_pos_in_fer = np.array([fer_link8_x, fer_link8_y, fer_link8_z, 1.0])
        fer_link8_pos_in_base = T_base_fer @ fer_link8_pos_in_fer

        return fer_link8_pos_in_base[0:3]

    def apply_putter_offset_in_base(self, p: Pose):
        # In simulation mode, just apply the offset directly in base frame
        x = p.position.x
        y = p.position.y
        z = p.position.z

        # Decrease Z by putter_length (instead of increasing) to move downward
        # z -= self.putter_length
        # Move X by putter_offset as before (assuming negative x is correct)
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

        if self.use_simulation_mode:
            # Simulation mode: apply offset directly in base frame
            wp1 = make_pose(behind_ball_pos[0], behind_ball_pos[1], 0.4)
            wp2 = make_pose(at_ball_pos[0], at_ball_pos[1], 0.4)
            wp3 = make_pose(beyond_hole_pos[0], beyond_hole_pos[1], 0.4)

            wp1 = self.apply_putter_offset_in_base(wp1)
            wp2 = self.apply_putter_offset_in_base(wp2)
            wp3 = self.apply_putter_offset_in_base(wp3)
        else:
            # Real world mode: use TF-based approach
            def make_pose_from_point_in_base(target_point):
                fer_link8_pos_in_base = self.transform_point_to_fer_link8_and_apply_offset(target_point)
                p = Pose()
                p.position.x = float(fer_link8_pos_in_base[0])
                p.position.y = float(fer_link8_pos_in_base[1])
                p.position.z = float(fer_link8_pos_in_base[2])
                p.orientation.x = qx
                p.orientation.y = qy
                p.orientation.z = qz
                p.orientation.w = qw
                return p

            wp1 = make_pose_from_point_in_base(behind_ball_pos)
            wp2 = make_pose_from_point_in_base(at_ball_pos)
            wp3 = make_pose_from_point_in_base(beyond_hole_pos)

        return [wp1, wp2, wp3]

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
            self.get_logger().info("Real putt requested, but simulation_mode is True. Switch to False.")
            return response

        self.get_logger().info("Real-world putt requested.")
        self.update_real_world_positions()
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

def main(args=None):
    rclpy.init(args=args)
    node = DemoNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
