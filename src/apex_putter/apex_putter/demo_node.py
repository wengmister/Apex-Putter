"""
Picks up and moves an object in Gazebo.

SERVICES:
    + /pick (some_type) - picks up the object and moves it
"""

from enum import auto, Enum

from apex_putter.MotionPlanningInterface import MotionPlanningInterface
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotState
import numpy as np
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import Point, Pose, Quaternion
from visualization_msgs.msg import Marker

from apex_putter.BallTrajectoryCalc import BallTrajectoryCalculator


class State(Enum):
    """Current state of the demo_node node."""
    START = auto(),
    IDLE = auto(),
    PLAN = auto(),
    EXECUTE = auto(),
    OPEN = auto(),
    CLOSE = auto(),
    PICK = auto(),
    PLACE = auto(),
    TASK = auto()


class DemoNode(Node):
    """Demo node that plans and executes robot trajectories, and animates a ball."""

    def __init__(self):
        super().__init__('demo_node')
        self.state = State.IDLE
        self.MPI = MotionPlanningInterface(
            self, 'base', 'fer_link8', '/joint_state_broadcaster/joint_states', 'fer_manipulator', 'fer_arm_controller')
        self.MPI_hand = MotionPlanningInterface(
            self, 'fer_hand', 'fer_hand_tcp', '/joint_state_broadcaster/joint_states', 'hand', 'fer_gripper')
        self.man_joint_names = [
            'fer_joint1', 'fer_joint2', 'fer_joint3', 'fer_joint4',
            'fer_joint5', 'fer_joint6', 'fer_joint7'
        ]
        self.hand_joint_names = [
            'fer_finger_joint1', 'fer_finger_joint2'
        ]
        self.manipulator_pos = [
            [-0.017, -0.017, 0.2967, -2.8448, 0.017, 2.8274, 1.0472],
            [0.0873, 0.2618, 0.1047, -2.7925, -0.2967, 3.0369, 1.274],
            [0.0174, 0.1919, 0.1919, -2.8099, -0.2269, 2.9845, 1.2217],
            [-0.6807, 0.2094, -0.1047, -2.6005, 0.0698, 2.8099, 0.0]
        ]

        self.hand_pos = [
            [0.035, 0.035],
            [0.026, 0.026]
        ]

        # Set up the waypoints
        self.waypoints = []
        # First waypoint
        waypoint1 = Pose()
        waypoint1.position = Point(x=0.5, y=0.5, z=0.5)
        waypoint1.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        # Second waypoint
        waypoint2 = Pose()
        waypoint2.position = Point(x=-0.5, y=0.3, z=0.7)
        waypoint2.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.waypoints.append(waypoint1)
        self.waypoints.append(waypoint2)

        self.curr_man = 0
        self.curr_hand = 0

        self.scene_parameters = [
            {
                'id': 'ball',
                'size': (0.03, 0.03, 0.03),
                'position': (0.35, 0.1, 0.015),
                'orientation': (0.0, 0.0, 0.0, 1.0),
                'frame_id': 'base'
            }
        ]
        self.scene_set = False
        self.scene_future = self.MPI.PlanningScene.load_scene_from_parameters(
            self.scene_parameters)

        # Create service
        self.demo_test = self.create_service(Empty, 'demo_test', self.test_callback)
        self.timer = self.create_timer(1/100, self.timer_callback)

        # Define ball and hole positions
        self.ball_position = [0.5, 0.0, 0.0]  # TODO: Adjust as needed
        self.hole_position = [0.8, 0.0, 0.0]  # TODO: Adjust as needed
        self.trajectory_calculator = BallTrajectoryCalculator(self.ball_position, self.hole_position)

        # Initialize the ball marker
        self.setup_ball_marker()

        # Initialize the hole marker
        self.setup_hole_marker()

        # Animation variables (for the ball)
        self.ball_animation_timer = None
        self.ball_animation_time = 0.0
        self.ball_animation_dt = 0.05
        self.ball_animation_end_time = 0.0
        self.ball_initial_velocity = 0.0
        self.ball_acceleration = 0.0
        self.ball_direction = np.array([0.0, 0.0, 0.0])

        # Putter dimensions and pose
        self.putter_length = 0.2
        self.putter_width = 0.03
        self.putter_height = 0.03
        # Position of the putter collision box relative to 'fer_link8'
        self.putter_pose = (self.putter_length/2.0, 0.0, 0.0)

        # Start the execution flow
        self.state = State.PLAN

    def setup_ball_marker(self):
        """Set up a Marker to represent the ball for visualization."""
        self.ball_marker_pub = self.create_publisher(Marker, 'ball_marker', 10)
        self.ball_marker = Marker()
        self.ball_marker.header.frame_id = 'base'
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
        self.ball_marker.pose.position.x = self.ball_position[0]
        self.ball_marker.pose.position.y = self.ball_position[1]
        self.ball_marker.pose.position.z = self.ball_position[2]
        self.ball_marker_pub.publish(self.ball_marker)

    def setup_hole_marker(self):
        """Set up a Marker to represent the hole for visualization."""
        self.hole_marker_pub = self.create_publisher(Marker, 'hole_marker', 10)
        self.hole_marker = Marker()
        self.hole_marker.header.frame_id = 'base'
        self.hole_marker.type = Marker.CYLINDER
        self.hole_marker.action = Marker.ADD
        # Set the hole size; adjust as needed
        self.hole_marker.scale.x = 0.1  # diameter in x
        self.hole_marker.scale.y = 0.1  # diameter in y
        self.hole_marker.scale.z = 0.01 # very thin cylinder
        self.hole_marker.color.r = 0.0
        self.hole_marker.color.g = 0.0
        self.hole_marker.color.b = 0.0
        self.hole_marker.color.a = 1.0
        self.hole_marker.pose.orientation.w = 1.0
        self.hole_marker.pose.position.x = self.hole_position[0]
        self.hole_marker.pose.position.y = self.hole_position[1]
        self.hole_marker.pose.position.z = self.hole_position[2]
        self.hole_marker_pub.publish(self.hole_marker)

"""
Picks up and moves an object in Gazebo.

SERVICES:
    + /pick (some_type) - picks up the object and moves it
"""

from enum import auto, Enum

from apex_putter.MotionPlanningInterface import MotionPlanningInterface
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotState
import numpy as np
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import Point, Pose, Quaternion
from visualization_msgs.msg import Marker

from apex_putter.BallTrajectoryCalc import BallTrajectoryCalculator


class State(Enum):
    """Current state of the demo_node node."""
    START = auto(),
    IDLE = auto(),
    PLAN = auto(),
    EXECUTE = auto(),
    OPEN = auto(),
    CLOSE = auto(),
    PICK = auto(),
    PLACE = auto(),
    TASK = auto()


class DemoNode(Node):
    """Demo node that plans and executes robot trajectories, and animates a ball."""

    def __init__(self):
        super().__init__('demo_node')
        self.state = State.IDLE
        self.MPI = MotionPlanningInterface(
            self, 'base', 'fer_link8', '/joint_state_broadcaster/joint_states', 'fer_manipulator', 'fer_arm_controller')
        self.MPI_hand = MotionPlanningInterface(
            self, 'fer_hand', 'fer_hand_tcp', '/joint_state_broadcaster/joint_states', 'hand', 'fer_gripper')
        self.man_joint_names = [
            'fer_joint1', 'fer_joint2', 'fer_joint3', 'fer_joint4',
            'fer_joint5', 'fer_joint6', 'fer_joint7'
        ]
        self.hand_joint_names = [
            'fer_finger_joint1', 'fer_finger_joint2'
        ]
        self.manipulator_pos = [
            [-0.017, -0.017, 0.2967, -2.8448, 0.017, 2.8274, 1.0472],
            [0.0873, 0.2618, 0.1047, -2.7925, -0.2967, 3.0369, 1.274],
            [0.0174, 0.1919, 0.1919, -2.8099, -0.2269, 2.9845, 1.2217],
            [-0.6807, 0.2094, -0.1047, -2.6005, 0.0698, 2.8099, 0.0]
        ]

        self.hand_pos = [
            [0.035, 0.035],
            [0.026, 0.026]
        ]

        # Set up the waypoints
        self.waypoints = []
        # First waypoint
        waypoint1 = Pose()
        waypoint1.position = Point(x=0.5, y=0.5, z=0.5)
        waypoint1.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        # Second waypoint
        waypoint2 = Pose()
        waypoint2.position = Point(x=-0.5, y=0.3, z=0.7)
        waypoint2.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.waypoints.append(waypoint1)
        self.waypoints.append(waypoint2)

        self.curr_man = 0
        self.curr_hand = 0

        self.scene_parameters = [
            {
                'id': 'ball',
                'size': (0.03, 0.03, 0.03),
                'position': (0.35, 0.1, 0.015),
                'orientation': (0.0, 0.0, 0.0, 1.0),
                'frame_id': 'base'
            }
        ]
        self.scene_set = False
        self.scene_future = self.MPI.PlanningScene.load_scene_from_parameters(
            self.scene_parameters)

        # Create service
        self.demo_test = self.create_service(Empty, 'demo_test', self.test_callback)
        self.timer = self.create_timer(1/100, self.timer_callback)

        # Define ball and hole positions
        self.ball_position = [0.5, 0.0, 0.0]  # Adjust as needed
        self.hole_position = [0.8, 0.0, 0.0]  # Adjust as needed
        self.trajectory_calculator = BallTrajectoryCalculator(self.ball_position, self.hole_position)

        # Initialize the ball marker
        self.setup_ball_marker()

        # Initialize the hole marker
        self.setup_hole_marker()

        # Animation variables (for the ball)
        self.ball_animation_timer = None
        self.ball_animation_time = 0.0
        self.ball_animation_dt = 0.05
        self.ball_animation_end_time = 0.0
        self.ball_initial_velocity = 0.0
        self.ball_acceleration = 0.0
        self.ball_direction = np.array([0.0, 0.0, 0.0])

        # Remove putter-related lines
        # self.putter_length = 0.2
        # self.putter_width = 0.03
        # self.putter_height = 0.03
        # self.putter_pose = (self.putter_length/2.0, 0.0, 0.0)

        self.state = State.PLAN

    def setup_ball_marker(self):
        """Set up a Marker to represent the ball for visualization."""
        self.ball_marker_pub = self.create_publisher(Marker, 'ball_marker', 10)
        self.ball_marker = Marker()
        self.ball_marker.header.frame_id = 'base'
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
        self.ball_marker.pose.position.x = self.ball_position[0]
        self.ball_marker.pose.position.y = self.ball_position[1]
        self.ball_marker.pose.position.z = self.ball_position[2]
        self.ball_marker_pub.publish(self.ball_marker)

    def setup_hole_marker(self):
        """Set up a Marker to represent the hole for visualization."""
        self.hole_marker_pub = self.create_publisher(Marker, 'hole_marker', 10)
        self.hole_marker = Marker()
        self.hole_marker.header.frame_id = 'base'
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
        self.hole_marker.pose.position.x = self.hole_position[0]
        self.hole_marker.pose.position.y = self.hole_position[1]
        self.hole_marker.pose.position.z = self.hole_position[2]
        self.hole_marker_pub.publish(self.hole_marker)

    # Remove the attach_putter method entirely
    # def attach_putter(self):
    #     pass

    def calculate_ball_trajectory(self):
        unit_direction, distance = self.trajectory_calculator.calculate_trajectory()
        return unit_direction, distance

    async def test_callback(self, request, response):
        self.task_step = 0
        self.state = State.TASK
        return response

    def animate_ball_movement(self, unit_direction, distance):
        """Set up parameters for ball animation and start a timer to animate."""
        mu = 0.26  # friction coefficient
        g = 9.81
        a = -mu * g
        v0 = self.MPI.MotionPlanner.calculate_initial_ball_velocity(distance, mu=mu)

        self.ball_animation_time = 0.0
        self.ball_animation_dt = 0.05
        self.ball_animation_end_time = -v0 / a
        self.ball_direction = unit_direction
        self.ball_initial_velocity = v0
        self.ball_acceleration = a

        # Create a timer that updates the ball position each step
        self.ball_animation_timer = self.create_timer(self.ball_animation_dt, self.ball_timer_callback)

    def ball_timer_callback(self):
        """Update the ball's position each timestep until it stops."""
        t = self.ball_animation_time

        if t > self.ball_animation_end_time:
            # Stop the animation
            self.destroy_timer(self.ball_animation_timer)
            self.ball_animation_timer = None
            return

        s = self.ball_initial_velocity * t + 0.5 * self.ball_acceleration * t**2
        if s < 0:
            # Ball shouldn't roll backwards
            self.destroy_timer(self.ball_animation_timer)
            self.ball_animation_timer = None
            return

        position = np.array(self.ball_position) + self.ball_direction * s
        self.ball_marker.pose.position.x = float(position[0])
        self.ball_marker.pose.position.y = float(position[1])
        self.ball_marker.pose.position.z = float(position[2])
        self.ball_marker.header.stamp = self.get_clock().now().to_msg()
        self.ball_marker_pub.publish(self.ball_marker)

        self.ball_animation_time += self.ball_animation_dt

    async def gripper_callback(self):
        planned_traj = await self.MPI_hand.plan_joint_space_async(
            self.hand_joint_names,
            self.hand_pos[self.curr_hand],
            robot_state=None,
            max_velocity_scaling_factor=0.1,
            max_acceleration_scaling_factor=0.1
        )
        if planned_traj is not None:
            status = await self.MPI_hand.execute_trajectory_async(planned_traj)

    def timer_callback(self):
        # Just wait for the scene to load if needed
        if not self.scene_set and self.scene_future.done():
            self.scene_set = True
            # Not attaching the putter now

        if self.state == State.IDLE:
            pass
        elif self.state == State.PLAN:
            if hasattr(self, 'plan_traj_future') and self.plan_traj_future.done():
                self.execute_traj_future = self.MPI.execute_trajectory(
                    self.plan_traj_future.result())
                self.get_logger().info("Plan loop ended")
                self.state = State.EXECUTE
        elif self.state == State.EXECUTE:
            if hasattr(self, 'execute_traj_future') and self.execute_traj_future.done():
                self.task_step += 1
                self.get_logger().info("Execution loop ended")
                self.state = State.TASK
        elif self.state == State.TASK:
            if self.task_step == 0:
                self.get_logger().info("Step 0")
                self.get_logger().info(f'Planned cartesian waypoints: {self.waypoints}')
                self.plan_traj_future = self.MPI.plan_cartesian_path(waypoints=self.waypoints)
                if self.plan_traj_future is None:
                    self.get_logger().error('Planning failed - no trajectory returned')
                else:
                    self.get_logger().info('Planning completed, executing trajectory...')
                    self.state = State.PLAN
            elif self.task_step == 1:
                self.get_logger().info("Step 1: Robot done. Animate the ball now.")
                unit_direction, distance = self.calculate_ball_trajectory()
                self.animate_ball_movement(unit_direction, distance)
                self.state = State.IDLE
            else:
                self.state = State.IDLE


def main(args=None):
    rclpy.init(args=args)
    my_node = DemoNode()
    rclpy.spin(my_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


    def calculate_ball_trajectory(self):
        unit_direction, distance = self.trajectory_calculator.calculate_trajectory()
        return unit_direction, distance

    async def test_callback(self, request, response):
        self.task_step = 0
        self.state = State.TASK
        return response

    def animate_ball_movement(self, unit_direction, distance):
        """Set up parameters for ball animation and start a timer to animate."""
        mu = 0.26  # friction coefficient
        g = 9.81
        a = -mu * g
        v0 = self.MPI.MotionPlanner.calculate_initial_ball_velocity(distance, mu=mu)

        self.ball_animation_time = 0.0
        self.ball_animation_dt = 0.05
        self.ball_animation_end_time = -v0 / a
        self.ball_direction = unit_direction
        self.ball_initial_velocity = v0
        self.ball_acceleration = a

        # Create a timer that updates the ball position at each step
        self.ball_animation_timer = self.create_timer(self.ball_animation_dt, self.ball_timer_callback)

    def ball_timer_callback(self):
        """Update the ball's position each timestep until it stops."""
        t = self.ball_animation_time

        if t > self.ball_animation_end_time:
            # Stop the animation
            self.destroy_timer(self.ball_animation_timer)
            self.ball_animation_timer = None
            return

        s = self.ball_initial_velocity * t + 0.5 * self.ball_acceleration * t**2
        if s < 0:
            # Ball should not roll backwards; stop if that happens
            self.destroy_timer(self.ball_animation_timer)
            self.ball_animation_timer = None
            return

        position = np.array(self.ball_position) + self.ball_direction * s
        self.ball_marker.pose.position.x = float(position[0])
        self.ball_marker.pose.position.y = float(position[1])
        self.ball_marker.pose.position.z = float(position[2])
        self.ball_marker.header.stamp = self.get_clock().now().to_msg()
        self.ball_marker_pub.publish(self.ball_marker)

        self.ball_animation_time += self.ball_animation_dt

    async def gripper_callback(self):
        planned_traj = await self.MPI_hand.plan_joint_space_async(
            self.hand_joint_names,
            self.hand_pos[self.curr_hand],
            robot_state=None,
            max_velocity_scaling_factor=0.1,
            max_acceleration_scaling_factor=0.1
        )
        if planned_traj is not None:
            status = await self.MPI_hand.execute_trajectory_async(planned_traj)

    def timer_callback(self):
        # Wait for planning scene to load if not done yet
        if not self.scene_set and self.scene_future.done():
            self.scene_set = True
            # Attach the putter once the scene is loaded
            self.attach_putter()

        if self.state == State.IDLE:
            pass
        elif self.state == State.PLAN:
            # Wait for the plan to complete
            if hasattr(self, 'plan_traj_future') and self.plan_traj_future.done():
                self.execute_traj_future = self.MPI.execute_trajectory(
                    self.plan_traj_future.result())
                self.get_logger().info("Plan loop ended")
                self.state = State.EXECUTE
        elif self.state == State.EXECUTE:
            # Wait for execution to complete
            if hasattr(self, 'execute_traj_future') and self.execute_traj_future.done():
                self.task_step += 1
                self.get_logger().info("Execution loop ended")
                self.state = State.TASK
        elif self.state == State.TASK:
            if self.task_step == 0:
                self.get_logger().info("Step 0")
                self.get_logger().info(f'Planned cartesian waypoints: {self.waypoints}')
                # Plan the cartesian path
                self.plan_traj_future = self.MPI.plan_cartesian_path(waypoints=self.waypoints)
                if self.plan_traj_future is None:
                    self.get_logger().error('Planning failed - no trajectory returned')
                else:
                    self.get_logger().info('Planning completed, executing trajectory...')
                    self.state = State.PLAN
            elif self.task_step == 1:
                # After the robot has executed the planned trajectory, animate the ball
                self.get_logger().info("Step 1: Robot done. Animate the ball now.")
                unit_direction, distance = self.calculate_ball_trajectory()
                self.animate_ball_movement(unit_direction, distance)
                self.state = State.IDLE
            else:
                self.state = State.IDLE


def main(args=None):
    """Execute the main loop of the node."""
    rclpy.init(args=args)
    my_node = DemoNode()
    rclpy.spin(my_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
