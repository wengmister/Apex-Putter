import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from enum import Enum, auto

from geometry_msgs.msg import Pose, Quaternion
from visualization_msgs.msg import Marker
from std_srvs.srv import Empty

from apex_putter.MotionPlanningInterface import MotionPlanningInterface
from apex_putter.BallTrajectoryCalc import BallTrajectoryCalculator

class State(Enum):
    START = auto()
    IDLE = auto()
    TASK = auto()

class DemoNode(Node):
    def __init__(self):
        super().__init__('demo_node')

        # Parameter to toggle simulation mode vs real-world mode
        self.declare_parameter('simulation_mode', True)
        self.use_simulation_mode = self.get_parameter('simulation_mode').get_parameter_value().bool_value

        # Initialize motion planning interface
        self.MPI = MotionPlanningInterface(
            node=self,
            base_frame='base',
            end_effector_frame='fer_link8'
        )

        # Hard-coded positions for simulation
        if self.use_simulation_mode:
            self.ball_position = np.array([0.5, 0.0, 0.0])
            self.hole_position = np.array([0.8, 0.0, 0.0])
        else:
            self.ball_position = np.array([0.5, 0.0, 0.0])
            self.hole_position = np.array([0.8, 0.0, 0.0])

        self.trajectory_calculator = BallTrajectoryCalculator(self.ball_position, self.hole_position)

        # Define the robot's start pose (Pose only, as in the working example)
        self.start_pose = Pose()
        self.start_pose.position.x = 0.4
        self.start_pose.position.y = 0.0
        self.start_pose.position.z = 0.4
        self.start_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Compute the waypoints dynamically from ball to hole
        self.waypoints = self.compute_waypoints()

        # Internal state machine
        self.state = State.START
        self.task_step = 0

        # Set up markers for ball and hole
        self.ball_marker_pub = self.create_publisher(Marker, 'ball_marker', 10)
        self.hole_marker_pub = self.create_publisher(Marker, 'hole_marker', 10)
        self.ball_marker = Marker()
        self.hole_marker = Marker()
        self.setup_ball_marker()
        self.setup_hole_marker()

        # Timer to continuously republish markers so they stay visible in RViz
        self.marker_timer = self.create_timer(1.0, self.publish_markers)

        # Animation variables for the ball
        self.ball_animation_timer = None
        self.ball_animation_time = 0.0
        self.ball_animation_dt = 0.05
        self.ball_animation_end_time = 0.0
        self.ball_initial_velocity = 0.0
        self.ball_acceleration = 0.0
        self.ball_direction = np.array([0.0, 0.0, 0.0])

        # Create a service to trigger the demo
        self.demo_test_srv = self.create_service(Empty, 'demo_test', self.test_callback, callback_group=MutuallyExclusiveCallbackGroup())

        # Use an asynchronous callback for the timer so we can await coroutines
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("DemoNode initialized. Waiting for demo_test service call to start demo.")


    def compute_waypoints(self):
        """Compute waypoints from behind the ball, through the ball, and beyond the hole."""
        direction = self.hole_position - self.ball_position
        distance = np.linalg.norm(direction)
        if distance < 1e-6:
            self.get_logger().error("Ball and hole are at the same position! Using default waypoints.")
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
            p.orientation.w = 1.0
            return p

        z_height = 0.4
        wp1 = make_pose(behind_ball_pos[0], behind_ball_pos[1], z_height)
        wp2 = make_pose(at_ball_pos[0], at_ball_pos[1], z_height)
        wp3 = make_pose(beyond_hole_pos[0], beyond_hole_pos[1], z_height)

        return [wp1, wp2, wp3]

    def default_waypoints(self):
        waypoint1 = Pose()
        waypoint1.position.x = 0.45
        waypoint1.position.y = 0.0
        waypoint1.position.z = 0.4
        waypoint1.orientation.w = 1.0

        waypoint2 = Pose()
        waypoint2.position.x = 0.5
        waypoint2.position.y = 0.0
        waypoint2.position.z = 0.4
        waypoint2.orientation.w = 1.0

        waypoint3 = Pose()
        waypoint3.position.x = 0.55
        waypoint3.position.y = 0.0
        waypoint3.position.z = 0.4
        waypoint3.orientation.w = 1.0
        return [waypoint1, waypoint2, waypoint3]

    def setup_ball_marker(self):
        """Set up a Marker to represent the ball in RViz."""
        self.ball_marker.ns = "ball_marker_ns"
        self.ball_marker.id = 0
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
        self.ball_marker.pose.position.x = float(self.ball_position[0])
        self.ball_marker.pose.position.y = float(self.ball_position[1])
        self.ball_marker.pose.position.z = float(self.ball_position[2])

    def setup_hole_marker(self):
        """Set up a Marker to represent the hole in RViz."""
        self.hole_marker.ns = "hole_marker_ns"
        self.hole_marker.id = 0
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
        self.hole_marker.pose.position.x = float(self.hole_position[0])
        self.hole_marker.pose.position.y = float(self.hole_position[1])
        self.hole_marker.pose.position.z = float(self.hole_position[2])

    def publish_markers(self):
        now = self.get_clock().now().to_msg()
        self.ball_marker.header.stamp = now
        self.hole_marker.header.stamp = now
        self.ball_marker_pub.publish(self.ball_marker)
        self.hole_marker_pub.publish(self.hole_marker)

    async def test_callback(self, request, response):
        self.task_step = 0
        self.state = State.TASK
        self.get_logger().info("Demo started via service call.")
        return response

    async def timer_callback(self):
        if self.state == State.START:
            self.state = State.IDLE

        elif self.state == State.IDLE:
            # Just waiting for 'demo_test' to start the task
            pass

        elif self.state == State.TASK:
            if self.task_step == 0:
                self.get_logger().info("Planning a cartesian path with computed waypoints...")
                result = await self.MPI.move_arm_cartesian(
                    waypoints=self.waypoints,
                    avoid_collisions=False
                )

                if result:
                    self.get_logger().info("Trajectory executed successfully. Robot should have 'hit.'")
                    self.task_step += 1

                    self.get_logger().info("Animating the ball movement with friction...")
                    unit_direction, distance = self.calculate_ball_trajectory()
                    self.animate_ball_movement(unit_direction, distance)

                    self.state = State.IDLE
                else:
                    self.get_logger().error("Planning or execution failed.")
                    self.state = State.IDLE

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

def main(args=None):
    rclpy.init(args=args)
    node = DemoNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
