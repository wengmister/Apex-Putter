"""
Picks up and moves an object in Gazebo.

SERVICES:
    + /pick (some_type) - picks up the object and moves it


"""

from enum import auto, Enum

from motion_planner.MotionPlanningInterface import MotionPlanningInterface
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotState
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Empty


class State(Enum):
    """Current state of the pick_node node."""
    START = auto(),
    IDLE = auto(),
    PLAN = auto(),
    EXECUTE = auto(),
    OPEN = auto(),
    CLOSE = auto(),
    PICK = auto(),
    PLACE = auto(),
    TASK = auto()


class PickNode(Node):
    """Picks up an object."""

    def __init__(self):
        super().__init__('pick_node')
        self.state = State.IDLE
        self.MPI = MotionPlanningInterface(
            self, 'base', 'fer_link8', '/joint_state_broadcaster/joint_states', 'fer_manipulator', 'fer_arm_controller')
        self.MPI_hand = MotionPlanningInterface(
            self, 'fer_hand', 'fer_hand_tcp', '/joint_states', 'hand', 'fer_gripper')
        self.man_joint_names = [
            'fer_joint1', 'fer_joint2', 'fer_joint3', 'fer_joint4',
            'fer_joint5', 'fer_joint6', 'fer_joint7'
        ]
        self.hand_joint_names = [
            'fer_finger_joint1', 'fer_finger_joint2'
        ]
        self.manipulator_pos = [
            [-0.017, 0.0524, 0.2793, -2.8274, -0.0698, 2.8798, 1.0995],
            [0.104719, 0.27925, 0.0872, -2.7925, -0.31415, 3.07178, 1.2915],
            [-0.01745, 0.13962, 0.24435, -2.82743, -0.17453, 2.96706, 1.16937],
            [-0.6807, 0.2094, -0.1047, -2.6005, 0.0698, 2.8099, 0.0]
        ]

        self.hand_pos = [
            [0.035, 0.035],
            [0.020, 0.020]
        ]

        self.curr_man = 0
        self.curr_hand = 0

        self.scene_parameters = [
            {
                'id': 'object',
                'size': (0.03, 0.03, 0.03),
                'position': (0.35, 0.099, 0.015),
                'orientation': (0.0, 0.0, 0.0, 1.0),
                'frame_id': 'base'
            },
            {
                'id': 'table',
                'size': (0.5, 1.0, 0.01),
                'position': (0.35, 0.0, -0.005),
                # 'orientation' is optional; defaults to (0.0, 0.0, 0.0, 1.0)
                'frame_id': 'base'
            },
            {
                'id': 'obstacle',
                'size': (0.3, 0.05, 0.2),
                'position': (0.25, -0.1, 0.1),
                'orientation': (0.0, 0.0, 0.0, 1.0),
                'frame_id': 'base'
            }
        ]
        self.scene_set = False
        self.scene_future = self.MPI.PlanningScene.load_scene_from_parameters(
            self.scene_parameters)

        # Create service
        self.pick = self.create_service(Empty, 'pick', self.pick_callback)

        self.timer = self.create_timer(1/100, self.timer_callback)

    def pick_callback(self, request, response):
        self.task_step = 0
        self.state = State.TASK
        return response

    def timer_callback(self):
        if self.state == State.IDLE:
            pass
        elif self.state == State.PLAN:
            # Get the planning future
            if self.plan_traj_future.done():
                self.execute_traj_future = self.MPI.execute_trajectory(
                    self.plan_traj_future.result())
                self.state = State.EXECUTE
        elif self.state == State.EXECUTE:
            if self.execute_traj_future.done():
                self.task_step += 1
                self.state = State.TASK
        elif self.state == State.OPEN:
            # Open gripper
            if self.plan_traj_future.done():
                self.execute_traj_future = self.MPI_hand.execute_trajectory(
                    self.plan_traj_future.result())
                self.state = State.EXECUTE
        elif self.state == State.CLOSE:
            # Close gripper
            if self.plan_traj_future.done():
                self.execute_traj_future = self.MPI_hand.execute_trajectory(
                    self.plan_traj_future.result())
                self.state = State.EXECUTE
        elif self.state == State.PICK:
            # Attach block to gripper in scene
            if self.attach_future.done():
                self.task_step += 1
                self.state = State.TASK
        elif self.state == State.PLACE:
            # Place block in scene
            if self.detach_future.done():
                self.task_step += 1
                self.state = State.TASK
        elif self.state == State.TASK:
            if self.task_step == 0:
                self.get_logger().info("Step 0")
                self.plan_traj_future = self.MPI.plan_joint_space(
                    self.man_joint_names,
                    self.manipulator_pos[self.curr_man],
                    robot_state=None,
                    max_velocity_scaling_factor=0.1,
                    max_acceleration_scaling_factor=0.1
                )
                if self.plan_traj_future is None:
                    self.get_logger().error('Planning failed - no trajectory returned')

                self.get_logger().info('Planning completed, executing trajectory...')
                self.state = State.PLAN
            elif self.task_step == 1:
                self.get_logger().info("Step 1")
                self.plan_traj_future = self.MPI_hand.plan_joint_space(
                    self.hand_joint_names,
                    self.hand_pos[self.curr_hand],
                    robot_state=None,
                    max_velocity_scaling_factor=0.1,
                    max_acceleration_scaling_factor=0.1
                )
                if self.plan_traj_future is None:
                    self.get_logger().error('Planning failed - no trajectory returned')
                self.state = State.OPEN
            elif self.task_step == 2:
                self.get_logger().info("Step 2")
                self.curr_man += 1
                self.plan_traj_future = self.MPI.plan_joint_space(
                    self.man_joint_names,
                    self.manipulator_pos[self.curr_man],
                    robot_state=None,
                    max_velocity_scaling_factor=0.1,
                    max_acceleration_scaling_factor=0.1
                )
                self.state = State.PLAN
            elif self.task_step == 3:
                self.get_logger().info("Step 3")
                self.curr_hand += 1
                self.plan_traj_future = self.MPI_hand.plan_joint_space(
                    self.hand_joint_names,
                    self.hand_pos[self.curr_hand],
                    robot_state=None,
                    max_velocity_scaling_factor=0.1,
                    max_acceleration_scaling_factor=0.1
                )
                if self.plan_traj_future is None:
                    self.get_logger().error('Planning failed - no trajectory returned')
                self.state = State.CLOSE
            elif self.task_step == 4:
                self.curr_man += 1
                self.plan_traj_future = self.MPI.plan_joint_space(
                    self.man_joint_names,
                    self.manipulator_pos[self.curr_man],
                    robot_state=None,
                    max_velocity_scaling_factor=0.1,
                    max_acceleration_scaling_factor=0.1
                )
                self.state = State.PLAN
            elif self.task_step == 5:
                self.attach_future = self.MPI_hand.attach_object(
                    'object', 'fer_hand')
                self.state = State.PICK
            elif self.task_step == 6:
                self.curr_man += 1
                self.plan_traj_future = self.MPI.plan_joint_space(
                    self.man_joint_names,
                    self.manipulator_pos[self.curr_man],
                    robot_state=None,
                    max_velocity_scaling_factor=0.1,
                    max_acceleration_scaling_factor=0.1
                )
                self.state = State.PLAN
            elif self.task_step == 7:
                self.curr_hand -= 1
                self.plan_traj_future = self.MPI_hand.plan_joint_space(
                    self.hand_joint_names,
                    self.hand_pos[self.curr_hand],
                    robot_state=None,
                    max_velocity_scaling_factor=0.1,
                    max_acceleration_scaling_factor=0.1
                )
                if self.plan_traj_future is None:
                    self.get_logger().error('Planning failed - no trajectory returned')
                self.state = State.OPEN
            elif self.task_step == 8:
                self.detach_future = self.MPI_hand.detach_object(
                    'object', 'fer_hand')
                self.state = State.PLACE
            else:
                self.state = State.IDLE


def main(args=None):
    """Execute the main loop of the node."""
    rclpy.init(args=args)
    my_node = PickNode()
    rclpy.spin(my_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
