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
from geometry_msgs.msg import Point, Pose, Quaternion


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
    """Picks up an object."""

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

    async def test_callback(self, request, response):
        self.task_step = 0
        self.state = State.TASK
        return response

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
        if self.state == State.IDLE:
            pass
        elif self.state == State.PLAN:
            # Get the planning future
            if self.plan_traj_future.done():
                self.execute_traj_future = self.MPI.execute_trajectory(
                    self.plan_traj_future.result())
                self.get_logger().info("Plan loop ended")                
                self.state = State.EXECUTE
        elif self.state == State.EXECUTE:
            if self.execute_traj_future.done():
                self.task_step += 1
                self.get_logger().info("Execution loop ended")
                self.state = State.TASK
        elif self.state == State.TASK:
            if self.task_step == 0:
                self.get_logger().info("Step 0")
                self.get_logger().info(f'Planned cartesian waypoints: {self.waypoints}')
                # robot_state = self.MPI.RobotState.get_current_joint_state()
                # robot_pose = self.MPI.RobotState.compute_forward_kinematics(robot_state)
                # robot_pose = robot_state.position
                # self.get_logger().info(f'Curr robot state pose converted : {robot_pose}')

                # start_pose=robot_state,
                self.plan_traj_future = self.MPI.plan_cartesian_path(waypoints=self.waypoints)         

                # self.plan_traj_future = self.MPI.plan_joint_space(
                #     self.man_joint_names,
                #     self.manipulator_pos[self.curr_man],
                #     robot_state=None,
                #     max_velocity_scaling_factor=0.1,
                #     max_acceleration_scaling_factor=0.1
                # )

                if self.plan_traj_future is None:
                    self.get_logger().error('Planning failed - no trajectory returned')

                self.get_logger().info('Planning completed, executing trajectory...')
                self.state = State.PLAN            
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
