# Write an integration test that uses the kinematic Franka simulation to 
# command the Franka Panda robot to move from one pose to another and verify that the pose is achieved.

from geometry_msgs.msg import Point, Pose, Quaternion

from motion_planner.MotionPlanner import MotionPlanner
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotState
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Empty

class Test_Integration(Node):
    def __init__(self):
        super().__init__('test_motion_planner')

        # Create callback group for the service
        cb_group = MutuallyExclusiveCallbackGroup()

        # Initialize service with callback group
        self.test_1 = self.create_service(
            Empty,
            'plan_joint',
            self.test_1_callback,
            callback_group=cb_group
        )

    def test_1_callback(self):
        pass



# Write an integration test, using the Franka Panda Robot demonstration that creates 
# a planning scene object and attempts to move the robot into that object and verify that planning fails.