"""
Picks up and moves an object in Gazebo.

SERVICES:
    + /pick (some_type) - picks up the object and moves it


"""

from enum import auto, Enum

from geometry_msgs.msg import Pose, Quaternion
from apex_putter.MotionPlanningInterface import MotionPlanningInterface
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Empty
import modern_robotics as mr
import apex_putter.transform_operations as to


def quaternion_to_rotation_matrix(q: Quaternion):
    # Extract components of the quaternion
    x = q.x
    y = q.y
    z = q.z
    w = q.w

    R = np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x**2 + y**2)]
    ])

    return R


def rotation_matrix_to_quaternion(R):
    # Compute the trace of the matrix
    T = np.trace(R)

    if T > 0:
        w = np.sqrt(T + 1) / 2
        x = (R[2, 1] - R[1, 2]) / (4 * w)
        y = (R[0, 2] - R[2, 0]) / (4 * w)
        z = (R[1, 0] - R[0, 1]) / (4 * w)
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            x = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) / 2
            w = (R[2, 1] - R[1, 2]) / (4 * x)
            y = (R[0, 1] + R[1, 0]) / (4 * x)
            z = (R[0, 2] + R[2, 0]) / (4 * x)
        elif R[1, 1] > R[2, 2]:
            y = np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) / 2
            w = (R[0, 2] - R[2, 0]) / (4 * y)
            x = (R[0, 1] + R[1, 0]) / (4 * y)
            z = (R[1, 2] + R[2, 1]) / (4 * y)
        else:
            z = np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) / 2
            w = (R[1, 0] - R[0, 1]) / (4 * z)
            x = (R[0, 2] + R[2, 0]) / (4 * z)
            y = (R[1, 2] + R[2, 1]) / (4 * z)

    return Quaternion(x=x, y=y, z=z, w=w)


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
        self.state = State.START
        self.MPI = MotionPlanningInterface(self, 'base', 'fer_link8')

        self.manipulator_pos = [
            [-0.017, 0.0524, 0.2793, -2.8274, -0.0698, 2.8798, 1.0995],
            [0.104719, 0.27925, 0.0872, -2.7925, -0.31415, 3.07178, 1.2915],
            [-0.01745, 0.13962, 0.24435, -2.82743, -0.17453, 2.96706, 1.16937],
            [-0.6807, 0.2094, -0.1047, -2.6005, 0.0698, 2.8099, 0.0]
        ]

        self.pose = Pose()
        self.pose.position.x = 0.3481
        self.pose.position.y = 0.3313
        self.pose.position.z = 0.41426
        self.pose.orientation.x = 0.90305
        self.pose.orientation.y = 0.4295
        self.pose.orientation.z = -3.8634e-05
        self.pose.orientation.w = -5.0747e-06

        self.pose1 = Pose()
        self.pose1.position.x = 0.36158
        self.pose1.position.y = 0.199983
        self.pose1.position.z = 0.41427
        self.pose1.orientation.x = 0.90305
        self.pose1.orientation.y = 0.429622
        self.pose1.orientation.z = -3.8634e-05
        self.pose1.orientation.w = -5.0747e-06

        self.pose2 = Pose()
        self.pose2.position.x = 0.3809
        self.pose2.position.y = 0.0127
        self.pose2.position.z = 0.414288
        self.pose2.orientation.x = 0.90305
        self.pose2.orientation.y = 0.4295
        self.pose2.orientation.z = -3.8634e-05
        self.pose2.orientation.w = -5.0747e-06

        self.pose3 = Pose()
        self.pose3.position.x = 0.3976
        self.pose3.position.y = -0.14943
        self.pose3.position.z = 0.414288
        self.pose3.orientation.x = 0.903011
        self.pose3.orientation.y = 0.429618
        self.pose3.orientation.z = -3.8634e-05
        self.pose3.orientation.w = -5.0747e-06

        self.waypoints = [self.pose1, self.pose2, self.pose3]

        self.hand_pos = [
            [0.025, 0.025],
            [0.020, 0.020]
        ]

        self.curr_man = 0
        self.curr_hand = 0

        # Create service
        self.pick = self.create_service(Empty, 'pick', self.pick_callback,
                                        callback_group=MutuallyExclusiveCallbackGroup())
        self.test = self.create_service(
            Empty, 'test', self.test_callback, callback_group=MutuallyExclusiveCallbackGroup())

        self.timer = self.create_timer(1/100, self.timer_callback)

    async def pick_callback(self, request, response):
        """Pick service callback."""
        await self.MPI.move_arm_joints(self.manipulator_pos[0])
        await self.MPI.open_gripper()
        await self.MPI.move_arm_joints(self.manipulator_pos[1])
        await self.MPI.close_gripper(0.020)
        await self.MPI.move_arm_joints(self.manipulator_pos[2])
        await self.MPI.attach_object('object', 'fer_hand')
        await self.MPI.move_arm_joints(self.manipulator_pos[3])
        await self.MPI.open_gripper()
        await self.MPI.detach_object('object', 'fer_hand')
        return response

    async def test_callback(self, request, response):
        """Test."""
        # transform_operations.detected obj pose -> goal pose.
        # await self.MPI.move_arm_pose(goal_pose=self.pose)
        # await self.MPI.move_arm_cartesian(waypoints=self.waypoints)
        pose = Pose()
        pose.position.x = 0.3
        pose.position.y = 0.0
        pose.position.z = 0.01
        await self.align_club_face(pose)

    async def align_club_face(self, ball_pose: Pose, hole_pose: Pose = None):
        ball_radius = 0.0213
        ball_position = ball_pose.position
        ball_orientation = ball_pose.orientation

        club_face = Pose()
        club_face.position.x = 0.013274
        club_face.position.y = 0.047312
        club_face.position.z = 0.58929
        club_face.orientation.x = 0.659466
        club_face.orientation.y = -0.215248
        club_face.orientation.z = 0.719827
        club_face.orientation.w = 0.0249158

        R_ee_club = quaternion_to_rotation_matrix(club_face.orientation)
        p_club = [club_face.position.x,
                  club_face.position.y, club_face.position.z]

        R_ball = quaternion_to_rotation_matrix(ball_orientation)
        p_ball = [ball_position.x, ball_position.y, ball_position.z]

        T_base_ball = mr.RpToTrans(R_ball, p_ball)
        T_ee_club = mr.RpToTrans(R_ee_club, p_club)

        T_base_ee = T_base_ball @ mr.TransInv(T_ee_club)

        R_base_ee, p_ee = mr.TransToRp(T_base_ee)

        ee_pose = Pose()
        ee_pose.position.x = p_ee[0]
        ee_pose.position.y = p_ee[1]
        ee_pose.position.z = p_ee[2]

        ee_pose.orientation = rotation_matrix_to_quaternion(R_base_ee)

        await self.MPI.move_arm_pose(ee_pose)

    async def timer_callback(self):
        """Timer callback."""
        if self.state == State.START:
            # self.scene_parameters = [
            #     {
            #         'id': 'object',
            #         'size': (0.03, 0.03, 0.03),
            #         'position': (0.35, 0.099, 0.015),
            #         'orientation': (0.0, 0.0, 0.0, 1.0),
            #         'frame_id': 'base'
            #     },
            #     {
            #         'id': 'table',
            #         'size': (0.5, 1.0, 0.01),
            #         'position': (0.35, 0.0, -0.005),
            #         'frame_id': 'base'
            #     },
            #     {
            #         'id': 'obstacle',
            #         'size': (0.3, 0.05, 0.2),
            #         'position': (0.25, -0.1, 0.1),
            #         'orientation': (0.0, 0.0, 0.0, 1.0),
            #         'frame_id': 'base'
            #     }
            # ]
            # self.scene_set = False
            # self.scene_future = await self.MPI.load_scene_from_parameters(
            #     self.scene_parameters)
            self.state = State.IDLE
        elif self.state == State.IDLE:
            pass


def main(args=None):
    """Execute the main loop of the node."""
    rclpy.init(args=args)
    my_node = PickNode()
    rclpy.spin(my_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
