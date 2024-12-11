"""
Picks up and moves an object in Gazebo.

SERVICES:
    + /pick (some_type) - picks up the object and moves it


"""

from enum import auto, Enum

from geometry_msgs.msg import Pose, Quaternion, TransformStamped
from apex_putter.MotionPlanningInterface import MotionPlanningInterface
import apex_putter.transform_operations as transOps
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Empty
import modern_robotics as mr
import apex_putter.transform_operations as to
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


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


def quaternion_from_euler(ai, aj, ak):
    """
    Convert from euler angles of to a quaternion.

    Args:
    ----
    ai: roll
    aj: pitch
    ak: yaw

    Return:
    ------
    A Quaternion corresponding to the rotation

    """
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = np.cos(ai)
    si = np.sin(ai)
    cj = np.cos(aj)
    sj = np.sin(aj)
    ck = np.cos(ak)
    sk = np.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


def quaternion_difference(q1: Quaternion, q2: Quaternion):
    q1 = np.array([q1.x, q1.y, q1.z, q1.w])
    q2 = np.array([q2.x, q2.y, q2.z, q2.w])

    q_star = np.array([-q2[0], -q2[1], -q2[2], q2[3]])
    q2_mag = np.linalg.norm(q2)
    q2_inv = q_star / q2_mag

    w = q1[3] * q2_inv[3] - q1[0] * q2_inv[0] - \
        q1[1] * q2_inv[1] - q1[2] * q2_inv[2]
    x = q1[3] * q2_inv[0] + q1[0] * q2_inv[3] + \
        q1[1] * q2_inv[2] - q1[2] * q2_inv[1]
    y = q1[3] * q2_inv[1] - q1[0] * q2_inv[2] + \
        q1[1] * q2_inv[3] + q1[2] * q2_inv[0]
    z = q1[3] * q2_inv[2] + q1[0] * q2_inv[1] - \
        q1[1] * q2_inv[0] + q1[2] * q2_inv[3]

    q_diff = np.array([x, y, z, w])

    return q_diff


def rotate_quaternion(q1: Quaternion, q2: Quaternion):
    q1 = np.array([q1.x, q1.y, q1.z, q1.w])
    q2 = np.array([q2.x, q2.y, q2.z, q2.w])


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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'fer_link8'
        t.child_frame_id = 'club_face'

        t.transform.translation.x = float(0.02)
        t.transform.translation.y = float(0.0)
        t.transform.translation.z = float(0.52)
        t.transform.rotation.x = 0.0  # 0.659466
        t.transform.rotation.y = 0.0  # -0.215248
        t.transform.rotation.z = 0.0  # 0.719827
        t.transform.rotation.w = 1.0  # 0.0249158

        self.tf_static_broadcaster.sendTransform(t)

        # Create service
        self.ready_srv = self.create_service(
            Empty, 'ready', self.ready_callback, callback_group=MutuallyExclusiveCallbackGroup())
        self.home_srv = self.create_service(
            Empty, 'home_robot', self.home_callback, callback_group=MutuallyExclusiveCallbackGroup())
        self.putt_srv = self.create_service(
            Empty, 'putt', self.putt_callback, callback_group=MutuallyExclusiveCallbackGroup())
        self.swing_srv = self.create_service(
            Empty, 'swing', self.swing_callback, callback_group=MutuallyExclusiveCallbackGroup())
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

    async def home_callback(self, request, response):
        """Make the robot go to home pose"""
        self.get_logger().info("Home requested.")
        await self.MPI.move_arm_joints(joint_values=[-0.4, -0.4, 0.0, -1.6, 0.0, 1.57, 0.0], max_velocity_scaling_factor=0.2, max_acceleration_scaling_factor=0.2)
        ball_pose = Pose()
        ball_pose.position.x = 0.3
        ball_pose.position.y = 0.0
        ball_pose.position.z = 0.021
        hole_pose = Pose()
        hole_pose.position.x = 0.2
        hole_pose.position.y = 0.4
        hole_pose.position.z = 0.015
        self.goal_club_tf(ball_pose, hole_pose)
        self.goal_ee_tf()
        return response

    async def ready_callback(self, request, response):
        """Prepare the robot for putting"""
        self.get_logger().info("Ready requested.")
        self.get_logger().info("=============================================================")

        # Look up the ideal ee transform first
        ideal_pose = await self.MPI.get_transform('base', 'goal_ee')
        self.get_logger().info("Ready done.")
        self.get_logger().info("=============================================================")
        # self.get_logger().info(ideal_pose)
        # ball_tf = await self.MPI.get_transform('base', 'ball')
        # await self.MPI.add_box('ball', (0.042, 0.042, 0.042), (ball_tf.pose.position.x, ball_tf.pose.position.y, ball_tf.pose.position.z))
        await self.MPI.move_arm_pose(ideal_pose.pose, max_velocity_scaling_factor=0.2, max_acceleration_scaling_factor=0.2)
        # await self.MPI.remove_box('ball')
        self.get_logger().info("Ready done.")
        self.get_logger().info("=============================================================")
        return response

    async def putt_callback(self, request, response):
        club_face_tf = await self.MPI.get_transform('base', 'club_face')
        hole_tf = await self.MPI.get_transform('base', 'hole')
        ee_tf = await self.MPI.get_transform('base', 'fer_link8')
        ee_pose = ee_tf.pose
        hole_pose = hole_tf.pose

        club_face_pos = club_face_tf.pose.position
        hole_pos = hole_pose.position

        traj_vec = np.array([hole_pos.x - club_face_pos.x,
                             hole_pos.y - club_face_pos.y])
        traj_mag = np.linalg.norm(traj_vec)
        traj_norm = traj_vec / traj_mag

        waypoints = []
        for i in range(5):
            pose = Pose()
            pose.position.x = ee_pose.position.x - i*0.05*traj_norm[0]
            pose.position.y = ee_pose.position.y - i*0.05*traj_norm[1]
            pose.position.z = ee_pose.position.z
            pose.orientation = ee_pose.orientation
            waypoints.append(pose)

        for i in range(4, -1):
            pose = Pose()
            pose.position.x = ee_pose.position.x - i*0.05*traj_norm[0]
            pose.position.y = ee_pose.position.y - i*0.05*traj_norm[1]
            pose.position.z = ee_pose.position.z
            pose.orientation = ee_pose.orientation
            waypoints.append(pose)

        for i in range(5):
            pose = Pose()
            pose.position.x = ee_pose.position.x + i*0.05*traj_norm[0]
            pose.position.y = ee_pose.position.y + i*0.05*traj_norm[1]
            pose.position.z = ee_pose.position.z
            pose.orientation = ee_pose.orientation
            waypoints.append(pose)

        await self.MPI.move_arm_cartesian(waypoints)
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

    async def test_callback(self, request, response):
        """Test."""
        # transform_operations.detected obj pose -> goal pose.
        # await self.MPI.move_arm_pose(goal_pose=self.pose)
        # await self.MPI.move_arm_cartesian(waypoints=self.waypoints)
        club_face_tf = await self.MPI.get_transform('base', 'club_face')
        hole_tf = await self.MPI.get_transform('base', 'hole')
        ee_tf = await self.MPI.get_transform('base', 'fer_link8')
        ee_pose = ee_tf.pose
        hole_pose = hole_tf.pose

        club_face_pos = club_face_tf.pose.position
        hole_pos = hole_pose.position

        traj_vec = np.array([hole_pos.x - club_face_pos.x,
                             hole_pos.y - club_face_pos.y])
        traj_mag = np.linalg.norm(traj_vec)
        traj_norm = traj_vec / traj_mag

        waypoints = []
        for i in range(5):
            pose = Pose()
            pose.position.x = ee_pose.position.x - i*0.05*traj_norm[0]
            pose.position.y = ee_pose.position.y - i*0.05*traj_norm[1]
            pose.position.z = ee_pose.position.z
            pose.orientation = ee_pose.orientation
            waypoints.append(pose)

        for i in range(4, -1):
            pose = Pose()
            pose.position.x = ee_pose.position.x - i*0.05*traj_norm[0]
            pose.position.y = ee_pose.position.y - i*0.05*traj_norm[1]
            pose.position.z = ee_pose.position.z
            pose.orientation = ee_pose.orientation
            waypoints.append(pose)

        for i in range(5):
            pose = Pose()
            pose.position.x = ee_pose.position.x + i*0.05*traj_norm[0]
            pose.position.y = ee_pose.position.y + i*0.05*traj_norm[1]
            pose.position.z = ee_pose.position.z
            pose.orientation = ee_pose.orientation
            waypoints.append(pose)

        await self.MPI.move_arm_cartesian(waypoints)
        return response

    def goal_club_tf(self, ball_pose: Pose, hole_pose: Pose = None):
        radius = 0.045
        ball_pos = ball_pose.position
        hole_pos = hole_pose.position
        ball_hole_vec = np.array(
            [hole_pos.x - ball_pos.x, hole_pos.y - ball_pos.y])
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

        t.transform.translation.x = ball_pos.x - \
            0.05 * ball_hole_unit[0]
        t.transform.translation.y = ball_pos.y - \
            0.05 * ball_hole_unit[1]
        t.transform.translation.z = ball_pos.z
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

    async def timer_callback(self):
        """Timer callback."""
        if self.state == State.START:
            self.scene_parameters = [
                {
                    'id': 'hole',
                    'size': (0.03, 0.03, 0.03),
                    'position': (0.2, 0.4, 0.015),
                    'orientation': (0.0, 0.0, 0.0, 1.0),
                    'frame_id': 'base'
                },
                {
                    'id': 'table',
                    'size': (0.5, 1.0, 0.01),
                    'position': (0.35, 0.0, -0.005),
                    'frame_id': 'base'
                },
                {
                    'id': 'ball',
                    'size': (0.042, 0.042, 0.042),
                    'position': (0.3, 0.0, 0.021),
                    'orientation': (0.0, 0.0, 0.0, 1.0),
                    'frame_id': 'base'
                }
            ]
            t = TransformStamped()

            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base'
            t.child_frame_id = 'hole'

            t.transform.translation.x = float(0.2)
            t.transform.translation.y = float(0.4)
            t.transform.translation.z = float(0.015)
            t.transform.rotation.x = 0.0  # 0.659466
            t.transform.rotation.y = 0.0  # -0.215248
            t.transform.rotation.z = 0.0  # 0.719827
            t.transform.rotation.w = 1.0  # 0.0249158

            self.tf_static_broadcaster.sendTransform(t)

            t = TransformStamped()

            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base'
            t.child_frame_id = 'ball'

            t.transform.translation.x = float(0.3)
            t.transform.translation.y = float(0.0)
            t.transform.translation.z = float(0.021)
            t.transform.rotation.x = 0.0  # 0.659466
            t.transform.rotation.y = 0.0  # -0.215248
            t.transform.rotation.z = 0.0  # 0.719827
            t.transform.rotation.w = 1.0  # 0.0249158

            self.tf_static_broadcaster.sendTransform(t)

            self.scene_set = False
            self.scene_future = await self.MPI.load_scene_from_parameters(
                self.scene_parameters)

            ball_pose = Pose()
            ball_pose.position.x = 0.3
            ball_pose.position.y = 0.0
            ball_pose.position.z = 0.021
            hole_pose = Pose()
            hole_pose.position.x = 0.2
            hole_pose.position.y = 0.4
            hole_pose.position.z = 0.015
            # await self.goal_club_tf(ball_pose, hole_pose)
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
