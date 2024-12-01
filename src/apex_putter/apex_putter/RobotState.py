"""
The RobotState class manages the robot's state, providing functionality for
tracking joint states, retrieving the end-effector pose, and performing
inverse and forward kinematics.

Subscribers
-----------
    + /<joint_state_topic> (JointState) - Subscribes to the current joint
      states of the robot.

Clients
-------
    + /compute_ik (GetPositionIK) - Service client for computing inverse
      kinematics to find joint configurations for a desired end-effector pose.
    + /compute_fk (GetPositionFK) - Service client for computing forward
      kinematics to find the end-effector pose given joint states.

Dependencies
------------
    + tf2_ros.Buffer - Used to fetch transforms for retrieving the
        end-effector pose.
    + tf2_ros.TransformListener - Listens for TF transforms between
      base and end-effector frames.

Parameters
----------
    node (Node): The running ROS node used for subscriptions, clients,
                 and logging.
    base_frame (str): The reference frame for the robot's base.
    end_effector_frame (str): The frame of the robot's end effector.
    joint_state_topic (str): The topic name for subscribing to joint states.
    group_name (str): The name of the planning group for the robot.

Methods
-------
    + joint_state_callback(msg: JointState) -
        Updates the internal joint state based on received messages.
    + get_current_joint_state() -> JointState -
        Retrieves the most recent joint state.
    + get_current_end_effector_pose() -> PoseStamped -
        Retrieves the most up-to-date end-effector pose using TF transforms.
    + compute_inverse_kinematics(pose: Pose) -> GetPositionIK.Response -
        Computes the joint state required to achieve a given end-effector pose.
    + compute_forward_kinematics(joint_state: JointState) ->
        GetPositionFK.Response -
        Computes the end-effector pose from a given joint state.

"""


import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import Header
from moveit_msgs.msg import RobotState as MoveItRobotState, MoveItErrorCodes


class RobotState:
    def __init__(self, node: Node, base_frame: str, end_effector_frame: str,
                 joint_state_topic: str, group_name: str):
        self.node = node
        self.base_frame = base_frame
        self.end_effector_frame = end_effector_frame
        self.group_name = group_name

        # Initialize the TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        # Initialize the joint state
        self.joint_state = JointState()

        # Subscribe to the joint state topic
        self.joint_state_subscription = self.node.create_subscription(
            JointState, joint_state_topic, self.joint_state_callback, 10)

        # Create an IK service client
        self.ik_client = self.node.create_client(
            GetPositionIK, 'compute_ik',
            callback_group=MutuallyExclusiveCallbackGroup())

        # Create an FK service client
        self.fk_client = self.node.create_client(
            GetPositionFK, 'compute_fk',
            callback_group=MutuallyExclusiveCallbackGroup())

    def joint_state_callback(self, msg):
        self.joint_state = msg

    def get_current_joint_state(self):
        return self.joint_state

    async def get_current_end_effector_pose(self):
        """
        Retrieves the most up-to-date end-effector pose from the robot.

        Returns:
            PoseStamped: The current end-effector pose.
        """
        try:
            transform = await self.tf_buffer.lookup_transform_async(
                self.base_frame, self.end_effector_frame, rclpy.time.Time())
            pose = PoseStamped()
            pose.header = transform.header
            pose.pose.position = transform.transform.translation
            pose.pose.orientation = transform.transform.rotation
            return pose
        except Exception as e:
            self.node.get_logger().error(
                f'Failed to get end-effector pose: {e}')
            return None

    async def compute_inverse_kinematics(self, pose: Pose = None):
        if pose is None:
            self.node.get_logger().error(
                'Cannot compute IK without a valid pose.')
            return None

        ik_request = GetPositionIK.Request()
        ik_request.ik_request.group_name = self.group_name
        ik_request.ik_request.pose_stamped = PoseStamped(
            header=Header(
                frame_id=self.base_frame,
                stamp=self.node.get_clock().now().to_msg()
            ),
            pose=pose
        )
        ik_request.ik_request.ik_link_name = self.end_effector_frame
        ik_request.ik_request.robot_state.joint_state = \
            self.get_current_joint_state()

        # Call the service asynchronously and await the result
        ik_response_future = self.ik_client.call_async(ik_request)
        await ik_response_future
        return ik_response_future.result()

    async def compute_forward_kinematics(self, joint_state: JointState = None):
        if joint_state is None:
            joint_state = self.get_current_joint_state()

        fk_request = GetPositionFK.Request()
        fk_request.header.frame_id = self.base_frame
        fk_request.fk_link_names = [self.end_effector_frame]
        fk_request.robot_state.joint_state = joint_state

        # Call the service asynchronously and await the result
        fk_response_future = self.fk_client.call_async(fk_request)
        await fk_response_future
        return fk_response_future.result()
