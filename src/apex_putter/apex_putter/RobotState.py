"""
RobotState class tracks the robot state.

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


from geometry_msgs.msg import Point, Pose, PoseStamped
from moveit_msgs.msg import RobotState as RobotStateObj
from moveit_msgs.srv import GetPositionFK, GetPositionIK
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener


class RobotState:
    def __init__(self, node: Node, base_frame: str, end_effector_frame: str):
        self.node = node
        self.base_frame = base_frame
        self.end_effector_frame = end_effector_frame

        # Initialize the TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        # Initialize the joint state
        self.joint_state = JointState()

        # Subscribe to the joint state topic
        self.joint_state_subscription = self.node.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        # Create an IK service client
        self.ik_client = self.node.create_client(
            GetPositionIK, 'compute_ik')

        # Create an FK service client
        self.fk_client = self.node.create_client(
            GetPositionFK, 'compute_fk')

    def joint_state_callback(self, msg):
        """Define a callback function for joint states."""
        self.joint_state = msg

    def get_robot_state(self):
        """Retrieve the current joint state."""
        robot_state = RobotStateObj()
        robot_state.joint_state = self.joint_state
        return robot_state

    async def get_transform(self, base_frame, end_frame):
        try:
            transform = await self.tf_buffer.lookup_transform_async(
                base_frame, end_frame, rclpy.time.Time())
            pose = PoseStamped()
            pose.header = transform.header
            pose.pose.position = transform.transform.translation
            pose.pose.orientation = transform.transform.rotation
            return pose
        except Exception as e:
            self.node.get_logger().error(
                f'Failed to get end-effector pose: {e}')
            return None

    async def get_current_end_effector_pose(self):
        """
        Retrieve the most up-to-date end-effector pose from the robot.

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

    async def compute_inverse_kinematics(self, group_name, curr_pose: Pose = None):
        if curr_pose is None:
            self.node.get_logger().error(
                'Cannot compute IK without a valid pose.')
            return None

        ik_request = GetPositionIK.Request()
        ik_request.ik_request.group_name = group_name
        ik_request.ik_request.pose_stamped = PoseStamped()
        ik_request.ik_request.pose_stamped.header = Header(
            frame_id=self.base_frame,
            stamp=self.node.get_clock().now().to_msg()
        )
        ik_request.ik_request.pose_stamped.pose = Pose()
        ik_request.ik_request.pose_stamped.pose.position = Point()
        ik_request.ik_request.pose_stamped.pose.position.x = curr_pose.position.x
        ik_request.ik_request.pose_stamped.pose.position.y = curr_pose.position.y
        ik_request.ik_request.pose_stamped.pose.position.z = curr_pose.position.z
        ik_request.ik_request.pose_stamped.pose.orientation = curr_pose.orientation

        ik_request.ik_request.ik_link_name = self.end_effector_frame
        ik_request.ik_request.robot_state.joint_state = \
            self.joint_state

        # Call the service asynchronously and await the result
        ik_response = await self.ik_client.call_async(ik_request)
        return ik_response.solution

    async def compute_forward_kinematics(self, joint_state: JointState = None):
        if joint_state is None:
            joint_state = self.joint_state

        fk_request = GetPositionFK.Request()
        fk_request.header.frame_id = self.base_frame
        fk_request.fk_link_names = [self.end_effector_frame]
        fk_request.robot_state.joint_state = joint_state

        # Call the service asynchronously and await the result
        fk_response = await self.fk_client.call_async(fk_request)
        return fk_response.result()
