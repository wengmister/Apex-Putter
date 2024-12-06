from geometry_msgs.msg import Pose
from apex_putter.MotionPlanner import MotionPlanner
from apex_putter.PlanningSceneClass import PlanningSceneClass
from apex_putter.RobotState import RobotState
from moveit_msgs.msg import RobotState as RobotStateObj
from rclpy.node import Node


class MotionPlanningInterface():

    def __init__(self,
                 node: Node,
                 base_frame: str,
                 end_effector_frame: str):
        self.MotionPlanner = MotionPlanner(node,
                                           base_frame,
                                           end_effector_frame)
        self.RobotState = RobotState(node,
                                     base_frame,
                                     end_effector_frame)
        self.PlanningScene = PlanningSceneClass(node)
        self.node = node

        # Joint names for manipulator
        self.man_joint_names = [
            'fer_joint1', 'fer_joint2', 'fer_joint3', 'fer_joint4',
            'fer_joint5', 'fer_joint6', 'fer_joint7'
        ]
        self.hand_joint_names = [
            'fer_finger_joint1', 'fer_finger_joint2'
        ]

        self.arm_move_group = 'fer_manipulator'
        self.gripper_move_group = 'hand'

        self.arm_controller = 'fer_arm_controller'
        self.gripper_controller = 'fer_gripper'

    # MotionPlanner functions
    async def move_arm_joints(self, joint_values, start_config=None, max_velocity_scaling_factor=0.1, max_acceleration_scaling_factor=0.1):
        """Move arm to desired joint values."""
        if start_config is None:
            start_state = self.RobotState.get_robot_state()
        else:
            start_state = RobotStateObj()
            start_state.joint_state = start_config
        traj = await self.MotionPlanner.plan_joint_async(
            joint_name=self.man_joint_names,
            joint_values=joint_values,
            group_name=self.arm_move_group,
            start_state=start_state,
            max_velocity_scaling_factor=max_velocity_scaling_factor,
            max_acceleration_scaling_factor=max_acceleration_scaling_factor
        )
        self.node.get_logger().info('Execute')
        return await self.MotionPlanner.execute_trajectory_async(
            trajectory=traj,
            controller=self.arm_controller
        )

    async def move_arm_pose(self, goal_pose: Pose, start_pose: Pose = None, max_velocity_scaling_factor=0.1, max_acceleration_scaling_factor=0.1):
        """Move arm to pose for 'fer_link8'."""
        goal_position = goal_pose.position
        goal_orientation = goal_pose.orientation
        if start_pose is None:
            start_state = self.RobotState.get_robot_state()
        else:
            start_state = await self.RobotState.compute_inverse_kinematics(curr_pose=start_pose, group_name=self.arm_move_group)
        traj = await self.MotionPlanner.plan_pose_async(
            group_name=self.arm_move_group,
            start_state=start_state,
            goal_position=goal_position,
            goal_orientation=goal_orientation,
            max_velocity_scaling_factor=max_velocity_scaling_factor,
            max_acceleration_scaling_factor=max_acceleration_scaling_factor
        )
        return await self.MotionPlanner.execute_trajectory_async(
            trajectory=traj,
            controller=self.arm_controller
        )

    async def move_arm_cartesian(self, waypoints, start_pose=None, max_velocity_scaling_factor=0.1, max_acceleration_scaling_factor=0.1, avoid_collisions=True):
        """Move arm using a cartesian path."""
        if start_pose is None:
            start_pose = await self.RobotState.get_current_end_effector_pose()
        start_state = await self.RobotState.compute_inverse_kinematics(curr_pose=start_pose.pose, group_name=self.arm_move_group)
        print("start_state passed.")
        traj = await self.MotionPlanner.plan_cartesian_path_async(
            waypoints=waypoints,
            group_name=self.arm_move_group,
            start_state=start_state,
            max_velocity_scaling_factor=max_velocity_scaling_factor,
            max_acceleration_scaling_factor=max_acceleration_scaling_factor,
            avoid_collisions=avoid_collisions
        )
        print("traj passed.")
        return await self.MotionPlanner.execute_trajectory_async(
            trajectory=traj,
            controller=self.arm_controller
        )

    async def open_gripper(self):
        start_state = self.RobotState.get_robot_state()
        traj = await self.MotionPlanner.plan_joint_async(
            self.hand_joint_names,
            [0.035, 0.035],
            self.gripper_move_group,
            start_state=start_state,
            max_velocity_scaling_factor=0.01,
            max_acceleration_scaling_factor=0.01
        )
        return await self.MotionPlanner.execute_trajectory_async(
            trajectory=traj,
            controller=self.gripper_controller
        )

    async def close_gripper(self, width):
        start_state = self.RobotState.get_robot_state()
        traj = await self.MotionPlanner.plan_joint_async(
            self.hand_joint_names,
            [width, width],
            self.gripper_move_group,
            start_state=start_state,
            max_velocity_scaling_factor=0.01,
            max_acceleration_scaling_factor=0.01
        )
        return await self.MotionPlanner.execute_trajectory_async(
            trajectory=traj,
            controller=self.gripper_controller
        )

    def save_trajectory(self, name, trajectory):
        """
        Save a trajectory with a specified name.

        Args:
        ----
            name (str): The name to store the trajectory under.
            trajectory (RobotTrajectory): The planned trajectory to save.

        """
        self.MotionPlanner.save_trajectory(name, trajectory)

    def get_saved_trajectory(self, name):
        """
        Get a saved trajectory by name.

        Args
        ----
            name (str): The name of the trajectory to retrieve.

        Returns
        -------
            RobotTrajectory: The retrieved trajectory, or None if not found.

        """
        return self.MotionPlanner.get_saved_trajectory(name)

    def get_named_configuration(self, named_configuration):
        """
        Retrieve the joint names and values for a named configuration.

        Args
        ----
            named_configuration (str): The name of the target configuration.
                Joint 1 (Shoulder Pan)
                Joint 2 (Shoulder Lift)
                Joint 3 (Elbow)
                Joint 4 (Wrist 1)
                Joint 5 (Wrist 2)
                Joint 6 (Wrist 3)
                Joint 7 (Wrist 4)

        Returns
        -------
            tuple: (joint_names, joint_values) for the named configuration.

        """
        return self.MotionPlanner.get_named_config(named_configuration)

    def set_named_configuration(self, named_configuration, joint_values):
        """
        Set a named configuration with the given joint names and values.

        Test_joint_values = [0.0, -0.8, 0.2, -1.4, 0.1, 1.0, -0.4]

        Args:
        ----
            named_configuration (str): The name of the target configuration.
            joint_names (list[str]): List of joint names.
            joint_values (list[str]): List of joint values.

        """
        self.MotionPlanner.set_named_config(named_configuration, joint_values)

    async def plan_to_named_configuration(self, named_configuration, start_pose,
                                          max_velocity_scaling_factor,
                                          max_acceleration_scaling_factor, execute):
        """
        Wrap plan_to_named_configuration_async synchronously.

        Args
        ----
            named_configuration (str): The name of the target configuration.
            start_pose (Pose, optional): Starting pose of the robot's \
                end-effector. Uses current pose if None.
            execute (bool, optional): Execute the trajectory if True. \
                Defaults to False.

        Returns
        -------
            Future: Future containing the RobotTrajectory when completed.

        """
        return await self.MotionPlanner.plan_to_named_config_async(
            named_configuration,
            start_pose,
            max_velocity_scaling_factor,
            max_acceleration_scaling_factor,
            execute
        )

    # PlanningScene functions
    async def add_box(self, box_id, size, position, orientation=(0.0, 0.0, 0.0, 1.0), frame_id='base'):
        return await self.PlanningScene.add_box_async(
            box_id, size, position, orientation, frame_id
        )

    async def remove_box(self, box_id, frame_id='base'):
        return await self.PlanningScene.remove_box_async(
            box_id,
            frame_id
        )

    async def attach_object(self, object_id, link_name):
        return await self.PlanningScene.attach_object_async(
            object_id,
            link_name
        )

    async def detach_object(self, object_id, link_name):
        return await self.PlanningScene.detach_object_async(
            object_id,
            link_name
        )

    async def load_scene_from_parameters(self, parameters):
        return await self.PlanningScene.load_scene_from_parameters_async(
            parameters
        )