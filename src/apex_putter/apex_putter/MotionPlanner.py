"""
The MotionPlanner class enables user to plan path, execute trajectory, \
    and utilize the path.

Publishers
----------
    None.

Subscribers
-----------
    + /<joint_state_topic> (JointState) - Subscribes to the joint state \
        of the robot.

Clients
-------
    + /move_action (MoveGroup) - Action client to communicate with the MoveIt.
    + /compute_cartesian_path (GetCartesianPath) - \
        Service client to compute Cartesian paths.
    + /execute_trajectory (ExecuteTrajectory) - \
        Action client to execute planned trajectories.

Parameters
----------
    + base_frame (str) - The reference frame for motion planning.
    + end_effector_frame (str) - The frame of the robot's end effector.
    + joint_state_topic (str) - The topic name for subscribing to joint states.
    + group_name (str) - The name of the planning group for the robot.
    + workspace_min (Vector3) - Minimum bounds for the planning workspace.
    + workspace_max (Vector3) - Maximum bounds for the planning workspace.

Methods
-------
    + plan_joint_async(joint_name, joint_values, ...) - \
        Plans a path to a target joint configuration asynchronously.
    + plan_joint_space(joint_name, joint_values, ...) - \
        Synchronous wrapper for `plan_joint_async`.
    + plan_pose_async(goal_position, goal_orientation, ...) - \
        Plans a path to a target pose asynchronously.
    + plan_work_space(goal_position, goal_orientation, ...) - \
        Synchronous wrapper for `plan_pose_async`.
    + plan_cartesian_path_async(waypoints, ...) - \
        Plans a Cartesian path through a series of waypoints asynchronously.
    + plan_cartesian_path(waypoints, ...) - \
        Synchronous wrapper for `plan_cartesian_path_async`.
    + get_named_config(named_configuration) - \
        Retrieves a stored named configuration (joint names and values).
    + set_named_config(named_configuration, joint_names, joint_values) - \
        Stores a named configuration.
    + plan_to_named_config_async(named_configuration, ...) - \
        Plans a path to a named configuration asynchronously.
    + plan_to_named_config(named_configuration, ...) - \
        Synchronous wrapper for `plan_to_named_config_async`.
    + save_trajectory(name, trajectory) - \
        Saves a planned trajectory under a specified name.
    + get_saved_trajectory(name) - \
        Retrieves a saved trajectory by name.
    + execute_trajectory_async(trajectory) - \
        Executes a planned trajectory asynchronously.
    + execute_trajectory(trajectory) - \
        Synchronous wrapper for `execute_trajectory_async`.

"""

from geometry_msgs.msg import Point, Pose, Vector3

from moveit_msgs.action import ExecuteTrajectory, MoveGroup

from moveit_msgs.msg import BoundingVolume, Constraints, JointConstraint, \
    MotionPlanRequest, MoveItErrorCodes, OrientationConstraint, \
    PlanningOptions, PositionConstraint, \
    RobotState, WorkspaceParameters

from moveit_msgs.srv import GetCartesianPath

from rclpy.action import ActionClient

from rclpy.node import Node

from shape_msgs.msg import SolidPrimitive

from std_msgs.msg import Header
from tf2_ros.buffer import Buffer

from tf2_ros.transform_listener import TransformListener


class MotionPlanner():

    def __init__(self, node: Node, base_frame: str, end_effector_frame: str):
        """
        Initialize the MotionPlanner class.

        Args:
        ----
            node (Node): The running ROS node used to interface with ROS.
            base_frame (str): The body frame of the robot.
            end_effector_frame (str): The frame of the end effector.
            joint_state_topic (str): The target joint state topic.
            group_name (str): Name of planning group.

        """
        self.node = node

        # MoveGroup action client
        self.move_group_action_client = ActionClient(
            self.node, MoveGroup, '/move_action')

        self.cartesian_path_client = self.node.create_client(
            GetCartesianPath, 'compute_cartesian_path')

        self.execute_trajectory_client = ActionClient(
            self.node, ExecuteTrajectory, 'execute_trajectory')

        self.base_frame = base_frame
        self.end_effector_frame = end_effector_frame

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        # Franka workspace (or just default 111?)
        self.workspace_min = Vector3(x=-0.855, y=-0.855, z=0.0)
        self.workspace_max = Vector3(x=0.855, y=0.855, z=1.19)

        # Named configurations
        self.configurations = {
            # Default home configuration from the franka documentation
            'home': (
                ['fer_joint1', 'fer_joint2', 'fer_joint3', 'fer_joint4',
                 'fer_joint5', 'fer_joint6', 'fer_joint7'],
                [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0]
            )
        }

        # Saved trajectories
        self.saved_trajectories = {}

    async def plan_joint_async(self, joint_name, joint_values,
                               group_name,
                               start_state: RobotState,
                               max_velocity_scaling_factor=0.1,
                               max_acceleration_scaling_factor=0.1,
                               execute: bool = False):
        """
        Plan a path from any valid starting joint configuration to any \
            valid goal joint configuration.

        Args
        ----
            joint_name: A list of joint names.
            joint_values: A list of joint positions to move to.
            start_state: The starting robot state. Defaults to None. \
                Uses current state if None.
            max_velocity_scaling_factor: Velocity scaling factor. \
                Defaults to 0.1.
            max_acceleration_scaling_factor: Acceleration scaling factor. \
                Defaults to 0.1.
            execute: Whether to execute the trajectory. Defaults to False.

        Returns
        -------
            the planned trajectory

        """

        # define goal joint constriants
        joint_constraints = []
        for i in range(len(joint_name)):
            if group_name == 'fer_manipulator':
                constraint_i = JointConstraint(
                    joint_name=joint_name[i],
                    position=joint_values[i],
                    tolerance_above=0.0001,
                    tolerance_below=0.0001
                )
                joint_constraints.append(constraint_i)
            elif group_name == 'hand':
                constraint_i = JointConstraint(
                    joint_name=joint_name[i],
                    position=joint_values[i],
                    tolerance_above=0.0001,
                    tolerance_below=0.0001
                )
                joint_constraints.append(constraint_i)

        self.node.get_logger().info(
            f'Goal joint Constraint: {joint_constraints}')

        # wrap into GoalConstraints
        goal_constraints = Constraints(joint_constraints=joint_constraints)

        self.node.get_logger().info(f'{start_state}')

        motion_plan_request = MotionPlanRequest()
        if group_name == 'fer_manipulator':
            # moveit plan request
            motion_plan_request = MotionPlanRequest(
                workspace_parameters=WorkspaceParameters(
                    header=Header(
                        stamp=self.node.get_clock().now().to_msg(),
                        frame_id=self.base_frame
                    ),
                    min_corner=self.workspace_min,
                    max_corner=self.workspace_max
                ),
                # start_state=start_state,
                goal_constraints=[goal_constraints],
                group_name=group_name,
                allowed_planning_time=20.0,
                num_planning_attempts=20,
                max_velocity_scaling_factor=max_velocity_scaling_factor,
                max_acceleration_scaling_factor=max_acceleration_scaling_factor,
            )
        elif group_name == 'hand':
            # moveit plan request
            motion_plan_request = MotionPlanRequest(
                workspace_parameters=WorkspaceParameters(
                    header=Header(
                        stamp=self.node.get_clock().now().to_msg(),
                        frame_id=self.base_frame
                    ),
                ),
                goal_constraints=[goal_constraints],
                group_name=group_name,
                allowed_planning_time=5.0,
                num_planning_attempts=5,
            )

        self.node.get_logger().info('Sending goal.')
        # --------------------   Begin Citation [1]   --------------------
        goal = MoveGroup.Goal(
            request=motion_plan_request,
            planning_options=PlanningOptions(
                plan_only=(not execute)
            ),
        )
        self.node.get_logger().info('Goal Sent.')

        goal_handle = await self.move_group_action_client.send_goal_async(goal)
        self.node.get_logger().info('Goal handle received.')
        result_response = await goal_handle.get_result_async()
        self.node.get_logger().info('Result received.')
        result = result_response.result
        # --------------------  End Citation [1]   --------------------
        error_code = result.error_code

        # translate error code in MoveItErrorCodes
        if error_code.val != MoveItErrorCodes.SUCCESS:
            self.node.get_logger().error(f'Planning failed with error code: {
                error_code.val}, message: {error_code.message}, source: {error_code.source}')
            return None

        if execute:
            return result.executed_trajectory
        else:
            self.node.get_logger().info(f'Planned trajectory: \
            {result.planned_trajectory}')
            return result.planned_trajectory

    async def plan_pose_async(self,
                              group_name,
                              start_state: RobotState,
                              goal_position=None, goal_orientation=None,
                              max_velocity_scaling_factor=0.1,
                              max_acceleration_scaling_factor=0.1,
                              execute=False):
        """
        Plans and optionally executes robot motion to reach target pose in \
            workspace coordinates.

        Args:
        ----
            goal_position (Point): Target position in base frame. \
                None for orientation-only control.
            goal_orientation (Quaternion): Target orientation in base frame. \
                None for position-only control.
            start_pose (Pose, optional): Starting pose. \
                Uses current state if None.
            max_velocity_scaling_factor (float, optional): \
                Velocity scaling (0-1). Defaults to 0.1.
            max_acceleration_scaling_factor (float, optional): \
                Acceleration scaling (0-1). Defaults to 0.1.
            execute (bool, optional): Execute trajectory if True. \
                Defaults to False.

        Return:
        ------
            RobotTrajectory: Executed trajectory if execute=True, \
                otherwise planned trajectory.

        """

        # define goal constraints
        position_constraints = []
        orientation_constraints = []

        # Wrap into point class if provided a list
        if type(goal_position) is not Point:
            self.node.get_logger().info(f'None point type goal position provided: {
                type(goal_position)} with val: {goal_position}.')
            goal_position = Point(
                x=goal_position[0], y=goal_position[1], z=goal_position[2])
            self.node.get_logger().info(
                f'Converted to point type goal position: {goal_position}')

        # position constraint
        if goal_position is not None:
            position_constraint = PositionConstraint(
                header=Header(
                    frame_id=self.base_frame,
                    stamp=self.node.get_clock().now().to_msg()
                ),
                target_point_offset=Vector3(),
                link_name=self.end_effector_frame,
                constraint_region=BoundingVolume(
                    primitives=[SolidPrimitive(
                        type=SolidPrimitive.BOX,
                        dimensions=[0.001, 0.001, 0.001]
                    )],
                    primitive_poses=[Pose(position=goal_position)]
                ),
                weight=1.0
            )
            position_constraints.append(position_constraint)
        # orientation constraint
        if goal_orientation is not None:
            orientation_constraint = OrientationConstraint(
                header=Header(
                    frame_id=self.base_frame,
                    stamp=self.node.get_clock().now().to_msg()
                ),
                link_name=self.end_effector_frame,
                orientation=goal_orientation,
                absolute_x_axis_tolerance=0.0001,
                absolute_y_axis_tolerance=0.0001,
                absolute_z_axis_tolerance=0.0001,
                weight=1.0
            )
            orientation_constraints.append(orientation_constraint)

        # wrap into GoalConstraints
        goal_constraints = Constraints(
            position_constraints=position_constraints,
            orientation_constraints=orientation_constraints)

        self.node.get_logger().info(f'{goal_constraints}')
        # motion plan request msg
        motion_plan_request = MotionPlanRequest(
            workspace_parameters=WorkspaceParameters(
                header=Header(
                    stamp=self.node.get_clock().now().to_msg(),
                    frame_id=self.base_frame
                ),
                min_corner=self.workspace_min,
                max_corner=self.workspace_max
            ),
            # start_state=start_state,
            goal_constraints=[goal_constraints],
            group_name=group_name,
            allowed_planning_time=20.0,
            num_planning_attempts=20,
            max_velocity_scaling_factor=max_velocity_scaling_factor,
            max_acceleration_scaling_factor=max_acceleration_scaling_factor,
        )

        goal = MoveGroup.Goal(
            request=motion_plan_request,
            planning_options=PlanningOptions(
                plan_only=(not execute)
            ),
        )

        # boilerplate
        goal_handle = await self.move_group_action_client.send_goal_async(goal)
        result_response = await goal_handle.get_result_async()
        result = result_response.result
        error_code = result.error_code

        # translate error code in MoveItErrorCodes
        if error_code.val != MoveItErrorCodes.SUCCESS:
            self.node.get_logger().error(
                f'Planning failed with error code: {error_code.val}')
            return None
        if execute:
            return result.executed_trajectory
        else:
            return result.planned_trajectory

    async def plan_cartesian_path_async(self, waypoints,
                                        group_name,
                                        start_state,
                                        max_velocity_scaling_factor=0.1,
                                        max_acceleration_scaling_factor=0.1,
                                        avoid_collisions=True):
        """
        Plan a Cartesian path from a starting pose to a series of waypoints.

        Args
        ----
            waypoints (list of Pose): List of waypoints to follow in sequence.
            start_pose (Pose, optional): Starting pose of the robot's \
                end-effector. Uses current pose if None.
            max_velocity_scaling_factor (float, optional): Scaling factor for \
                joint velocity. Defaults to 0.1.
            max_acceleration_scaling_factor (float, optional): Scaling factor \
                for joint acceleration. Defaults to 0.1.
            execute (bool, optional): If True, execute the planned trajectory \
                immediately. Defaults to False.
            avoid_collisions (bool, optional): Whether to avoid collisions \
                during planning. Defaults to True.

        Returns
        -------
            RobotTrajectory: Returns 'executed_trajectory' if execute=True, \
                otherwise 'planned_trajectory'.

        """

        self.node.get_logger().info(f'Waypoint: {waypoints}')

        # Create the request for the GetCartesianPath service
        cartesian_path_request = GetCartesianPath.Request(
            header=Header(
                frame_id=self.base_frame,
                stamp=self.node.get_clock().now().to_msg()
            ),
            start_state=start_state,
            waypoints=waypoints,
            max_step=0.01,
            max_velocity_scaling_factor=max_velocity_scaling_factor,
            max_acceleration_scaling_factor=max_acceleration_scaling_factor,
            group_name=group_name,
            link_name=self.end_effector_frame,
            avoid_collisions=avoid_collisions,
        )

        # Wait for the service and then call it asynchronously
        if not self.cartesian_path_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error(
                'GetCartesianPath service not available.')
            return None

        response = await self.cartesian_path_client.call_async(
            cartesian_path_request)

        # Check response and if not error, return trajectory
        if response.error_code.val == response.error_code.SUCCESS:
            self.node.get_logger().info('Cartesian path planned successfully.')
            return response.solution
        else:
            self.node.get_logger().error(
                f'Failed to plan cartesian path:\
                      Error code {response.error_code.val}')
            return None

    def get_named_config(self, named_configuration):
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
        config = self.configurations.get(named_configuration)
        if config is None:
            self.node.get_logger().error(
                f"Named configuration '{named_configuration}' not found.")
            return None, None

        return config

    def set_named_config(self, named_configuration, joint_names, joint_values):
        """
        Set a named configuration with the given joint names and values.

        Test_joint_values = [0.0, -0.8, 0.2, -1.4, 0.1, 1.0, -0.4]

        Args:
        ----
            named_configuration (str): The name of the target configuration.
            joint_names (list[str]): List of joint names.
            joint_values (list[str]): List of joint values.

        """
        self.node.get_logger().info(
            f"Setting named configuration '{named_configuration}' \
            with: {joint_names} to {joint_values}")
        self.configurations[named_configuration] = (joint_names, joint_values)

    async def plan_to_named_config_async(self, named_configuration,
                                         start_pose=None,
                                         max_velocity_scaling_factor=0.1,
                                         max_acceleration_scaling_factor=0.1,
                                         execute=False):
        """
        Plan a path from any valid starting pose to a named configuration.

        Args
        ----
            named_configuration (str): The name of the target configuration.
            start_pose (Pose, optional): Starting pose of the robot's \
                end-effector. Uses current pose if None.
            execute (bool, optional): Execute the trajectory if True. \
                Defaults to False.

        Returns
        -------
            RobotTrajectory: Executed or planned trajectory.

        """
        # Retrieve joint values for the named configuration
        joint_names, joint_values = self.get_named_config(
            named_configuration)

        if joint_names is None or joint_values is None:
            self.node.get_logger().error(
                f"Named configuration '{named_configuration}' not found.")
            return None

        # Convert start_pose to RobotState using IK
        start_state = None
        if start_pose is not None:
            start_state = await self.robot_state.compute_inverse_kinematics(
                start_pose)
            if start_state is None:
                self.node.get_logger().error('Failed to convert start_pose to \
                                             RobotState using IK.')
                return None

        # Plan to the joint values.
        # Use async call to prevent executor deadlock.
        planned_traj = await self.plan_joint_async(
            joint_names,
            joint_values,
            start_state=start_state,
            max_velocity_scaling_factor=max_velocity_scaling_factor,
            max_acceleration_scaling_factor=max_acceleration_scaling_factor,
            execute=execute
        )

        if planned_traj is None:
            self.node.get_logger().error('Planning failed - no trajectory returned')
            return None
        else:
            self.node.get_logger().info(
                'Planning completed, returning planned or executed trajectory...')
            self.node.get_logger().info(f'Planned trajectory: {planned_traj}')
            return planned_traj

    def save_trajectory(self, name, trajectory):
        """
        Save a trajectory with a specified name.

        Args:
        ----
            name (str): The name to store the trajectory under.
            trajectory (RobotTrajectory): The planned trajectory to save.

        """
        self.saved_trajectories[name] = trajectory
        self.node.get_logger().info(f"Trajectory '{name}' is saved.")

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
        if name in self.saved_trajectories:
            return self.saved_trajectories[name]
        else:
            self.node.get_logger().warning(
                f"Trajectory '{name}' sadly cannot be found.")
            return None

    async def execute_trajectory_async(self, trajectory, controller):
        """
        Execute a given trajectory using the ExecuteTrajectory action.

        Args
        ----
            trajectory (RobotTrajectory): The trajectory to execute.

        Returns
        -------
            bool: True if the trajectory was successfully executed, \
                otherwsie False.

        """
        # Wait until the ExecuteTrajectory action server is available
        if not self.execute_trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'Execute trajectory action server not available')
            return False

        # Create a goal message for the ExecuteTrajectory action
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory
        goal_msg.controller_names = [controller]

        # Send the goal to the action server
        goal_handle = await self.execute_trajectory_client.send_goal_async(
            goal_msg)
        if not goal_handle.accepted:
            self.node.get_logger().error('ExecuteTrajectory goal rejected.')
            return False

        # Wait for the result
        result = await goal_handle.get_result_async()
        if result.result.error_code.val == 1:
            self.node.get_logger().info('Trajectory execution succeeded.')
            return True

        else:
            self.node.get_logger().error('Trajectory execution failed.')
            return False
