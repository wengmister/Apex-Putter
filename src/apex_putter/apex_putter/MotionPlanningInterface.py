import rclpy
from rclpy.node import Node
from apex_putter.MotionPlanner import MotionPlanner
from apex_putter.RobotState import RobotState
from apex_putter.PlanningSceneClass import PlanningSceneClass


class MotionPlanningInterface():

    def __init__(self, node: Node, base_frame: str, end_effector_frame: str, joint_state_topic: str, group_name: str, controller: str):
        self.MotionPlanner = MotionPlanner(
            node, base_frame, end_effector_frame, joint_state_topic, group_name, controller)
        self.RobotState = RobotState(
            node, base_frame, end_effector_frame, joint_state_topic, group_name)
        self.PlanningScene = PlanningSceneClass(node)

    # MotionPlanner functions
    def plan_joint_space(self, joint_name, joint_values,
                         robot_state,
                         max_velocity_scaling_factor=0.1,
                         max_acceleration_scaling_factor=0.1,
                         execute=False):
        return self.MotionPlanner.plan_joint_space(joint_name, joint_values, robot_state, max_velocity_scaling_factor, max_acceleration_scaling_factor, execute)

    def plan_work_space(self, goal_position=None, goal_orientation=None, start_pose=None,
                        max_velocity_scaling_factor=0.1,
                        max_acceleration_scaling_factor=0.1,
                        execute=False):
        return self.MotionPlanner.plan_work_space(goal_position, goal_orientation, start_pose,
                                                  max_velocity_scaling_factor, max_acceleration_scaling_factor, execute)

    def plan_cartesian_path(self, waypoints, start_pose=None,
                            max_velocity_scaling_factor=0.1,
                            max_acceleration_scaling_factor=0.1,
                            avoid_collisions=True):
        return self.MotionPlanner.plan_cartesian_path(waypoints, start_pose, max_velocity_scaling_factor,
                                                      max_acceleration_scaling_factor, avoid_collisions)

    def execute_trajectory(self, trajectory):
        return self.MotionPlanner.execute_trajectory(trajectory)

    def save_trajectory(self, name, trajectory):
        self.MotionPlanner.save_trajectory(name, trajectory)

    def get_saved_trajectory(self, name):
        return self.MotionPlanner.get_saved_trajectory(name)

    def get_named_configuration(self, named_configuration):
        return self.MotionPlanner.get_named_config(named_configuration)

    def set_named_configuration(self, named_configuration, joint_values):
        self.MotionPlanner.set_named_config(named_configuration, joint_values)

    def plan_to_named_configuration(self, named_configuration, start_pose, max_velocity_scaling_factor, max_acceleration_scaling_factor, execute):
        return self.MotionPlanner.plan_to_named_config(named_configuration, start_pose, max_velocity_scaling_factor, max_acceleration_scaling_factor, execute)
    
    def plan_swing_trajectory(self, ball_position):
        return self.MotionPlanner.plan_swing_trajectory(ball_position)

    # RobotState functions
    async def get_current_end_effector_pose(self):
        return await self.RobotState.get_current_end_effector_pose()

    async def compute_forward_kinematics(self, joint_state):
        return await self.RobotState.compute_forward_kinematics(joint_state)

    async def compute_inverse_kinematics(self, pose):
        return await self.RobotState.robot_state.compute_inverse_kinematics(pose)

    # PlanningScene functions
    def add_box(self, box_id, size, position, orientation=(0.0, 0.0, 0.0, 1.0), frame_id='base'):
        self.PlanningScene.add_box(
            box_id, size, position, orientation, frame_id)

    def remove_box(self, box_id, frame_id='base'):
        return self.PlanningScene.remove_box(box_id, frame_id)

    def attach_object(self, object_id, link_name):
        return self.PlanningScene.attach_object(object_id, link_name)

    def detach_object(self, object_id, link_name):
        return self.PlanningScene.detach_object(object_id, link_name)

    def load_scene_from_parameters(self, parameters):
        return self.PlanningScene.load_scene_from_parameters(parameters)
