from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("fer", package_name="franka_fer_moveit_config").to_moveit_configs()
    return LaunchDescription([
        # IncludeLaunchDescription(
        #     launch_description_source=PathJoinSubstitution([
        #         FindPackageShare('franka_fer_moveit_config'),
        #         'launch',
        #         'moveit_rviz.launch.py'
        #         # 'demo.launch.py'
        #     ]),
        #     launch_arguments={'robot_ip': 'panda0.robot'}.items()
        # ),
        IncludeLaunchDescription(
            launch_description_source=PathJoinSubstitution([
                FindPackageShare('apex_putter'),
                'launch',
                'vision.launch.xml'
            ])
        ),
        Node(
            package='apex_putter',
            executable='demo',
            name='demo'  # Changed from exec_name to name
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='log',
            arguments=[
                '-d',
                PathJoinSubstitution([
                    FindPackageShare('apex_putter'),
                    'config',
                    'apriltag.rviz'
                ]),
                '--ros-args',
                '--log-level',
                'fatal'
            ],
            parameters=[
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics
            ],
        )
    ])
