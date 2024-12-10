from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            launch_description_source=PathJoinSubstitution([
                FindPackageShare('franka_fer_moveit_config'),
                'launch',
                'demo.launch.py'
            ]),
            # launch_arguments={'robot_ip': 'panda0.robot'}.items()
        ),
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
        )
    ])