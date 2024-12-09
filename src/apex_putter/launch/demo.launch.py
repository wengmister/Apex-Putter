from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, \
    PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        'fer', package_name='franka_fer_moveit_config').to_moveit_configs()

    return LaunchDescription([
        DeclareLaunchArgument('demo', default_value='true',
                              description='Sets the mode to demo.'),
        IncludeLaunchDescription(
            launch_description_source=[FindPackageShare(
                'franka_fer_moveit_config'), '/launch', '/franka_fer_moveit_config'],
            launch_arguments={'robot_ip': 'panda0.robot'}  
        ),
        IncludeLaunchDescription(
            launch_description_source=[FindPackageShare(
                'apex_putter'), '/launch', '/vision.launch.xml']
        ),
        Node(
            package='apex_putter',
            executable='demo',
            exec_name='demo'
        )
    ])