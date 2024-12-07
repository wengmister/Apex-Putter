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
        Node(
            package='rviz2',
            executable='rviz2',
            output='log',
            arguments=['-d', PathJoinSubstitution(
                [FindPackageShare('franka_fer_moveit_config'), 'config',
                 'moveit.rviz'])],
            parameters=[moveit_config.planning_pipelines,
                        moveit_config.robot_description_kinematics],
            condition=IfCondition(EqualsSubstitution(
                LaunchConfiguration('demo'), 'false'))
        ),
        IncludeLaunchDescription(
            launch_description_source=[FindPackageShare(
                'franka_fer_moveit_config'), '/launch', '/demo.launch.py'],
            condition=IfCondition(EqualsSubstitution(
                LaunchConfiguration('demo'), 'true'))
        ),
        Node(
            package='apex_putter',
            executable='demo_node',
            exec_name='demo_node'
        )
    ])
