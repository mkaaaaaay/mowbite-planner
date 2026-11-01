"""Starts the MowBite Planner with config/mowbite_planner.yaml.

    ros2 launch mowbite_planner mowbite_planner.launch.py
    ros2 launch mowbite_planner mowbite_planner.launch.py params_file:=/path/to/my.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('mowbite_planner'), 'config', 'mowbite_planner.yaml']),
            description='parameter file of the planner'),
        DeclareLaunchArgument(
            'node_name', default_value='mowbite_planner',
            description='node name, its services and topic are under it (~/plan_coverage, ...)'),
        Node(
            package='mowbite_planner',
            executable='mowbite_planner_node',
            name=LaunchConfiguration('node_name'),
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
