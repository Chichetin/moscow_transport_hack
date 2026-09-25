"""Jury entry point: ros2 launch tram_odometry odometry.launch.py [params_file:=<yaml>]"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('tram_odometry'), 'config',
                                  'params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='params.yaml of the node (docs/contracts.md §3)'),
        Node(package='tram_odometry', executable='odometry_node', name='tram_odometry',
             output='screen', parameters=[{'params_file': LaunchConfiguration('params_file')}]),
    ])
