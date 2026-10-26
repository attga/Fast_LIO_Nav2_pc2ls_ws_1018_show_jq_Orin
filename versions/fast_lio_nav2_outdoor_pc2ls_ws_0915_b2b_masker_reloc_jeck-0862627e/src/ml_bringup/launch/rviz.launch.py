#!/usr/bin/env python3
"""Standalone viewer using the same RViz configuration as localization.launch.py."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('ml_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        Node(package='rviz2', executable='rviz2', name='rviz',
             parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
             arguments=['-d', os.path.join(share, 'rviz', 'fast_lio_livox.rviz')]),
    ])
