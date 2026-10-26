#!/usr/bin/env python3
"""Standalone FAST-LIO frontend. Uses the shared MID360 profile.
Publishes /lidar_odometry and /cloud_registered_body; dynamic TF is disabled.
Use localization.launch.py for the complete localization TF chain.
"""

from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # ================= FAST-LIO2 前端 =================
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='laserMapping',
            output='screen',
            parameters=[
                os.path.join(get_package_share_directory('fast_lio'), 'config', 'mid360.yaml'),
                {'publish.scan_publish_en': True},
                {'publish.dense_publish_en': True},
                {'publish.scan_bodyframe_pub_en': True},
                {'publish.tf_publish_en': False},
                {'runtime_pos_log_enable': False},
                {'pcd_save_enable': False},
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
