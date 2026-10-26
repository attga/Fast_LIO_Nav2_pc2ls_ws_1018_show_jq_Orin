#!/usr/bin/env python3
"""独立 FAST-LIO 前端，加载 fast_lio 包中的 MID360 配置。
发布 /lidar_odometry 和 /cloud_registered_body，关闭动态 TF。
完整定位 TF 链请使用 localization.launch.py。
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
