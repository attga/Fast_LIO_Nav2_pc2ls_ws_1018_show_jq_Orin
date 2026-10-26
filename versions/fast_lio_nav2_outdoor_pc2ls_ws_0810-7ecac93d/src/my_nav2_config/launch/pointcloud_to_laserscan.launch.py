#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            remappings=[
                ('cloud_in', '/cloud_registered_body'),
                ('scan',     '/scan'),
            ],
            parameters=[{
                # Key param: height filtering in base_link frame,
                # completely decoupled from global Z-axis drift
                'target_frame':        'base_link',
                'transform_tolerance':  0.1,
                # Height bounds relative to base_link (independent of global Z)
                'min_height':  0.10,   # filter ground & chassis structure (m)
                'max_height':  1.5,    # filter overhead branches / objects (m)
                # Scan angle range (full 360 deg)
                'angle_min':       -3.14159,
                'angle_max':        3.14159,
                'angle_increment':  0.0087,   # ~0.5 deg resolution
                # Distance range
                'range_min':  0.3,    # filter robot body noise (m)
                'range_max':  5.0,    # matches costmap observation range (m)
                # Timing
                'scan_time':  0.1,    # one scan frame = 1/10 Hz (s)
                # Infinity handling
                'use_inf':      True,
                'inf_epsilon':  1.0,
            }]
        )
    ])


