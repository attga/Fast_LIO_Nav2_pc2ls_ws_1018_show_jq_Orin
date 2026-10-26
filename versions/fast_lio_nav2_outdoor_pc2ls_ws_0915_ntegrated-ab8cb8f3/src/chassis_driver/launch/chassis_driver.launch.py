import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='chassis_driver',
            executable='chassis_driver_node',
            name='chassis_driver_node',
            output='screen',
            parameters=[{
                'track_width': 0.258,
                'wheel_radius': 0.0509,
                'odom_rate': 50.0,
                'path_publish_rate': 2.0,  # 仅 /trajectory 显示限频，/wheel_odom 随新反馈发布。
                'max_odom_gap': 0.5,       # 仅限制轮速积分跨越的时间缺口，不是停车超时。
                'can_interface': 'can0',
            }]
        ),
    ])
