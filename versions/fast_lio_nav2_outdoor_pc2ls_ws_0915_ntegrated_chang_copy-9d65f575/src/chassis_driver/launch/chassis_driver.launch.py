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
                'track_width': 0.46,
                'wheel_radius': 0.085,
                'odom_rate': 50.0,
                'can_interface': 'can0',
            }]
        ),
    ])
