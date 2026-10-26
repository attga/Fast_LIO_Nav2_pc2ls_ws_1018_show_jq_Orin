"""仅发布固定安装关系；不启动前端、EKF 或定位。"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from ml_bringup.description import robot_description


def nodes(context):
    options = context.launch_configurations
    description = robot_description(options['mounts_file'],
                                    options['enable_yesense'] == 'true')
    return [Node(package='robot_state_publisher', executable='robot_state_publisher',
                 name='robot_state_publisher', output='screen', parameters=[{
                     'robot_description': description,
                     'use_sim_time': options['use_sim_time'] == 'true'}])]


def generate_launch_description():
    share = get_package_share_directory('ml_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('mounts_file', default_value=os.path.join(share, 'config', 'robot', 'rear_axle.yaml')),
        DeclareLaunchArgument('enable_yesense', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=nodes),
    ])
