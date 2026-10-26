import os
import shlex
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    parameter_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    map_data_dir = LaunchConfiguration('map_data_dir')
    xacro_path = os.path.join(share_dir, 'config', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(share_dir, 'config', 'relocalization.rviz')
    if not os.path.exists(rviz_config_file):
        rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'config', 'params.yaml'),
        description='Path to the ROS2 parameters file to use.'
    )

    return LaunchDescription([
        params_declare,
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('map_data_dir', default_value=os.path.join(share_dir, 'data')),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro', ' ', shlex.quote(xacro_path)]),
                'use_sim_time': use_sim_time
            }]
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_relocalization',
            name='lio_sam_relocalization',
            parameters=[parameter_file, {'use_sim_time': use_sim_time, 'mapDataDir': map_data_dir}],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_sc_pose',
            name='lio_sam_sc_pose',
            parameters=[parameter_file, {'use_sim_time': use_sim_time, 'mapDataDir': map_data_dir}],
            output='screen'
        ),
        # IMU 预积分 + TransformFusion: 输出 Nav2 所需的 TF 链 (map->odom, odom->base_link)
        # 与 sc_liorf_localization 定位链路一致, 替代 AMCL/EKF
        Node(
            package='lio_sam',
            executable='lio_sam_imuPreintegration',
            parameters=[parameter_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(use_rviz),
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])
