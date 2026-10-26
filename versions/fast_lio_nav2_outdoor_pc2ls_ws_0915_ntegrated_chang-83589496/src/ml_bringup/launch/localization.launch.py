#!/usr/bin/env python3
"""MID360 定位入口：SC 启动初始化、双 EKF、轮速/IMU 与持续 ICP。"""
import ast
import math
import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def mount_node(name, child, value, sim):
    mount = ast.literal_eval(value)
    if not isinstance(mount, (list, tuple)) or len(mount) != 6:
        raise ValueError(f'{name} requires [x,y,z,roll,pitch,yaw], metres/radians')
    mount = [float(v) for v in mount]
    if not all(math.isfinite(v) for v in mount):
        raise ValueError(f'{name} must contain finite values')
    arguments = []
    for flag, value in zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'), mount):
        arguments.extend(['--' + flag, str(value)])
    return Node(package='tf2_ros', executable='static_transform_publisher',
                name=name, output='screen', parameters=[sim],
                arguments=arguments + ['--frame-id', 'base_link', '--child-frame-id', child])


def launch_nodes(context):
    options = context.launch_configurations
    lidar_mount_yaw = float(options['lidar_mount_yaw'])
    if not math.isfinite(lidar_mount_yaw):
        raise ValueError('lidar_mount_yaw must be finite (radians)')
    share = get_package_share_directory('ml_bringup')
    ekf_config = os.path.abspath(os.path.expanduser(options['ekf_config_directory']))
    sim = {'use_sim_time': options['use_sim_time'].lower() == 'true'}
    map_parameters = {
        'map_directory': os.path.abspath(os.path.expanduser(options['map_directory'])),
        'pose_file': options['pose_file'],
    }
    log_directory = options['fastlio_log_directory']
    if not log_directory:
        log_root = os.environ.get('ROS_LOG_DIR', os.path.expanduser('~/.ros/log'))
        log_directory = os.path.join(log_root, 'fast_lio_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
    log_directory = os.path.abspath(os.path.expanduser(log_directory))
    os.makedirs(log_directory, exist_ok=True)

    local_parameters = [os.path.join(ekf_config, 'local_ekf.yaml')]
    global_parameters = [os.path.join(ekf_config, 'global_ekf.yaml')]
    nodes = [mount_node('wheel_odom_frame', 'body', options['wheel_mount'], sim)]
    if options['enable_yesense'].lower() == 'true':
        imu_config = os.path.join(ekf_config, 'imu.yaml')
        local_parameters.append(imu_config)
        nodes.extend([
            mount_node('yesense_mount', 'gyro_link', options['imu_mount'], sim),
            Node(package='ml_bringup', executable='imu_measurements.py',
                 name='imu_measurements', output='screen', parameters=[
                     os.path.join(ekf_config, 'imu_measurements.yaml'), sim]),
        ])

    nodes.extend([
        Node(package='fast_lio', executable='fastlio_mapping', name='laserMapping',
             output='screen', parameters=[options['fastlio_config'], sim, {
                 'publish.scan_publish_en': True,
                 'publish.dense_publish_en': True,
                 'publish.scan_bodyframe_pub_en': True,
                 'publish.tf_publish_en': False,
                 'publish.imu_to_base_yaw': lidar_mount_yaw,
                 'runtime_pos_log_enable': False,
                 'pcd_save_enable': False,
                 'debug_log_directory': log_directory,
             }]),
        Node(package='robot_localization', executable='ekf_node',
             name='ekf_filter_node_local', output='screen',
             remappings=[('odometry/filtered', '/local_odometry/filter')],
             parameters=local_parameters + [sim]),
        Node(package='robot_localization', executable='ekf_node',
             name='ekf_filter_node_global', namespace='global', output='screen',
             remappings=[('odometry/filtered', '/global_odometry/filter')],
             parameters=global_parameters + [sim]),
        Node(package='ml_icp_localization', executable='ml_icp_localization_node',
             name='ml_icp_localization_node', output='screen', parameters=[
                 options['icp_config'], map_parameters, sim,
                 {'wait_for_initialization': True}]),
        Node(package='ml_relocalization', executable='relocalization_node',
             name='alaserRelocalizer', output='screen', parameters=[
                 options['relocalization_config'], map_parameters, sim,
                 {'start_on_launch': False}]),
        Node(package='ml_localization_manager', executable='ml_localization_manager',
             name='ml_localization_manager', output='screen', parameters=[
                 options['manager_config'], sim]),
    ])
    if options['rviz'].lower() == 'true':
        nodes.append(Node(package='rviz2', executable='rviz2', name='rviz',
                          parameters=[sim], arguments=[
                              '-d', os.path.join(share, 'rviz', 'fast_lio_livox.rviz')]))
    return nodes


def generate_launch_description():
    share = get_package_share_directory('ml_bringup')
    reloc_share = get_package_share_directory('ml_relocalization')
    identity = '[0.0,0.0,0.0,0.0,0.0,0.0]'
    return LaunchDescription([
        DeclareLaunchArgument('map_directory', description='外部地图目录：包含 Scans、优化位姿、sc_db.bin 和 map.pcd'),
        DeclareLaunchArgument('pose_file', default_value='isam_poses.txt'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('enable_yesense', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('lidar_mount_yaw', default_value='1.5707963267948966',
                              description='MID360 内置 IMU 在车体系中的安装 yaw，单位 rad；俯视向左 90 度为 +1.5707963267948966。仅改输出坐标，不改雷达-内置 IMU 外参。'),
        DeclareLaunchArgument('imu_mount', default_value=identity,
                              description='T_base_link_gyro_link（IMU 到 base_link）：[x,y,z,roll,pitch,yaw]，单位 m/rad'),
        DeclareLaunchArgument('wheel_mount', default_value=identity,
                              description='T_base_link_body（轮速 body 到 base_link）：[x,y,z,roll,pitch,yaw]，单位 m/rad'),
        DeclareLaunchArgument('ekf_config_directory', default_value=os.path.join(share, 'config', 'ekf')),
        DeclareLaunchArgument('fastlio_config', default_value=os.path.join(
            get_package_share_directory('fast_lio'), 'config', 'mid360.yaml')),
        DeclareLaunchArgument('icp_config', default_value=os.path.join(
            get_package_share_directory('ml_icp_localization'), 'config', 'localization.yaml')),
        DeclareLaunchArgument('manager_config', default_value=os.path.join(
            get_package_share_directory('ml_localization_manager'), 'config', 'manager.yaml')),
        DeclareLaunchArgument('relocalization_config', default_value=os.path.join(
            reloc_share, 'config', 'relocalization.yaml')),
        DeclareLaunchArgument('fastlio_log_directory', default_value='',
                              description='默认在 ROS_LOG_DIR 或 ~/.ros/log 下创建带时间标记的目录'),
        OpaqueFunction(function=launch_nodes),
    ])
