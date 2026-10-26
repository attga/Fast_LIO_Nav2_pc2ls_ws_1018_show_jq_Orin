#!/usr/bin/env python3
"""MID360 定位入口：SC 启动初始化、双 EKF、Yesense IMU 与持续 NDT。"""
import os
import math
import yaml
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch_ros.actions import Node
from ml_bringup.description import robot_description


def launch_nodes(context):
    options = context.launch_configurations
    delay = float(options['manager_start_delay'])
    if not math.isfinite(delay) or delay < 0.0:
        raise ValueError('manager_start_delay must be finite and nonnegative')
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

    # 交接时间门限仅在 Manager YAML 配置；两端显式接收同一组值。
    with open(options['manager_config'], encoding='utf-8') as stream:
        manager_settings = yaml.safe_load(stream)['ml_localization_manager']['ros__parameters']
    reloc_timing = {key: manager_settings[key] for key in
                    ('max_reloc_age', 'reloc_sync_tolerance')}
    local_parameters = [os.path.join(ekf_config, 'local_ekf.yaml')]
    global_parameters = [os.path.join(ekf_config, 'global_ekf.yaml')]
    # 前端和重定位共用 URDF 安装关系；默认采用新车后轴中心配置。
    description = robot_description(
        options['robot_mounts_file'],
        enable_yesense=options['enable_yesense'].lower() == 'true',
        imu_mount=options['imu_mount'], wheel_mount=options['wheel_mount'])
    nodes = [Node(package='robot_state_publisher', executable='robot_state_publisher',
                  name='robot_state_publisher', output='screen',
                  parameters=[sim, {'robot_description': description}])]
    if options['enable_yesense'].lower() == 'true':
        imu_config = os.path.join(ekf_config, 'imu.yaml')
        local_parameters.append(imu_config)
        global_parameters.append(imu_config)
        nodes.extend([
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
                 'output.base_frame': 'base_link',
                 'output.imu_frame': 'livox_imu_link',
                 'output.lidar_frame': 'livox_lidar_link',
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
        Node(package='ml_ndt_localization', executable='ml_ndt_localization_node',
             name='ml_ndt_localization_node', output='screen', parameters=[
                 options['ndt_config'], {'map_directory': map_parameters['map_directory']}, sim,
                 {'wait_for_initialization': True}]),
        Node(package='ml_relocalization', executable='relocalization_node',
             name='alaserRelocalizer', output='screen', parameters=[
                 options['relocalization_config'], map_parameters, sim,
                 reloc_timing, {'start_on_launch': False, 'base_frame': 'base_link',
                  'matching_frame': 'livox_imu_link'}]),
    ])
    manager = Node(package='ml_localization_manager', executable='ml_localization_manager',
                   name='ml_localization_manager', output='screen',
                   parameters=[options['manager_config'], sim, reloc_timing])
    # 临时规避服务就绪竞态：使用实际经过的秒数，不依赖 bag 的 /clock。
    # 固定延迟不保证地图加载完成；服务未就绪的请求计时问题仍待修复。
    nodes.append(TimerAction(period=delay, actions=[manager]) if delay > 0.0 else manager)
    if options['rviz'].lower() == 'true':
        nodes.append(Node(package='rviz2', executable='rviz2', name='rviz',
                          parameters=[sim], arguments=[
                              '-d', os.path.join(share, 'rviz', 'fast_lio_livox.rviz')]))
    return nodes


def generate_launch_description():
    share = get_package_share_directory('ml_bringup')
    reloc_share = get_package_share_directory('ml_relocalization')
    return LaunchDescription([
        DeclareLaunchArgument('map_directory', description='外部地图目录：包含 Scans、优化位姿、sc_db.bin 和 map.pcd'),
        DeclareLaunchArgument('pose_file', default_value='isam_poses.txt'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('manager_start_delay', default_value='6.0',
                              description='Manager 延迟启动的实际秒数；临时措施，0 表示立即启动'),
        DeclareLaunchArgument('enable_yesense', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('robot_mounts_file',
                              default_value=os.path.join(share, 'config', 'robot', 'rear_axle.yaml'),
                              description='URDF 安装配置；后轴中心配置，Livox yaw=+90°；不适用于旧车 bag'),
        DeclareLaunchArgument('imu_mount', default_value='',
                              description='T_base_link_gyro_link（IMU 到 base_link）：[x,y,z,roll,pitch,yaw]，单位 m/rad'),
        DeclareLaunchArgument('wheel_mount', default_value='',
                              description='T_base_link_body（轮速 body 到 base_link）：[x,y,z,roll,pitch,yaw]，单位 m/rad'),
        DeclareLaunchArgument('ekf_config_directory', default_value=os.path.join(share, 'config', 'ekf')),
        DeclareLaunchArgument('fastlio_config', default_value=os.path.join(
            get_package_share_directory('fast_lio'), 'config', 'mid360.yaml')),
        DeclareLaunchArgument('ndt_config', default_value=os.path.join(
            get_package_share_directory('ml_ndt_localization'), 'config', 'localization.yaml')),
        DeclareLaunchArgument('manager_config', default_value=os.path.join(
            get_package_share_directory('ml_localization_manager'), 'config', 'manager.yaml')),
        DeclareLaunchArgument('relocalization_config', default_value=os.path.join(
            reloc_share, 'config', 'relocalization.yaml')),
        DeclareLaunchArgument('fastlio_log_directory', default_value='',
                              description='默认在 ROS_LOG_DIR 或 ~/.ros/log 下创建带时间标记的目录'),
        OpaqueFunction(function=launch_nodes),
    ])
