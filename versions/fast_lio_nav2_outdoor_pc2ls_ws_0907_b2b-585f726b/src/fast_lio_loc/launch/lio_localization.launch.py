#!/usr/bin/env python3
"""
lio_localization.launch.py
─────────────────────────────────────────────────────────
全栈定位 launch（用于 bag 回放测试重定位）

架构:
  [LIO-SAM PCD 先验地图]
          ↓ 加载
  [global_localization_ros2.py] → ICP → map->odom TF
          +
  [FAST-LIO2 fastlio_mapping] → ESIKF → camera_init->base_link TF
          ↓
  [RViz2] 可视化

使用示例:
  # 终端1 - 启动定位系统
  source install/setup.bash
  ros2 launch my_nav2_config lio_localization.launch.py \\
      map_pcd:=$HOME/Downloads/LOAM/GlobalMap.pcd

  # 终端2 - 从不同初始位置播放 bag
  ros2 bag play outdoor_Mainbuilding_10hz_2020-12-24-16-38-00 \\
      --clock --rate 0.5 --start-offset 60
─────────────────────────────────────────────────────────
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    fast_lio_pkg = get_package_share_directory('fast_lio')

    # ── 启动参数 ───────────────────────────────────────────
    map_pcd       = LaunchConfiguration('map_pcd')
    use_sim_time  = LaunchConfiguration('use_sim_time')
    config_file   = LaunchConfiguration('config_file')
    fitness_th    = LaunchConfiguration('fitness_threshold')
    loc_freq      = LaunchConfiguration('localization_freq')
    map_voxel     = LaunchConfiguration('map_voxel_size')
    fov_far       = LaunchConfiguration('fov_far')
    enable_ws     = LaunchConfiguration('enable_warm_start')
    yaw_offset    = LaunchConfiguration('yaw_map_offset_deg')
    invert_imu    = LaunchConfiguration('invert_imu_yaw')
    sanity_z      = LaunchConfiguration('sanity_z_max')
    sc_map_dir    = LaunchConfiguration('sc_map_dir')
    enable_yesense= LaunchConfiguration('enable_yesense_yaw')

    return LaunchDescription([
        # ── 参数声明 ──────────────────────────────────────
        DeclareLaunchArgument(
            'map_pcd',
            default_value=os.path.abspath('maps/num12around/GlobalMap.pcd'),
            description='先验 PCD 全局地图路径'),

        DeclareLaunchArgument(
            'sc_map_dir',
            default_value='',
            description='ScanContext .npy 文件所在目录，留空则默认使用 map_pcd 所在目录'),

        DeclareLaunchArgument(
            'enable_yesense_yaw',
            default_value='true',
            description='是否等待并使用 Yesense IMU 磁力计进行 Yaw 粗筛去歧义'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='bag 回放=true；实机=false'),

        DeclareLaunchArgument(
            'config_file', default_value='mid360.yaml',
            description='FAST-LIO2 配置文件名'),

        DeclareLaunchArgument(
            'fitness_threshold', default_value='0.75',
            description='ICP fitness 阈值 (0~1)，越大越严格'),

        DeclareLaunchArgument(
            'localization_freq', default_value='0.5',
            description='全局定位频率 (Hz)'),

        DeclareLaunchArgument(
            'map_voxel_size', default_value='0.4',
            description='地图降采样体素尺寸 (m)'),

        DeclareLaunchArgument(
            'fov_far', default_value='80.0',
            description='定位最远距离裁剪 (m)'),

        DeclareLaunchArgument(
            'enable_warm_start', default_value='false',
            description='热启动开关：室内对称环境用 false，室外/非对称用 true'),
        # default_value='-155.72'
        DeclareLaunchArgument(
            'yaw_map_offset_deg', default_value='-38.0',
            description='Yaw偏移(deg)，-9999=自动标定，手动值=正确ICP_Yaw - IMU_Yaw'
            ' | 当前值-155.72,-38.0,由原点标定(ICP_Yaw=0, IMU_Yaw=155.72)室内'),
            
        DeclareLaunchArgument(
            'invert_imu_yaw', default_value='false',
            description='是否反转IMU Yaw的符号。若IMU采用NED(顺时针为正)则设为true，以适配ROS(逆时针为正)'),
            
        DeclareLaunchArgument(
            'sanity_z_max', default_value='100.0',
            description='高度验证最大允许误差(m)。室内设为0.5防止匹配到天花板；室外起伏地形建议设大(如100.0)'),

        LogInfo(msg=['[lio_localization] 先验地图: ', map_pcd]),
        LogInfo(msg=['[lio_localization] use_sim_time = ', use_sim_time]),

        # ── Node 1: 静态 TF ─────────
        # 连接 fast_lio 的 body 与 nav2 需要的 base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_body_to_base',
            # arguments=['0.20', '0', '-0.53', '0', '0', '0', 'body', 'base_link'],
            arguments=['0', '0', '0', '0', '0', '0', 'body', 'base_link'],

            output='screen',
        ),
        # 连接 base_link 与 livox_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_lidar',
            # arguments=['-0.211', '-0.023', '0.574', '0', '0', '0', 'base_link', 'livox_frame'],
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'livox_frame'],
            output='screen',
        ),

        # ── Node 2: FAST-LIO2 ─────────────────────────────────
        # 发布: /Odometry (nav_msgs), /cloud_registered (PointCloud2)
        # TF:   camera_init -> body
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='fast_lio_mapping',
            output='screen',
            parameters=[
                PathJoinSubstitution([fast_lio_pkg, 'config', config_file]),
                {'use_sim_time': use_sim_time},
            ],
        ),

        # ── Node 3: RViz2 (延迟1s等FAST-LIO初始化) ────────────
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', os.path.join(fast_lio_pkg, 'rviz', 'fastlio.rviz')],
                    output='screen',
                ),
            ]
        ),

        # ── Node 4: 全局 ICP 重定位 (延迟3s等FAST-LIO第一帧) ──
        # 订阅: /cloud_registered, /Odometry, /initialpose
        # 发布: map->camera_init TF (即 map->odom)
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg='[lio_localization] 启动全局 ICP 重定位节点...'),
                Node(
                    package='fast_lio_loc',
                    executable='global_localization_ros2.py',
                    name='fast_lio_localization',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'map_pcd_path': map_pcd,
                        'fitness_threshold': fitness_th,
                        'localization_freq': loc_freq,
                        'map_voxel_size': map_voxel,
                        'scan_voxel_size': 0.1,
                        'fov_far': fov_far,
                        'odom_frame': 'camera_init',
                        # 热启动开关：室内对称环境用 false（冷启动 RANSAC 更可靠）
                        # 室外/非对称环境可在启动时传入 enable_warm_start:=true
                        'enable_warm_start': enable_ws,
                        # Yaw标定偏移：-9999=自动标定（首次定位成功后保存），
                        # 或手动指定 offset=正确ICP_Yaw - IMU_Yaw 解决首次混淆
                        'yaw_map_offset_deg': yaw_offset,
                        'invert_imu_yaw': invert_imu,
                        'sanity_z_max': sanity_z,
                        'sc_map_dir': sc_map_dir,
                        'enable_yesense_yaw': enable_yesense,
                    }],
                ),
            ]
        ),
    ])
