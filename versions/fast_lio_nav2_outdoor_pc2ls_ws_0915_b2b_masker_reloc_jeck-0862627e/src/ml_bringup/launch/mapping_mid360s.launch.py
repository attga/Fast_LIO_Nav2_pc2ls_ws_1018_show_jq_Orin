#!/usr/bin/env python3
"""
FAST-LIO2 + Livox Mid-360s Combined Launch File

用法:
    ros2 launch fast_lio mapping_mid360s.launch.py

自定义参数:
    ros2 launch fast_lio mapping_mid360s.launch.py \
        publish_freq:=20.0 \
        lidar_ip:=192.168.1.12
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ml_bringup')
    livox_pkg_dir = get_package_share_directory('livox_ros_driver2')

    # ---- Livox 驱动参数 ----
    # xfer_format: 1=CustomMsg (FAST-LIO需要逐点时间戳)，用整数避免类型不匹配
    publish_freq = LaunchConfiguration('publish_freq', default='10.0')
    lidar_ip = LaunchConfiguration('lidar_ip', default='192.168.1.12')  # 雷达IP
    frame_id = LaunchConfiguration('frame_id', default='livox_frame')

    # ---- FAST-LIO 参数 ----
    livox_config_path = os.path.join(livox_pkg_dir, 'config', 'MID360s_config.json')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'fast_lio_livox.rviz')

    return LaunchDescription([
        # ================= Livox Mid-360s 驱动 =================
        # 使用 xfer_format=1 输出 CustomMsg（FAST-LIO需要逐点时间戳做运动补偿）
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen',
            parameters=[{
                'xfer_format': 1,                 # 1 = CustomMsg
                'multi_topic': 0,                  # 0 = 所有雷达共用话题
                'data_src': 0,                     # 0 = 实时雷达数据
                'publish_freq': publish_freq,      # 发布频率
                'output_data_type': 0,
                'frame_id': frame_id,
                'lvx_file_path': '/home/livox/livox_test.lvx',
                'user_config_path': livox_config_path,
                'cmdline_input_bd_code': 'livox0000000001',
            }],
        ),

        # ================= FAST-LIO2 SLAM =================
        # Mid-360: lidar_type=1 (Livox), scan_line=40
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='laserMapping',
            output='screen',
            parameters=[{
                # 话题（与 livox_ros_driver2 默认话题一致）
                'common.lid_topic': '/livox/lidar',
                'common.imu_topic': '/livox/imu',
                'common.time_sync_en': False,

                # Mid-360 雷达参数
                'preprocess.lidar_type': 1,     # 1 = Livox (AVIA/Mid-360/HAP 通用)
                'preprocess.scan_line': 40,      # Mid-360 有 40 条扫描线
                'preprocess.blind': 9.0,   # 过滤水平距离3m内且高度<2m的点（blind=半径²）

                # IMU 噪声参数（默认值，可调优）
                'mapping.acc_cov': 0.1,
                'mapping.gyr_cov': 0.1,
                'mapping.b_acc_cov': 0.0001,
                'mapping.b_gyr_cov': 0.0001,
                'mapping.fov_degree': 360.0,     # Mid-360 水平 360° FOV
                'mapping.det_range': 200.0,      # Mid-360 最大测距约 200m

                # 外参 LiDAR → IMU（根据实际安装调整）
                'mapping.extrinsic_est_en': False,
                'mapping.extrinsic_T': [-0.011, -0.02329, 0.04412],
                'mapping.extrinsic_R': [
                    1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0,
                ],

                # 发布
                'publish.scan_publish_en': True,
                'publish.dense_publish_en': True,
                'publish.scan_bodyframe_pub_en': True,

                # 特征提取（Livox 需关闭内置特征提取，使用全部点云）
                'feature_extract_enable': False,
                'point_filter_num': 2,
                'max_iteration': 3,
                'filter_size_surf': 0.5,
                'filter_size_map': 0.5,
                'cube_side_length': 1000.0,

                # 调试
                'runtime_pos_log_enable': False,
                'pcd_save_enable': False,
            }],
        ),

        # ================= RViz2 可视化（可选） =================
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
