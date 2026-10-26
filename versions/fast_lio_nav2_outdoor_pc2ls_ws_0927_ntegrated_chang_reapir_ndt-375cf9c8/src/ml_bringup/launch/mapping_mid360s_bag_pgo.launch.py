#!/usr/bin/env python3
"""
FAST-LIO2 + SC-PGO 离线回放 Launch(回环检测 + 位姿图优化)

用法:
    终端1: ros2 launch ml_bringup mapping_mid360s_bag_pgo.launch.py rviz:=true
    终端2: ros2 bag play <bag> --topics /livox/lidar /livox/imu -r 2.0
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

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        # 产物目录:与 tools 同级的 src/map(map.pcd / sc_db.bin / poses / times / Scans)
        # realpath 解析 symlink-install 符号链接,定位到源码目录
        DeclareLaunchArgument('save_directory',
                              default_value=os.path.join(os.path.dirname(os.path.dirname(
                                  os.path.realpath(__file__))), 'map') + '/'),

        # ================= FAST-LIO2 里程计 =================
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
                {'runtime_pos_log_enable': False},
                {'pcd_save_enable': False},
            ],
        ),

        # ================= 回环检测 + 位姿图优化(PGO) =================
        Node(
            package='sc_pgo',
            executable='alaserPGO', sigterm_timeout='120', sigkill_timeout='120',
            name='alaserPGO',
            output='screen',
            parameters=[
                {'save_directory': LaunchConfiguration('save_directory')},
                # ---- 关键帧 ----
                {'keyframe_meter_gap': 0.5},
                {'keyframe_deg_gap': 10.0},
                # ---- 距离回环检测(优化位姿半径搜索 + 时间差下界) ----
                {'loop_distance_radius': 50.0},
                {'loop_distance_time_diff': 200.0},
                # ---- GICP 验证(对应距离 30m 在代码内) ----
                {'loop_fitness_score_thres': 0.7},
                {'loop_icp_t_max': 10.0},
                {'loop_noise_score': 0.5},
                # ---- 位姿图 ----
                {'odom_trans_noise': 1e-4, 'odom_rot_noise': 1e-6},
                {'mapviz_filter_size': 0.4},
                # ---- 地面 z 约束(压 z 漂移) ----
                {'ground_z_en': True},
                {'ground_z_sigma': 0.1},
                {'ground_z_kappa': 1000.0},
            ],
            remappings=[
                # PGO 订阅 FAST-LIO 的里程计和点云
                ('/aft_mapped_to_init', '/lidar_odometry'),
                ('/velodyne_cloud_registered_local', '/cloud_registered_body'),
                # 注：ROS1 原版第三个 remap (/cloud_for_scancontext) 从未被代码订阅，是 no-op，已移除
            ],
        ),

        # ================= RViz2 =================
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', os.path.join(pkg_dir, 'rviz', 'fast_lio_livox.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
