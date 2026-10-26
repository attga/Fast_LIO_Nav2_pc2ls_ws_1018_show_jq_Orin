#!/usr/bin/env python3
"""
robot_bringup.launch.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
One-click bringup for the full robot navigation system (FAST-LIO + Nav2)

Startup sequence:
  t= 0s : [A] 9-axis IMU driver (yesense_std_ros2)  <- highest priority, init first
  t= 3s : [B] Livox MID-360s LiDAR driver           <- wait for IMU to stabilize
  t=13s : [C] FAST-LIO localization system           <- wait for LiDAR init (~10s)
              Internal timers:
                t+1s (14s) -> RViz2 visualization
                t+3s (16s) -> Global ICP relocalization node starts
  t=30s : [D] Nav2 navigation stack                  <- wait for ICP first fix + buffer

Usage:
  # Launch with default map paths (verify default_value matches your board paths)
  ros2 launch my_nav2_config robot_bringup.launch.py

  # Override map paths from command line:
  ros2 launch my_nav2_config robot_bringup.launch.py \
      map_pcd:=/home/nvidia/maps/cloudGlobal.pcd \
      map:=/home/nvidia/maps/Nav2_map/cloudGlobal2D.yaml

Note: lio_localization.launch.py will auto-launch RViz2 at t=14s.
      Comment out the RViz node in that file if not needed.
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # ── 各功能包 share 路径 ────────────────────────────────────────────────────────────────────
    chassis_share      = get_package_share_directory('chassis_driver')
    yesense_share      = get_package_share_directory('yesense_std_ros2')
    livox_share        = get_package_share_directory('livox_ros_driver2')
    fast_lio_loc_share = get_package_share_directory('fast_lio_loc')
    nav2_share         = get_package_share_directory('my_nav2_config')

    # ── 时序常量（如需调整在此处修改，勿动下方逻辑）──────────────────
    T_LIDAR_START  = 3.0   # 雷达启动延时 (s)：等待 IMU 稳定
    T_FASTLIO_START = 13.0  # FAST-LIO 启动延时 (s)：T_LIDAR + 雷达初始化约10s
    T_NAV2_START   = 30.0  # Nav2 启动延时 (s)：T_FASTLIO + FAST-LIO帧(~2s)
                            #                   + ICP触发(3s) + ICP计算(~5s) + 缓冲(7s)

    # ── 可在命令行覆盖的全局参数 ──────────────────────────────────────
    map_pcd      = LaunchConfiguration('map_pcd')
    map_yaml     = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── 参数声明（default_value 请确认与板子实际路径一致）────────────
    declare_map_pcd = DeclareLaunchArgument(
        'map_pcd',
        default_value='/home/nvidia/Fast_LIO_Nav2_ws/maps/cloudGlobal.pcd',
        description='Absolute path to the prior PCD map used by FAST-LIO ICP relocalization'
    )

    declare_map_yaml = DeclareLaunchArgument(
        'map',
        default_value='/home/nvidia/Fast_LIO_Nav2_ws/maps/Nav2_map/cloudGlobal2D.yaml',
        description='Absolute path to the 2D occupancy grid map yaml used by Nav2 map_server'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock (set true for bag replay, false for real robot)'
    )

    # ────────────────────────────────────────────────────────────────────
    # [A] Chassis driver (CAN motor control) — starts at t=0s
    #     Subscribes to /cmd_vel and drives ZLAC8015D motors via CAN bus.
    #     NOTE: The odom->base_link TF in chassis_driver_node.cpp has been
    #           DISABLED. FAST-LIO owns the full TF chain: map->camera_init->body->base_link.
    # ────────────────────────────────────────────────────────────────────
    chassis_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(chassis_share, 'launch', 'chassis_driver.launch.py')
        )
    )

    # ────────────────────────────────────────────────────────────────────
    # [B] 9-axis IMU driver — starts at t=0s immediately
    #     IMU must power up first to provide initial attitude prior for FAST-LIO
    # ────────────────────────────────────────────────────────────────────
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yesense_share, 'launch', 'yesense_node.launch.py')
        )
    )

    # ────────────────────────────────────────────────────────────────────
    # [B] Livox MID-360s LiDAR driver — starts at t=3s
    #     Waits for IMU self-test and stable output (~2-3s)
    # ────────────────────────────────────────────────────────────────────
    livox_launch = TimerAction(
        period=T_LIDAR_START,
        actions=[
            LogInfo(msg='[robot_bringup] [t=3s]  IMU stable. Starting Livox MID-360s LiDAR driver...'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(livox_share, 'launch_ROS2', 'msg_MID360s_launch.py')
                )
            ),
        ]
    )

    # ────────────────────────────────────────────────────────────────────
    # [C] FAST-LIO localization system — starts at t=13s
    #     Waits for LiDAR driver to fully initialize and publish point clouds (~5-10s)
    #     Internal timers (relative to this node's start time):
    #       +1s (t=14s): RViz2 auto-launches
    #       +3s (t=16s): Global ICP relocalization node starts searching
    # ────────────────────────────────────────────────────────────────────
    localization_launch = TimerAction(
        period=T_FASTLIO_START,
        actions=[
            LogInfo(msg='[robot_bringup] [t=13s] LiDAR point cloud ready. Starting FAST-LIO localization...'),
            LogInfo(msg='[robot_bringup]          Expected: RViz@t=14s, ICP relocalization@t=16s'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(fast_lio_loc_share, 'launch', 'lio_localization.launch.py')
                ),
                launch_arguments={
                    'map_pcd':      map_pcd,
                    'use_sim_time': use_sim_time,
                }.items()
            ),
        ]
    )

    # ────────────────────────────────────────────────────────────────────
    # [D] Nav2 navigation stack — starts at t=30s
    #     Waits for ICP to complete first global relocalization and
    #     publish a stable map->camera_init TF.
    #     Timeline breakdown:
    #       t=13s FAST-LIO starts -> t=15s first odometry frame published
    #       t=16s ICP triggered   -> t=21s ICP first fix complete (~5s)
    #       t=30s Nav2 starts     -> ~9s safety buffer, ensures full TF chain
    # ────────────────────────────────────────────────────────────────────
    nav2_launch = TimerAction(
        period=T_NAV2_START,
        actions=[
            LogInfo(msg='━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'),
            LogInfo(msg='[robot_bringup] [t=30s] ICP relocalization complete. Starting Nav2...'),
            LogInfo(msg='[robot_bringup]          Wait for "Managed nodes are active", then'),
            LogInfo(msg='[robot_bringup]          click "2D Goal Pose" in RViz to start navigation!'),
            LogInfo(msg='━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_share, 'launch', 'nav2_bringup.launch.py')
                ),
                launch_arguments={
                    'map':          map_yaml,
                    'use_sim_time': use_sim_time,
                }.items()
            ),
        ]
    )

    # ── 组装启动描述符 ─────────────────────────────────────────────────
    return LaunchDescription([

        # 1. Parameter declarations (must come before all actions)
        declare_map_pcd,
        declare_map_yaml,
        declare_use_sim_time,

        # 2. Startup sequence
        LogInfo(msg='━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'),
        LogInfo(msg='[robot_bringup] FAST-LIO + Nav2 Robot Navigation System Starting'),
        LogInfo(msg='[robot_bringup] Sequence: Chassis+IMU(0s) -> LiDAR(3s) -> FAST-LIO(13s) -> Nav2(30s)'),
        LogInfo(msg='━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'),

        # [A] t=0s: Chassis driver starts immediately (CAN bus, independent of sensors)
        LogInfo(msg='[robot_bringup] [t=0s]  Starting chassis driver (ZLAC8015D via CAN)...'),
        chassis_launch,

        # [B] t=0s: IMU starts immediately
        LogInfo(msg='[robot_bringup] [t=0s]  Starting 9-axis IMU driver (yesense_std_ros2)...'),
        imu_launch,

        # [C] t=3s: LiDAR delayed start
        livox_launch,

        # [D] t=13s: FAST-LIO delayed start
        localization_launch,

        # [E] t=30s: Nav2 delayed start
        nav2_launch,
    ])
