import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
import os
from launch.actions import TimerAction

def generate_launch_description():
    pkg_share = get_package_share_directory('my_nav2_config')
    
    # --------------------------------------------------------------------------
    # 启动配置参数
    # --------------------------------------------------------------------------
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    map_yaml_file = LaunchConfiguration('map')
    use_keepout = LaunchConfiguration('use_keepout')
    mask_yaml_file = LaunchConfiguration('mask')
    
    # 默认使用支持掩膜层的参数配置文件，同时支持命令行指定覆盖
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        description='Nav2 参数文件的完整路径')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时钟 (Gazebo测试时设为true)')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='是否自动启动 Nav2 状态机')

    # 静态地图 yaml 路径，由 map_server 加载
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='map_jeck/map.yaml',
        description='Nav2 map_server 使用的静态地图 yaml 文件完整路径')

    # 是否启用禁区掩膜层 (Keepout Filter)
    declare_use_keepout_cmd = DeclareLaunchArgument(
        'use_keepout',
        default_value='true',
        description='是否启用 Nav2 禁区掩膜层 (Keepout Zone)')

    # 禁区掩膜地图 yaml 路径，由 filter_mask_server 加载
    declare_mask_yaml_cmd = DeclareLaunchArgument(
        'mask',
        default_value='map_jeck/keepout_mask.yaml',
        description='Nav2 filter_mask_server 使用的禁区掩膜 yaml 文件完整路径')

    # --------------------------------------------------------------------------
    # 定义 Nav2 核心生命周期节点
    # 加入 map_server；定位由 FAST-LIO2 提供，不使用 amcl
    # --------------------------------------------------------------------------
    lifecycle_nodes = ['map_server',
                       'controller_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother']

    # 掩膜层专属生命周期管理节点组（采用独立管理器解耦，保障 use_keepout:=false 时主导航不受影响）
    costmap_filter_lifecycle_nodes = ['filter_mask_server', 'costmap_filter_info_server']

    # 定位由 0915 双 EKF 定位系统提供（Global EKF: map→odom, Local EKF: odom→base_link），不使用 amcl
    # 注意：0915 定位框架 (ml_bringup/localization.launch.py) 已自带完整的 Local/Global 双 EKF 与 TF 权威发布，
    # 因此 nav2_bringup 不再重复启动 ekf_node，避免 TF 冲突与算力浪费。
    # map_server 负责发布静态 /map 话题
    
    # 配置文件动态重写（用于将 use_sim_time 等全局变量注入 yaml）
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)

    # --------------------------------------------------------------------------
    # 节点启动定义
    # --------------------------------------------------------------------------
    load_nodes = GroupAction(
        actions=[
            # 0. pointcloud_to_laserscan — 将3D点云降维为2D激光（在 base_link 坐标系截断，消除全局Z轴漂移影响）
            Node(
                package='pointcloud_to_laserscan',
                executable='pointcloud_to_laserscan_node',
                name='pointcloud_to_laserscan',
                output='screen',
                remappings=[
                    ('cloud_in', '/cloud_registered'),
                    ('scan',     '/scan'),
                ],
                parameters=[{
                    'use_sim_time': use_sim_time,  # 与定位和 costmap 共用回放时钟。
                    'target_frame':        'base_link',  # 0915 中 /cloud_registered_body 的 frame_id 为 base_link
                    'transform_tolerance':  0.1,
                    'min_height':  -0.10,
                    'max_height':  1.00,
                    'angle_min':       -3.14159,
                    'angle_max':        3.14159,
                    'angle_increment':  0.0175,  # 方案 F1：由 0.0087 (0.5°) 优化为 0.0175 (1.0°)，线束减半降低光线追踪算力
                    'range_min':  0.15,
                    'range_max':  10.0,
                    'scan_time':  0.1,
                    'use_inf':      True,
                    'inf_epsilon':  1.0,
                }]
            ),

            # 1. 静态地图服务器 - 从 yaml 文件发布 /map 话题
            # 定位由 0915 定位系统提供完整 TF 树
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[configured_params,
                            {'yaml_filename': map_yaml_file}]),

            # 1.1 禁区掩膜地图服务器 - 从 yaml 文件发布 /filter_mask 话题
            Node(
                condition=IfCondition(use_keepout),
                package='nav2_map_server',
                executable='map_server',
                name='filter_mask_server',
                output='screen',
                parameters=[configured_params,
                            {'yaml_filename': mask_yaml_file}]),

            # 1.2 掩膜元信息服务器 - 向 /costmap_filter_info 话题发布 Keepout 类型及转换参数
            Node(
                condition=IfCondition(use_keepout),
                package='nav2_map_server',
                executable='costmap_filter_info_server',
                name='costmap_filter_info_server',
                output='screen',
                parameters=[configured_params]),

            # 1. 控制器服务器 (负责局部路径跟随)
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                parameters=[configured_params],
                remappings=[('cmd_vel', 'cmd_vel_nav')]), # 将原始速度指令重映射给平滑器逻辑

            # 2. 规划器服务器 (负责全局路径规划)
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', 'nav2_costmap_2d:=DEBUG']
            ),
            # 3. 行为服务器 (负责卡住时的自动脱困及其他常规行为)
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[configured_params]),

            # 4. 行为树导航器 (负责管理导航逻辑流程)
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[configured_params]),

            # 5. 多点任务协调器
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[configured_params, {'use_sim_time': use_sim_time}]),

            # 6. 速度平滑器 (防止突变，保护底盘电机)
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=[configured_params],
                remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', '/cmd_vel')]),

            # 7. 主导航生命周期管理器 (负责按顺序启动并激活主导航核心节点)
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]),

            # 8. 掩膜层专用生命周期管理器 (解耦管理，负责按顺序启动并激活掩膜层服务节点)
            Node(
                condition=IfCondition(use_keepout),
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_costmap_filters',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': costmap_filter_lifecycle_nodes}]),
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_map_yaml_cmd)      # [静态地图] 路径参数
    ld.add_action(declare_use_keepout_cmd)   # [Keepout] 禁区掩膜开关参数
    ld.add_action(declare_mask_yaml_cmd)     # [Keepout] 禁区掩膜地图路径参数
    ld.add_action(load_nodes)

    return ld
