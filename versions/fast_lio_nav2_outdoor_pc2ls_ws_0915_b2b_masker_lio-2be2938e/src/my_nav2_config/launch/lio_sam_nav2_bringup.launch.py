import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition


def generate_launch_description():
    nav2_share = get_package_share_directory('my_nav2_config')
    lio_sam_share = get_package_share_directory('lio_sam')

    # --------------------------------------------------------------------------
    # 参数定义
    # --------------------------------------------------------------------------
    map_yaml_file = LaunchConfiguration('map')
    mask_yaml_file = LaunchConfiguration('mask')
    use_keepout = LaunchConfiguration('use_keepout')
    params_file = LaunchConfiguration('params_file')
    lio_params_file = LaunchConfiguration('lio_params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(nav2_share, 'maps', 'lio_sam_map', 'map_edited_cleaned.yaml'),
        description='静态 2D 栅格地图 yaml 文件的完整路径')

    declare_mask_cmd = DeclareLaunchArgument(
        'mask',
        default_value=os.path.join(nav2_share, 'maps', 'lio_sam_map', 'keepout_mask.yaml'),
        description='禁区掩膜 2D 栅格地图 yaml 文件的完整路径')

    declare_use_keepout_cmd = DeclareLaunchArgument(
        'use_keepout',
        default_value='true',
        description='是否开启禁区掩膜层 (Keepout Zone)')

    declare_nav2_params_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav2_share, 'config', 'nav2_params.yaml'),
        description='Nav2 参数配置文件路径')

    declare_lio_params_cmd = DeclareLaunchArgument(
        'lio_params_file',
        default_value=os.path.join(lio_sam_share, 'config', 'params.yaml'),
        description='LIO-SAM 参数配置文件路径')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='是否自动打开 RViz2')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时间')

    # --------------------------------------------------------------------------
    # 1. 机器人 URDF 与 TF (robot_state_publisher)
    # 提供 base_link -> livox_frame / imu_link 静态坐标转换
    # --------------------------------------------------------------------------
    xacro_path = os.path.join(lio_sam_share, 'config', 'robot.urdf.xacro')
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro', ' ', xacro_path]), value_type=str),
            'use_sim_time': use_sim_time
        }]
    )

    # --------------------------------------------------------------------------
    # 2. LIO-SAM 定位节点组 (lio_sam_sc_pose + lio_sam_relocalization)
    # 负责输出 map->odom 与 odom->base_link 高频 100Hz TF 及 /local_lidar_odom
    # --------------------------------------------------------------------------
    lio_sam_reloc_node = Node(
        package='lio_sam',
        executable='lio_sam_relocalization',
        name='lio_sam_relocalization',
        parameters=[lio_params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    lio_sam_sc_pose_node = Node(
        package='lio_sam',
        executable='lio_sam_sc_pose',
        name='lio_sam_sc_pose',
        parameters=[lio_params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # --------------------------------------------------------------------------
    # 3. Nav2 核心导航栈 (Include nav2_bringup.launch.py)
    # --------------------------------------------------------------------------
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'nav2_bringup.launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'mask': mask_yaml_file,
            'use_keepout': use_keepout,
            'params_file': params_file,
            'use_ekf': 'false',  # 由 LIO-SAM 托管 TF，禁用 EKF 避免冲突
            'cloud_topic': '/livox/lidar',
            'use_sim_time': use_sim_time
        }.items()
    )

    # --------------------------------------------------------------------------
    # 4. RViz2 可视化
    # --------------------------------------------------------------------------
    rviz_config_file = os.path.join(lio_sam_share, 'config', 'relocalization.rviz')
    if not os.path.exists(rviz_config_file):
        rviz_config_file = os.path.join(lio_sam_share, 'config', 'rviz2.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    use_lidar_driver = LaunchConfiguration('use_lidar_driver')
    declare_use_lidar_driver_cmd = DeclareLaunchArgument(
        'use_lidar_driver',
        default_value='true',
        description='是否自动启动 Livox Mid360 雷达驱动节点')

    livox_share = get_package_share_directory('livox_ros_driver2')
    user_config_path = os.path.join(livox_share, 'config', 'MID360_config.json')

    livox_driver_node = Node(
        condition=IfCondition(use_lidar_driver),
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[
            {"xfer_format": 1},
            {"multi_topic": 0},
            {"data_src": 0},
            {"publish_freq": 10.0},
            {"output_data_type": 0},
            {"frame_id": 'livox_frame'},
            {"user_config_path": user_config_path},
            {"cmdline_input_bd_code": ''}
        ]
    )

    # --------------------------------------------------------------------------
    # 构建 LaunchDescription
    # --------------------------------------------------------------------------
    ld = LaunchDescription()
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_mask_cmd)
    ld.add_action(declare_use_keepout_cmd)
    ld.add_action(declare_nav2_params_cmd)
    ld.add_action(declare_lio_params_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_lidar_driver_cmd)

    # 启动节点组
    ld.add_action(livox_driver_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(lio_sam_sc_pose_node)
    ld.add_action(lio_sam_reloc_node)
    ld.add_action(nav2_bringup_launch)
    ld.add_action(rviz_node)

    return ld
