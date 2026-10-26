import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition


def generate_launch_description():
    share_dir = get_package_share_directory('lio_sam')
    parameter_file = LaunchConfiguration('params_file')
    xacro_path = os.path.join(share_dir, 'config', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(share_dir, 'config', 'relocalization.rviz')
    if not os.path.exists(rviz_config_file):
        rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'config', 'params.yaml'),
        description='Path to the ROS2 parameters file to use.'
    )

    # Headless switch: on embedded targets (e.g., Jetson Xavier NX) run with use_rviz:=false
    # to avoid RViz2's GPU/CPU cost; visualize topics from a remote PC instead.
    use_rviz_declare = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz2 (set false for headless deployment).'
    )

    use_lidar_driver_declare = DeclareLaunchArgument(
        'use_lidar_driver',
        default_value='true',
        description='是否自动启动 Livox Mid360 雷达驱动节点'
    )

    livox_share = get_package_share_directory('livox_ros_driver2')
    user_config_path = os.path.join(livox_share, 'config', 'MID360_config.json')

    livox_driver_node = Node(
        condition=IfCondition(LaunchConfiguration('use_lidar_driver')),
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

    return LaunchDescription([
        params_declare,
        use_rviz_declare,
        use_lidar_driver_declare,
        livox_driver_node,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(Command(['xacro', ' ', xacro_path]), value_type=str)
            }]
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_relocalization',
            name='lio_sam_relocalization',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_sc_pose',
            name='lio_sam_sc_pose',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        )
    ])
