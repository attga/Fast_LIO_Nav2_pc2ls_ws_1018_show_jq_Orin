"""Start the vehicle drivers and the LIO-SAM relocalization chain."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def include(package, relative_path, launch_arguments=None, condition=None):
    # Source-tree launches must also use the source-tree child launches.
    package_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    source_dir = package_dir if package == 'lio_sam' else os.path.join(
        os.path.dirname(package_dir), package)
    path = os.path.join(source_dir, relative_path)
    if not os.path.isfile(path):
        path = os.path.join(get_package_share_directory(package), relative_path)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=(launch_arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    package_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    drivers = IfCondition(LaunchConfiguration('start_drivers'))
    yesense = IfCondition(PythonExpression([
        "'", LaunchConfiguration('start_drivers'), "' == 'true' and '",
        LaunchConfiguration('enable_yesense'), "' == 'true'",
    ]))
    return LaunchDescription([
        DeclareLaunchArgument('start_drivers', default_value='true'),
        DeclareLaunchArgument('enable_yesense', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            package_dir, 'config', 'params.yaml')),
        DeclareLaunchArgument('map_data_dir', default_value=os.path.join(
            package_dir, 'data')),
        include('chassis_driver', 'launch/chassis_driver.launch.py', condition=drivers),
        include('livox_ros_driver2', 'launch_ROS2/msg_MID360s_launch.py', condition=drivers),
        include('yesense_std_ros2', 'launch/yesense_node.launch.py', condition=yesense),
        include('lio_sam', 'launch/relocalization.launch.py', {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_rviz': LaunchConfiguration('rviz'),
            'map_data_dir': LaunchConfiguration('map_data_dir'),
            'params_file': LaunchConfiguration('params_file'),
        }),
    ])
