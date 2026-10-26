"""Vehicle drivers and the map1012 localization chain in one launch process."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def include(package, relative_path, launch_arguments=None, condition=None):
    path = os.path.join(get_package_share_directory(package), relative_path)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=(launch_arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    names = (
        'map_directory', 'robot_mounts_file', 'manager_config',
        'relocalization_config', 'use_sim_time', 'rviz',
        'enable_yesense', 'manager_start_delay',
    )
    defaults = {
        'use_sim_time': 'false', 'rviz': 'false',
        'enable_yesense': 'true', 'manager_start_delay': '6.0',
    }
    arguments = [DeclareLaunchArgument(name, **(
        {'default_value': defaults[name]} if name in defaults else {}))
        for name in names]
    arguments.append(DeclareLaunchArgument('start_drivers', default_value='true'))
    drivers = IfCondition(LaunchConfiguration('start_drivers'))
    return LaunchDescription(arguments + [
        include('chassis_driver', 'launch/chassis_driver.launch.py', condition=drivers),
        include('livox_ros_driver2', 'launch_ROS2/msg_MID360s_launch.py', condition=drivers),
        include('yesense_std_ros2', 'launch/yesense_node.launch.py',
                condition=IfCondition(PythonExpression([
                    "'", LaunchConfiguration('start_drivers'), "' == 'true' and '",
                    LaunchConfiguration('enable_yesense'), "' == 'true'",
                ]))),
        include('ml_bringup', 'launch/localization.launch.py',
                {name: LaunchConfiguration(name) for name in names}),
    ])
