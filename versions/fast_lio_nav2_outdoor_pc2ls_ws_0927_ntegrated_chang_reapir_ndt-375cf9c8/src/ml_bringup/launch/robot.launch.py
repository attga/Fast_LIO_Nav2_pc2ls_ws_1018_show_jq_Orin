"""Start the existing hardware and localization launches together, without Nav2."""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
    LogInfo, OpaqueFunction, RegisterEventHandler, TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def stop_on_failure(event, context):
    if event.returncode != 0 and not context.is_shutdown:
        reason = f'Process {event.process_name} exited with code {event.returncode}'
        return [LogInfo(msg=reason), EmitEvent(event=Shutdown(reason=reason))]
    return []


def launch_robot(context):
    map_directory = os.path.abspath(os.path.expanduser(
        LaunchConfiguration('map_directory').perform(context)))
    for filename in ('map.pcd', 'isam_poses.txt', 'sc_db.bin'):
        path = os.path.join(map_directory, filename)
        if not os.path.isfile(path) or not os.access(path, os.R_OK):
            raise RuntimeError(f'Localization map file missing or unreadable: {path}')
    if not os.path.isdir(os.path.join(map_directory, 'Scans')):
        raise RuntimeError(f'Localization Scans directory missing: {map_directory}')

    imu_script = os.path.join(get_package_prefix('ml_bringup'),
                              'lib', 'ml_bringup', 'imu_measurements.py')
    if not os.path.isfile(imu_script) or not os.access(imu_script, os.X_OK):
        raise RuntimeError(
            f'IMU executable missing or not executable: {imu_script}. '
            'Rebuild ml_bringup with the updated CMakeLists.txt, then source install/setup.bash.')

    hardware_specifications = [
        ('chassis_driver', 'chassis_driver.launch.py', {}),
        ('livox_ros_driver2', 'msg_MID360s_launch.py', {}),
        ('yesense_std_ros2', 'yesense_node.launch.py', {}),
    ]
    actions = []
    # Resolve all hardware launches before any driver starts.
    for package, filename, arguments in hardware_specifications:
        share = get_package_share_directory(package)
        # Livox installs launch_ROS2; some existing underlays use launch instead.
        directories = ('launch_ROS2', 'launch') if package == 'livox_ros_driver2' else ('launch',)
        candidates = [os.path.join(share, directory, filename) for directory in directories]
        path = next((candidate for candidate in candidates if os.path.isfile(candidate)), None)
        if path is None:
            raise RuntimeError(f'Installed launch file missing; checked: {candidates}')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(path), launch_arguments=arguments.items()))

    # 关键时序编排：延时 3.5s 启动定位系统，留出足够时间让 Livox MID-360 完成驱动配置与队列排空
    loc_share = get_package_share_directory('ml_bringup')
    loc_path = os.path.join(loc_share, 'launch', 'localization.launch.py')
    loc_args = {
        'map_directory': map_directory,
        'robot_mounts_file': LaunchConfiguration('robot_mounts_file').perform(context),
        'use_sim_time': 'false',
        'enable_yesense': 'true',
        'lidar_mount_yaw': LaunchConfiguration('lidar_mount_yaw').perform(context),
        'rviz': LaunchConfiguration('rviz').perform(context),
        'imu_mount': LaunchConfiguration('imu_mount').perform(context),
        'wheel_mount': LaunchConfiguration('wheel_mount').perform(context),
    }
    startup_delay = float(LaunchConfiguration('startup_delay').perform(context))
    loc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(loc_path), launch_arguments=loc_args.items())

    if startup_delay > 0.0:
        actions.append(LogInfo(
            msg=f'[robot.launch] 硬件驱动已并发拉起，编排延时 {startup_delay:.1f}s 等待雷达/IMU建链与队列稳定...'))
        actions.append(TimerAction(
            period=startup_delay,
            actions=[
                LogInfo(msg=f'[robot.launch] {startup_delay:.1f}s 硬件预热完成，正式启动定位系统...'),
                loc_launch
            ]
        ))
    else:
        actions.append(LogInfo(msg='[robot.launch] 延时为 0，立即并发启动定位系统...'))
        actions.append(loc_launch)
    return actions


def generate_launch_description():
    share = get_package_share_directory('ml_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('map_directory', description='Localization map directory'),
        DeclareLaunchArgument('robot_mounts_file', default_value=os.path.join(share, 'config', 'robot', 'rear_axle.yaml')),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        # Current vehicle: the whole MID360 was rotated left 90 degrees after mapping.
        DeclareLaunchArgument('lidar_mount_yaw', default_value=''),
        DeclareLaunchArgument('imu_mount', default_value=''),
        DeclareLaunchArgument('wheel_mount', default_value=''),
        DeclareLaunchArgument('startup_delay', default_value='3.5',
                              description='Delay before launching localization stack (seconds)'),
        RegisterEventHandler(OnProcessExit(on_exit=stop_on_failure)),
        OpaqueFunction(function=launch_robot),
    ])
