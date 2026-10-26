from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_directory', description='地图目录：包含 sc_db.bin、优化位姿和 Scans'),
        DeclareLaunchArgument('pose_file', default_value='isam_poses.txt'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('matching_frame', default_value='base_link',
                              description='匹配传感器坐标系；不同于 base_frame 时需提前发布安装 TF'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='ml_relocalization', executable='relocalization_node',
             name='alaserRelocalizer', output='screen', parameters=[
                 PathJoinSubstitution([FindPackageShare('ml_relocalization'), 'config', 'relocalization.yaml']),
                 {'map_directory': LaunchConfiguration('map_directory'),
                  'pose_file': LaunchConfiguration('pose_file'),
                  'base_frame': LaunchConfiguration('base_frame'),
                  'matching_frame': LaunchConfiguration('matching_frame'),
                  'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
    ])
