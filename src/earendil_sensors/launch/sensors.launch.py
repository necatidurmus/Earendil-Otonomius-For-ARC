# Sensor adapter launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('earendil_sensors')
    config_file = os.path.join(pkg_dir, 'config', 'sensor_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config_file),
        DeclareLaunchArgument('use_gps', default_value='true'),
        DeclareLaunchArgument('use_imu', default_value='true'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_encoder', default_value='true'),

        Node(
            package='earendil_sensors',
            executable='gps_adapter.py',
            name='gps_adapter',
            output='screen',
            parameters=[LaunchConfiguration('config')],
            condition=IfCondition(LaunchConfiguration('use_gps')),
        ),
        Node(
            package='earendil_sensors',
            executable='imu_adapter.py',
            name='imu_adapter',
            output='screen',
            parameters=[LaunchConfiguration('config')],
            condition=IfCondition(LaunchConfiguration('use_imu')),
        ),
        Node(
            package='earendil_sensors',
            executable='lidar_adapter.py',
            name='lidar_adapter',
            output='screen',
            parameters=[LaunchConfiguration('config')],
            condition=IfCondition(LaunchConfiguration('use_lidar')),
        ),
        Node(
            package='earendil_sensors',
            executable='encoder_adapter.py',
            name='encoder_adapter',
            output='screen',
            parameters=[LaunchConfiguration('config')],
            condition=IfCondition(LaunchConfiguration('use_encoder')),
        ),
    ])
