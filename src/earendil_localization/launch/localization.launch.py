"""
localization.launch.py

Lokalizasyon stack'ini başlatır:
  - UKF Local  (robot_localization: odom + IMU)
  - UKF Global (robot_localization: local + GPS)
  - NavSat Transform
  - SLAM Toolbox (async mode)
  - GPS Monitor node
  - TF Mode Relay node
  - Localization Manager

Earendil-Otonomius'taki navigation_hybrid.launch.py'den modüler edildi.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('earendil_localization')

    ukf_local_config = os.path.join(pkg, 'config', 'ukf_local.yaml')
    ukf_global_config = os.path.join(pkg, 'config', 'ukf_global.yaml')
    navsat_config = os.path.join(pkg, 'config', 'navsat.yaml')
    slam_config = os.path.join(pkg, 'config', 'slam_toolbox_params.yaml')

    # UKF Local (odom + IMU füzyonu)
    ukf_local = Node(
        package='robot_localization',
        executable='ukf_node',
        name='ukf_local',
        output='screen',
        parameters=[ukf_local_config],
        remappings=[
            ('odometry/filtered', 'odometry/local'),
        ],
    )

    # NavSat Transform (GPS → odom frame)
    navsat = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='robot_localization',
                executable='navsat_transform_node',
                name='navsat_transform',
                output='screen',
                parameters=[navsat_config],
                remappings=[
                    ('imu/data', '/imu/data'),
                    ('gps/fix', '/fix'),
                    ('odometry/filtered', '/odometry/local'),
                ],
            )
        ],
    )

    # UKF Global (local + GPS füzyonu)
    ukf_global = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='robot_localization',
                executable='ukf_node',
                name='ukf_global',
                output='screen',
                parameters=[ukf_global_config],
                remappings=[
                    ('odometry/filtered', 'odometry/filtered'),
                ],
            )
        ],
    )

    # SLAM Toolbox
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config],
    )

    # GPS Monitor
    gps_monitor = Node(
        package='earendil_localization',
        executable='gps_monitor_node.py',
        name='gps_monitor',
        output='screen',
    )

    # TF Mode Relay
    tf_relay = Node(
        package='earendil_localization',
        executable='tf_mode_relay_node.py',
        name='tf_mode_relay',
        output='screen',
    )

    # Localization Manager
    loc_manager = Node(
        package='earendil_localization',
        executable='localization_manager_node.py',
        name='localization_manager',
        output='screen',
    )

    return LaunchDescription([
        ukf_local,
        ukf_global,
        navsat,
        slam_toolbox,
        gps_monitor,
        tf_relay,
        loc_manager,
    ])
