"""Localization launch — RTK + STM odom + STM IMU fusion.

Two-stage EKF:
  1. ukf_local: /stm/wheel_odom + /stm/imu/data → /odometry/local (odom → base_link)
  2. navsat_transform: /gps/fix + /odometry/local → /odometry/gps
  3. ukf_global: /odometry/local + /odometry/gps → /odometry/global (map → odom)

Usage:
  ros2 launch earendil_bringup localization.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('earendil_bringup')

    ukf_local_config = os.path.join(pkg_share, 'config', 'ukf_local.yaml')
    ukf_global_config = os.path.join(pkg_share, 'config', 'ukf_global.yaml')
    navsat_config = os.path.join(pkg_share, 'config', 'navsat.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('ukf_local_config', default_value=ukf_local_config),
        DeclareLaunchArgument('ukf_global_config', default_value=ukf_global_config),
        DeclareLaunchArgument('navsat_config', default_value=navsat_config),

        # ── Local EKF: wheel_odom + IMU → odom → base_link ──
        Node(
            package='robot_localization',
            executable='ukf_node',
            name='ukf_local',
            output='screen',
            parameters=[LaunchConfiguration('ukf_local_config')],
            remappings=[
                ('odometry/filtered', '/odometry/local'),
            ],
        ),

        # ── NavSat Transform: GPS + local odom → /odometry/gps ──
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[LaunchConfiguration('navsat_config')],
            remappings=[
                ('imu', '/stm/imu/data'),
                ('gps/fix', '/gps/fix'),
                ('gps/filtered', '/gps/filtered'),
                ('odometry/gps', '/odometry/gps'),
                ('odometry/filtered', '/odometry/local'),
            ],
        ),

        # ── Global EKF: local odom + GPS → map → odom ──
        Node(
            package='robot_localization',
            executable='ukf_node',
            name='ukf_global',
            output='screen',
            parameters=[LaunchConfiguration('ukf_global_config')],
            remappings=[
                ('odometry/filtered', '/odometry/global'),
            ],
        ),
    ])
