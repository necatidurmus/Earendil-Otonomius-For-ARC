"""Mission Manager launch — RSCP stage-aware ARC 2026 mission orchestrator.

Usage:
  ros2 launch earendil_navigation mission.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('earendil_navigation')
    mission_config = os.path.join(pkg_share, 'config', 'mission_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=mission_config),

        Node(
            package='earendil_navigation',
            executable='mission_manager.py',
            name='mission_manager',
            output='screen',
            parameters=[LaunchConfiguration('config')],
        ),
    ])
