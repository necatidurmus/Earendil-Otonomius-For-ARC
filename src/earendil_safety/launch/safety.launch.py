"""Safety system launch — safety_mux + watchdog."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('earendil_safety')
    config_file = os.path.join(pkg_dir, 'config', 'safety_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config_file),
        DeclareLaunchArgument('arm_required', default_value='true'),

        Node(
            package='earendil_safety',
            executable='safety_mux.py',
            name='safety_mux',
            output='screen',
            parameters=[
                LaunchConfiguration('config'),
                {'arm_required': LaunchConfiguration('arm_required')},
            ],
        ),
        Node(
            package='earendil_safety',
            executable='watchdog_node.py',
            name='watchdog_node',
            output='screen',
            parameters=[LaunchConfiguration('config')],
        ),
    ])
