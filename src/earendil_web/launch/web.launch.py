"""Launch file for earendil_web — Web dashboard and control panel."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('earendil_web')
    params_file = os.path.join(pkg_share, 'config', 'web_params.yaml')

    return LaunchDescription([
        Node(
            package='earendil_web',
            executable='web_server.py',
            name='web_dashboard',
            output='screen',
            parameters=[params_file],
            remappings=[],
        ),
    ])
