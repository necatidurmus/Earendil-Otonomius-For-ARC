"""Launch file for earendil_rscp_bridge — RSCP serial bridge node."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('earendil_rscp_bridge')
    params_file = os.path.join(pkg_share, 'config', 'rscp_bridge_params.yaml')

    return LaunchDescription([
        Node(
            package='earendil_rscp_bridge',
            executable='rscp_bridge_node.py',
            name='rscp_bridge',
            output='screen',
            parameters=[params_file],
            remappings=[],
        ),
    ])
