"""Launch file for stm_bridge — cmd_vel → H723 RPM + telemetry."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('earendil_control')
    control_params = os.path.join(pkg_share, 'config', 'control_params.yaml')
    vehicle_params = os.path.join(pkg_share, 'config', 'vehicle.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_hardware', default_value='false'),
        DeclareLaunchArgument('config', default_value=control_params),

        Node(
            package='earendil_control',
            executable='stm_bridge.py',
            name='stm_bridge',
            output='screen',
            parameters=[
                LaunchConfiguration('config'),
                {'use_hardware': LaunchConfiguration('use_hardware')},
            ],
        ),
    ])
