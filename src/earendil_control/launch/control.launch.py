"""Motor control launch — STM bridge only.

Launches:
  - stm_bridge: /cmd_vel_safe → H723 ASCII + telemetry → ROS topics

Replaces legacy twist_mux + motor_adapter (now in legacy/).
safety_mux.py handles priority; stm_bridge is the sole motor writer.

Usage:
  ros2 launch earendil_control control.launch.py use_hardware:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('earendil_control')
    config_file = os.path.join(pkg_share, 'config', 'control_params.yaml')

    use_hardware = LaunchConfiguration('use_hardware', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('use_hardware', default_value='false'),

        Node(
            package='earendil_control',
            executable='stm_bridge.py',
            name='stm_bridge',
            output='screen',
            parameters=[
                config_file,
                {'use_sim_time': False},
                {'use_hardware': use_hardware},
            ],
        ),
    ])
