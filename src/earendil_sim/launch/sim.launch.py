# =============================================================================
# Optional Simulation Launch
# Launches Gazebo + real vehicle bringup with use_sim_time=true.
# Independent from main vehicle packages — only used for testing.
#
# Launch:
#   ros2 launch earendil_sim sim.launch.py
# =============================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty.sdf'),
        DeclareLaunchArgument('use_rviz', default_value='false'),

        # Gazebo simulation
        # TODO: Add Gazebo world and robot spawn when needed
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(...),
        # ),

        # Launch real vehicle bringup with sim overrides
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('earendil_bringup'),
                            'launch', 'vehicle.launch.py')),
                    launch_arguments={
                        'use_gps': 'false',
                        'use_hardware': 'false',
                    }.items(),
                ),
            ],
        ),
    ])
