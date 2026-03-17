"""
mission.launch.py

Mission stack başlatır:
  - Mission Manager
  - RSCP Bridge
  - Safety Supervisor
  - Session Logger
  - Teleop
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_auto_start = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='Auto-start mission without RSCP START command',
    )

    mission_manager = Node(
        package='earendil_mission',
        executable='mission_manager_node.py',
        name='mission_manager',
        output='screen',
        parameters=[{
            'auto_start': LaunchConfiguration('auto_start'),
        }],
    )

    rscp_bridge = Node(
        package='earendil_rscp',
        executable='rscp_bridge_node.py',
        name='rscp_bridge',
        output='screen',
    )

    safety_supervisor = Node(
        package='earendil_safety',
        executable='safety_supervisor_node.py',
        name='safety_supervisor',
        output='screen',
    )

    session_logger = Node(
        package='earendil_logging',
        executable='session_logger_node.py',
        name='session_logger',
        output='screen',
    )

    return LaunchDescription([
        declare_auto_start,
        mission_manager,
        rscp_bridge,
        safety_supervisor,
        session_logger,
    ])
