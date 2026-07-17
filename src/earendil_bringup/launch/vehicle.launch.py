# =============================================================================
# Earendil Vehicle — Main Launch File (Real Hardware)
# =============================================================================
# This is the PRIMARY launch file for the real vehicle on RPi5.
# All sim_time defaults to false. No Gazebo, no spoofer, no RViz.
#
# Launch:
#   ros2 launch earendil_bringup vehicle.launch.py
#
# Arguments:
#   use_gps:=true|false
#   use_lidar:=true|false
#   use_web:=true|false
#   use_navigation:=true|false
#   use_hardware:=true|false  (motor driver serial)

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Package directories
    pkg_bringup = get_package_share_directory('earendil_bringup')
    pkg_desc = get_package_share_directory('earendil_description')

    # URDF
    urdf_file = os.path.join(pkg_desc, 'urdf', 'earendil_vehicle.urdf.xacro')
    robot_description = Command(['xacro ', urdf_file])

    # Arguments
    use_gps = LaunchConfiguration('use_gps', default='true')
    use_lidar = LaunchConfiguration('use_lidar', default='true')
    use_web = LaunchConfiguration('use_web', default='true')
    use_navigation = LaunchConfiguration('use_navigation', default='true')
    use_hardware = LaunchConfiguration('use_hardware', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('use_gps', default_value='true'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_web', default_value='true'),
        DeclareLaunchArgument('use_navigation', default_value='true'),
        DeclareLaunchArgument('use_hardware', default_value='false'),
        DeclareLaunchArgument('waypoints_file', default_value=''),

        # ── Robot Description (always) ──────────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(robot_description, value_type=str),
                'publish_frequency': 30.0,
            }],
        ),

        # ── Safety System (always — MANDATORY) ──────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('earendil_safety'),
                    'launch', 'safety.launch.py')),
        ),

        # ── Motor Control — STM bridge (always) ─────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('earendil_control'),
                    'launch', 'control.launch.py')),
            launch_arguments={'use_hardware': use_hardware}.items(),
        ),

        # ── RSCP Bridge — Competition Module serial link ────────────
        Node(
            package='earendil_rscp_bridge',
            executable='rscp_bridge_node.py',
            name='rscp_bridge',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('earendil_rscp_bridge'),
                'config', 'rscp_bridge_params.yaml')],
        ),

        # ── Sensors ─────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('earendil_sensors'),
                    'launch', 'sensors.launch.py')),
            launch_arguments={
                'use_gps': use_gps,
                'use_lidar': use_lidar,
            }.items(),
        ),

        # ── Localization (delayed start for sensor warmup) ──────────
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='robot_localization',
                    executable='ukf_node',
                    name='ukf_local_node',
                    output='screen',
                    parameters=[
                        os.path.join(pkg_bringup, 'config', 'ukf_local.yaml'),
                        {'use_sim_time': False},
                    ],
                    remappings=[('odometry/filtered', '/odometry/local')],
                ),
            ],
        ),

        # ── GPS + NavSat + GPS Monitor (delayed) ───────────────────
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='robot_localization',
                    executable='ukf_node',
                    name='ukf_global_node',
                    output='screen',
                    parameters=[
                        os.path.join(pkg_bringup, 'config', 'ukf_global.yaml'),
                        {'use_sim_time': False},
                    ],
                    remappings=[('odometry/filtered', '/odometry/global')],
                    condition=IfCondition(use_gps),
                ),
                Node(
                    package='robot_localization',
                    executable='navsat_transform_node',
                    name='navsat_transform',
                    output='screen',
                    parameters=[
                        os.path.join(pkg_bringup, 'config', 'navsat.yaml'),
                        {'use_sim_time': False},
                    ],
                    remappings=[
                        ('imu', '/stm/imu/data'),
                        ('gps/fix', '/gps/fix'),
                        ('gps/filtered', '/gps/filtered'),
                        ('odometry/gps', '/odometry/gps'),
                        ('odometry/filtered', '/odometry/local'),
                    ],
                    condition=IfCondition(use_gps),
                ),
                Node(
                    package='earendil_navigation',
                    executable='gps_monitor.py',
                    name='gps_monitor',
                    output='screen',
                    condition=IfCondition(use_gps),
                ),
                Node(
                    package='earendil_navigation',
                    executable='tf_mode_relay.py',
                    name='tf_mode_relay',
                    output='screen',
                ),
            ],
        ),

        # ── Nav2 (delayed start) ────────────────────────────────────
        TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('nav2_bringup'),
                            'launch', 'navigation_launch.py')),
                    launch_arguments={
                        'params_file': os.path.join(
                            pkg_bringup, 'config', 'nav2_params.yaml'),
                        'use_sim_time': 'false',
                    }.items(),
                    condition=IfCondition(use_navigation),
                ),
            ],
        ),

        # ── Mission Manager (delayed) ───────────────────────────────
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='earendil_navigation',
                    executable='mission_manager.py',
                    name='mission_manager',
                    output='screen',
                    condition=IfCondition(use_navigation),
                ),
            ],
        ),

        # ── Web Dashboard ───────────────────────────────────────────
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='earendil_web',
                    executable='web_server.py',
                    name='web_dashboard',
                    output='screen',
                    condition=IfCondition(use_web),
                ),
            ],
        ),
    ])
