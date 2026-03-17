"""
sim_bringup.launch.py

Tam simülasyon stack'ini başlatır:
  1. Gazebo (arc_arena.sdf dünyası)
  2. Robot spawning
  3. Localization (UKF + GPS Monitor + TF Relay + SLAM)
  4. Navigation (Nav2)
  5. Mission Manager
  6. Safety Supervisor
  7. RSCP Bridge
  8. Session Logger
  9. RViz (opsiyonel)

Kullanım:
  ros2 launch earendil_bringup sim_bringup.launch.py
  ros2 launch earendil_bringup sim_bringup.launch.py launch_rviz:=true
  ros2 launch earendil_bringup sim_bringup.launch.py world:=arc_tunnel
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── Launch arguments ─────────────────────────────────────────────
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='arc_arena',
        description='Gazebo world name: arc_arena | arc_tunnel | arc_empty',
    )
    declare_rviz = DeclareLaunchArgument(
        'launch_rviz',
        default_value='false',
        description='Launch RViz visualization',
    )

    world = LaunchConfiguration('world')
    launch_rviz = LaunchConfiguration('launch_rviz')

    # ── Package share directories ─────────────────────────────────────
    sim_pkg = get_package_share_directory('earendil_simulation')
    loc_pkg = get_package_share_directory('earendil_localization')
    nav_pkg = get_package_share_directory('earendil_navigation')
    mission_pkg = get_package_share_directory('earendil_mission')

    # ── Sub-launches ──────────────────────────────────────────────────
    # 1. Simulation (Gazebo + robot spawn)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg, 'launch', 'sim_full.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    # 2. Localization stack (delayed: Gazebo hazır olduktan sonra)
    localization_launch = TimerAction(
        period=25.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(loc_pkg, 'launch', 'localization.launch.py')
                ),
            )
        ],
    )

    # 3. Navigation (Nav2 — localization hazır olduktan sonra)
    navigation_launch = TimerAction(
        period=35.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_pkg, 'launch', 'navigation.launch.py')
                ),
            )
        ],
    )

    # 4. Mission Manager (Nav2 hazır olduktan sonra)
    mission_launch = TimerAction(
        period=50.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(mission_pkg, 'launch', 'mission.launch.py')
                ),
            )
        ],
    )

    return LaunchDescription([
        declare_world,
        declare_rviz,
        sim_launch,
        localization_launch,
        navigation_launch,
        mission_launch,
    ])
