# =============================================================================
# Earendil Vehicle — Modular Sensor Test Launch
# =============================================================================
# Her sensör ve localization bileşeni bağımsız toggle ile açılıp kapatılabilir.
# Adım adım test: manyeto+GPS → +IMU → +LiDAR → +kamera
#
# Kullanım:
#   ros2 launch earendil_bringup test_profile.launch.py use_gps:=true use_navsat:=true
#   ros2 launch earendil_bringup test_profile.launch.py use_gps:=true use_navsat:=true use_ukf_local:=true
#   ros2 launch earendil_bringup test_profile.launch.py use_gps:=true use_navsat:=true use_ukf_local:=true use_lidar:=true use_ukf_global:=true
#
# Toggle'lar:
#   use_gps        — GPS adapter node (LC25H EA)
#   use_lidar      — LiDAR adapter node (D800/LD06)
#   use_imu        — STM IMU verisi (stm_bridge zaten publish eder, flag bilgi amaçlı)
#   use_mag        — STM manyetometre verisi (stm_bridge zaten publish eder, flag bilgi amaçlı)
#   use_camera     — Stereo kamera (M8'de, şimdilik placeholder)
#   use_navsat     — navsat_transform + gps_monitor (GPS→map dönüşümü)
#   use_ukf_local  — UKF local (wheel_odom + IMU → odom→base_link TF)
#   use_ukf_global — UKF global (local + GPS → map→odom TF)
#   use_diagnostics— Diagnostics node (sistem sağlık izleme)
#   use_hardware   — Gerçek donanım modu (false = simülasyon)

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
    pkg_sensors = get_package_share_directory('earendil_sensors')
    pkg_desc = get_package_share_directory('earendil_description')
    pkg_safety = get_package_share_directory('earendil_safety')
    pkg_control = get_package_share_directory('earendil_control')

    # URDF
    urdf_file = os.path.join(pkg_desc, 'urdf', 'earendil_vehicle.urdf.xacro')
    robot_description = Command(['xacro ', urdf_file])

    # Launch configurations
    use_gps = LaunchConfiguration('use_gps', default='false')
    use_lidar = LaunchConfiguration('use_lidar', default='false')
    use_imu = LaunchConfiguration('use_imu', default='false')
    use_mag = LaunchConfiguration('use_mag', default='false')
    use_camera = LaunchConfiguration('use_camera', default='false')
    use_navsat = LaunchConfiguration('use_navsat', default='false')
    use_ukf_local = LaunchConfiguration('use_ukf_local', default='false')
    use_ukf_global = LaunchConfiguration('use_ukf_global', default='false')
    use_diagnostics = LaunchConfiguration('use_diagnostics', default='true')
    use_hardware = LaunchConfiguration('use_hardware', default='false')

    # Config file
    config_file = os.path.join(pkg_sensors, 'config', 'sensor_params.yaml')

    return LaunchDescription([
        # ── Arguments ────────────────────────────────────────────────
        DeclareLaunchArgument('use_gps', default_value='false',
            description='GPS adapter node (LC25H EA)'),
        DeclareLaunchArgument('use_lidar', default_value='false',
            description='LiDAR adapter node (D800/LD06)'),
        DeclareLaunchArgument('use_imu', default_value='false',
            description='STM IMU verisi (stm_bridge publish eder)'),
        DeclareLaunchArgument('use_mag', default_value='false',
            description='STM manyetometre verisi (stm_bridge publish eder)'),
        DeclareLaunchArgument('use_camera', default_value='false',
            description='Stereo kamera (M8 placeholder)'),
        DeclareLaunchArgument('use_navsat', default_value='false',
            description='navsat_transform + gps_monitor'),
        DeclareLaunchArgument('use_ukf_local', default_value='false',
            description='UKF local (wheel_odom + IMU fusion)'),
        DeclareLaunchArgument('use_ukf_global', default_value='false',
            description='UKF global (local + GPS fusion)'),
        DeclareLaunchArgument('use_diagnostics', default_value='true',
            description='Diagnostics node'),
        DeclareLaunchArgument('use_hardware', default_value='false',
            description='Gerçek donanım modu'),

        # ── Robot Description (her zaman) ────────────────────────────
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

        # ── Static TF'ler (her zaman) ───────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'tf.launch.py')),
        ),

        # ── Safety System (her zaman — MANDATORY) ───────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_safety, 'launch', 'safety.launch.py')),
        ),

        # ── Motor Control — STM bridge (her zaman) ──────────────────
        # stm_bridge hem motor komutu hem telemetri (IMU, mag, odom) publish eder
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_control, 'launch', 'control.launch.py')),
            launch_arguments={'use_hardware': use_hardware}.items(),
        ),

        # ── Sensör Node'ları (koşullu) ──────────────────────────────

        # GPS Adapter
        Node(
            package='earendil_sensors',
            executable='gps_adapter.py',
            name='gps_adapter',
            output='screen',
            parameters=[config_file, {'use_hardware': use_hardware}],
            condition=IfCondition(use_gps),
        ),

        # LiDAR Adapter
        Node(
            package='earendil_sensors',
            executable='lidar_adapter.py',
            name='lidar_adapter',
            output='screen',
            parameters=[config_file, {'use_hardware': use_hardware}],
            condition=IfCondition(use_lidar),
        ),

        # Diagnostics
        Node(
            package='earendil_sensors',
            executable='diagnostics_node.py',
            name='diagnostics_node',
            output='screen',
            parameters=[config_file],
            condition=IfCondition(use_diagnostics),
        ),

        # ── Localization Node'ları (koşullu, gecikmeli) ─────────────

        # navsat_transform + gps_monitor (GPS varsa anlamlı)
        TimerAction(
            period=3.0,
            actions=[
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
                    condition=IfCondition(use_navsat),
                ),
                Node(
                    package='earendil_navigation',
                    executable='gps_monitor.py',
                    name='gps_monitor',
                    output='screen',
                    condition=IfCondition(use_navsat),
                ),
            ],
        ),

        # UKF Local (wheel_odom + IMU → odom→base_link)
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
                    condition=IfCondition(use_ukf_local),
                ),
            ],
        ),

        # UKF Global (local + GPS → map→odom)
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
                    condition=IfCondition(use_ukf_global),
                ),
                Node(
                    package='earendil_navigation',
                    executable='tf_mode_relay.py',
                    name='tf_mode_relay',
                    output='screen',
                    condition=IfCondition(use_ukf_global),
                ),
            ],
        ),
    ])
