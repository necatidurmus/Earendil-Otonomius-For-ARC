"""
sim_full.launch.py

Gazebo simülasyonu başlatır ve rover'ı spawn eder.
Earendil-Otonomius'taki leo_gz.launch.py + spawn_robot.launch.py mantığından ilham alındı.

Dünyalar:
  - arc_arena   : ARC yarışması arena benzeri (açık alan + tünel)
  - arc_tunnel  : Sadece tünel geçiş testi
  - arc_empty   : Boş dünya (geliştirme için)

Kullanım:
  ros2 launch earendil_simulation sim_full.launch.py
  ros2 launch earendil_simulation sim_full.launch.py world:=arc_tunnel
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_pkg = get_package_share_directory('earendil_simulation')
    desc_pkg = get_package_share_directory('earendil_description')

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='arc_arena',
        description='Gazebo world: arc_arena | arc_tunnel | arc_empty',
    )
    declare_gui = DeclareLaunchArgument(
        'gz_gui',
        default_value='true',
        description='Launch Gazebo GUI',
    )

    world = LaunchConfiguration('world')

    # Gazebo launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch', 'gz_sim.launch.py',
            ])
        ),
        launch_arguments={
            'gz_args': PathJoinSubstitution([
                sim_pkg, 'worlds',
                [world, '.sdf'],
            ]),
        }.items(),
    )

    # ROS-Gazebo bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        output='screen',
    )

    # Robot State Publisher
    urdf_file = os.path.join(desc_pkg, 'urdf', 'earendil.urdf.xacro')
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': open(urdf_file).read()
                if os.path.exists(urdf_file) else '',
        }],
        output='screen',
    )

    # GPS Spoofer (tünel GPS simülasyonu)
    gps_spoofer = Node(
        package='earendil_simulation',
        executable='gps_spoofer.py',
        name='gps_spoofer',
        output='screen',
    )

    return LaunchDescription([
        declare_world,
        declare_gui,
        gz_sim,
        ros_gz_bridge,
        robot_state_publisher,
        gps_spoofer,
    ])
