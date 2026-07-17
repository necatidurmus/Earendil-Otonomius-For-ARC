"""Static TF tree for Earendil vehicle.

Frames (REP-105):
  odom → base_link (from stm_bridge wheel_odom)
  base_link → imu_link (statik)
  base_link → lidar_link (statik)
  base_link → gps_link (statik)
  base_link → wheel_FL, wheel_FR, wheel_RL, wheel_RR (statik)

Kullanım:
  ros2 launch earendil_bringup tf.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ── base_link → imu_link ──
        # STM MPU9250 montaj pozisyonu (TBD — sahada ölç)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_imu',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.1',
                '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
                '--frame-id', 'base_link', '--child-frame-id', 'imu_link',
            ],
        ),

        # ── base_link → lidar_link ──
        # LiDAR montaj pozisyonu (TBD — sahada ölç)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_lidar',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.3',
                '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
                '--frame-id', 'base_link', '--child-frame-id', 'lidar_link',
            ],
        ),

        # ── base_link → gps_link ──
        # RTK GPS anteni montaj pozisyonu (TBD — sahada ölç)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_gps',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.4',
                '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
                '--frame-id', 'base_link', '--child-frame-id', 'gps_link',
            ],
        ),

        # ── base_link → wheel_FL ──
        # Sol ön tekerlek (TBD — sahada ölç)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_wheel_fl',
            arguments=[
                '--x', '0.4125', '--y', '0.55', '--z', '-0.1',
                '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
                '--frame-id', 'base_link', '--child-frame-id', 'wheel_FL',
            ],
        ),

        # ── base_link → wheel_FR ──
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_wheel_fr',
            arguments=[
                '--x', '0.4125', '--y', '-0.55', '--z', '-0.1',
                '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
                '--frame-id', 'base_link', '--child-frame-id', 'wheel_FR',
            ],
        ),

        # ── base_link → wheel_RL ──
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_wheel_rl',
            arguments=[
                '--x', '-0.4125', '--y', '0.55', '--z', '-0.1',
                '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
                '--frame-id', 'base_link', '--child-frame-id', 'wheel_RL',
            ],
        ),

        # ── base_link → wheel_RR ──
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_wheel_rr',
            arguments=[
                '--x', '-0.4125', '--y', '-0.55', '--z', '-0.1',
                '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
                '--frame-id', 'base_link', '--child-frame-id', 'wheel_RR',
            ],
        ),
    ])
