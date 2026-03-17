"""
robot_bringup.launch.py

Gerçek rover bringup — SADECE GERÇEK DONANIM İÇİN.
Simülasyon için sim_bringup.launch.py kullanın.

Durum: STUB — Faz 4'te tamamlanacak.
Şu an placeholder olarak bırakılmıştır.

Eklenecekler (Faz 4):
  - GPS sürücüsü (U-blox / NMEA)
  - IMU sürücüsü (BNO055 / ICM-42688)
  - LiDAR sürücüsü (RPLiDAR vb.)
  - Kamera sürücüsü
  - Motor kontrol sürücüsü
  - Tüm navigation/mission stack'i (sim ile aynı)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    # TODO (Faz 4): Gerçek donanım sürücüleri eklenecek
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='earendil',
            description='Robot namespace',
        ),
        # Placeholder: Gerçek bringup Faz 4'te uygulanacak
    ])
