#!/usr/bin/env python3
"""
gps_spoofer.py

Tünel ve GPS-denied bölgelerde GPS sinyalini simüle eder.
Earendil-Otonomius'taki tunnel_gps_spoofer.py'den modüler hale getirildi.

Davranış:
  - Rover tünel bölgesine girince: STATUS_NO_FIX yayınlar
  - Rover kapı bölgesindeyken: STATUS_FIX ama büyük hata (~5m)
  - Açık alanda: Normal GPS yayını devam eder

Konfigürasyon: sim_config.yaml → tunnel / door / gps_datum bölümleri
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry


class GpsSpoofer(Node):
    """Simülasyonda tünel bölgesinde GPS sinyalini taklit eden node."""

    def __init__(self):
        super().__init__('gps_spoofer')

        # Parametreler (sim_config.yaml'dan okunacak — şimdilik default)
        self.declare_parameter('tunnel.x_min', 19.5)
        self.declare_parameter('tunnel.x_max', 30.5)
        self.declare_parameter('tunnel.y_max', 1.35)
        self.declare_parameter('door.x_min', 33.5)
        self.declare_parameter('door.x_max', 36.5)
        self.declare_parameter('door.y_max', 1.0)

        self._tunnel_x_min = self.get_parameter('tunnel.x_min').value
        self._tunnel_x_max = self.get_parameter('tunnel.x_max').value
        self._tunnel_y_max = self.get_parameter('tunnel.y_max').value
        self._door_x_min = self.get_parameter('door.x_min').value
        self._door_x_max = self.get_parameter('door.x_max').value
        self._door_y_max = self.get_parameter('door.y_max').value

        self._robot_x = 0.0
        self._robot_y = 0.0

        # Odom subscribe (robot konumunu takip etmek için)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        # GPS mesajını yeniden yayınla (spoof edilmiş)
        self.create_subscription(NavSatFix, '/fix_raw', self._gps_cb, 10)
        self._gps_pub = self.create_publisher(NavSatFix, '/fix', 10)

        self.get_logger().info('GpsSpoofer başlatıldı.')

    def _odom_cb(self, msg: Odometry) -> None:
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y

    def _gps_cb(self, msg: NavSatFix) -> None:
        spoof = NavSatFix()
        spoof.header = msg.header

        if self._in_tunnel():
            # Tünel: GPS yok
            spoof.status.status = NavSatStatus.STATUS_NO_FIX
            spoof.latitude = 0.0
            spoof.longitude = 0.0
            spoof.altitude = 0.0
            self.get_logger().debug('GPS SPOOF: NO_FIX (tünel)')
        elif self._in_door():
            # Kapı: zayıf sinyal (~5m hata)
            spoof.status.status = NavSatStatus.STATUS_FIX
            spoof.latitude = msg.latitude
            spoof.longitude = msg.longitude
            spoof.altitude = msg.altitude
            spoof.position_covariance = [25.0, 0.0, 0.0,
                                         0.0, 25.0, 0.0,
                                         0.0, 0.0, 25.0]
            spoof.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            self.get_logger().debug('GPS SPOOF: WEAK_FIX (kapı bölgesi)')
        else:
            # Normal GPS
            spoof = msg

        self._gps_pub.publish(spoof)

    def _in_tunnel(self) -> bool:
        return (self._tunnel_x_min <= self._robot_x <= self._tunnel_x_max
                and abs(self._robot_y) <= self._tunnel_y_max)

    def _in_door(self) -> bool:
        return (self._door_x_min <= self._robot_x <= self._door_x_max
                and abs(self._robot_y) <= self._door_y_max)


def main(args=None):
    rclpy.init(args=args)
    node = GpsSpoofer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
