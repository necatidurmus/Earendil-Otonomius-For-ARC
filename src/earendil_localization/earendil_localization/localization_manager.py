"""
localization_manager.py

Lokalizasyon yöneticisi — Dual-UKF GPS+SLAM koordinasyonu.
Earendil-Otonomius v0.2 mimarisini baz alır.

Yönetilen bileşenler:
  - UKF Local  (odom + IMU → /odometry/local)
  - UKF Global (local + GPS → /odometry/filtered)
  - SLAM Toolbox
  - GPS Monitor (mod geçiş kararları)
  - TF Mode Relay

Yayımlanan:
  - /localization/mode_status [earendil_msgs/LocalizationMode]
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

from earendil_localization.gps_monitor import GpsQualityMonitor
from earendil_msgs.msg import LocalizationMode


class LocalizationManager(Node):
    """Lokalizasyon katmanı koordinasyon node'u."""

    def __init__(self):
        super().__init__('localization_manager')

        # Parametreler
        self.declare_parameter('gps_to_slam_threshold', 0.3)
        self.declare_parameter('slam_to_gps_threshold', 0.45)
        self.declare_parameter('window_size', 10)
        self.declare_parameter('hysteresis_secs', 3.0)
        self.declare_parameter('gps_timeout_secs', 3.0)
        self.declare_parameter('confirmation_count', 3)

        self._monitor = GpsQualityMonitor(
            gps_to_slam_threshold=self.get_parameter(
                'gps_to_slam_threshold').value,
            slam_to_gps_threshold=self.get_parameter(
                'slam_to_gps_threshold').value,
            window_size=self.get_parameter('window_size').value,
            hysteresis_secs=self.get_parameter('hysteresis_secs').value,
            gps_timeout_secs=self.get_parameter('gps_timeout_secs').value,
            confirmation_count=self.get_parameter('confirmation_count').value,
        )

        # Publishers
        self._mode_pub = self.create_publisher(
            LocalizationMode, '/localization/mode_status', 10)
        self._mode_str_pub = self.create_publisher(
            String, '/localization/mode', 10)

        # Subscribers
        self.create_subscription(NavSatFix, '/fix', self._gps_cb, 10)

        # Watchdog timer
        self.create_timer(1.0, self._watchdog_cb)

        self.get_logger().info('LocalizationManager başlatıldı.')

    def _gps_cb(self, msg: NavSatFix) -> None:
        hdop = 1.0  # TODO: NavSatFix'ten HDOP oku (varsa)
        new_mode = self._monitor.update_gps(msg.status.status, hdop)
        if new_mode:
            self.get_logger().info(f'Lokalizasyon modu değişti: {new_mode}')
            self._publish_mode()

    def _watchdog_cb(self) -> None:
        new_mode = self._monitor.update_timeout_check()
        if new_mode:
            self.get_logger().warn('GPS timeout — SLAM moduna geçildi')
        self._publish_mode()

    def _publish_mode(self) -> None:
        mode_str = self._monitor.current_mode
        quality = self._monitor.window_average()

        # LocalizationMode mesajı
        msg = LocalizationMode()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode_name = mode_str
        msg.mode = (LocalizationMode.GPS if mode_str == 'gps'
                    else LocalizationMode.SLAM)
        msg.gps_quality = quality
        msg.slam_active = (mode_str == 'slam')
        msg.ukf_healthy = True  # TODO: UKF sağlık kontrolü
        self._mode_pub.publish(msg)

        # String topic (mission manager için basit interface)
        str_msg = String()
        str_msg.data = mode_str
        self._mode_str_pub.publish(str_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
