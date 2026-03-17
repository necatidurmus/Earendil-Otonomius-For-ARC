#!/usr/bin/env python3
"""gps_monitor_node.py — GPS Monitor standalone node (localization_manager içinde de çalışır)."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Float32

from earendil_localization.gps_monitor import GpsQualityMonitor


class GpsMonitorNode(Node):
    """GPS kalite izleme node'u — standalone çalıştırılabilir."""

    def __init__(self):
        super().__init__('gps_monitor')

        self.declare_parameter('gps_to_slam_threshold', 0.3)
        self.declare_parameter('slam_to_gps_threshold', 0.45)
        self.declare_parameter('window_size', 10)
        self.declare_parameter('hysteresis_secs', 3.0)
        self.declare_parameter('gps_timeout_secs', 3.0)
        self.declare_parameter('confirmation_count', 3)
        self.declare_parameter('verbose', True)

        self._verbose = self.get_parameter('verbose').value
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

        self._mode_pub = self.create_publisher(
            String, '/localization/mode', 10)
        self._quality_pub = self.create_publisher(
            Float32, '/localization/gps_quality', 10)

        self.create_subscription(NavSatFix, '/fix', self._gps_cb, 10)
        self.create_timer(0.5, self._timer_cb)

        self.get_logger().info('GpsMonitorNode başlatıldı.')

    def _gps_cb(self, msg: NavSatFix) -> None:
        new_mode = self._monitor.update_gps(msg.status.status)
        if new_mode and self._verbose:
            self.get_logger().info(
                f'GPS Monitor: mod → {new_mode} '
                f'(kalite={self._monitor.window_average():.3f})')
        self._publish()

    def _timer_cb(self) -> None:
        self._monitor.update_timeout_check()
        self._publish()

    def _publish(self) -> None:
        mode_msg = String()
        mode_msg.data = self._monitor.current_mode
        self._mode_pub.publish(mode_msg)

        quality_msg = Float32()
        quality_msg.data = float(self._monitor.window_average())
        self._quality_pub.publish(quality_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GpsMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
