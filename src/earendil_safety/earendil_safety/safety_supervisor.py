"""
safety_supervisor.py

Merkezi safety supervisor — tüm actuator komutları buradan geçer.
cmd_vel_unsafe → [filtre] → cmd_vel

Kontroller:
  - Watchdog: komut timeout'u (rover durur)
  - E-Stop: /e_stop topic'i (anlık dur)
  - Tilt limiti: IMU'dan aşırı eğim tespiti
  - Batarya: minimum batarya yüzdesi (stub)
"""
import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, String


class SafetySupervisor(Node):
    """
    Merkezi safety supervisor node.
    Tüm cmd_vel komutları bu node üzerinden geçer.
    """

    def __init__(self):
        super().__init__('safety_supervisor')

        # Parametreler
        self.declare_parameter('watchdog_timeout_secs', 2.0)
        self.declare_parameter('max_tilt_deg', 30.0)
        self.declare_parameter('min_battery_pct', 15.0)
        self.declare_parameter('status_rate_hz', 2.0)

        self._watchdog_timeout = self.get_parameter(
            'watchdog_timeout_secs').value
        self._max_tilt_rad = math.radians(
            self.get_parameter('max_tilt_deg').value)
        self._min_battery = self.get_parameter('min_battery_pct').value

        self._e_stop_active = False
        self._tilt_fault = False
        self._battery_fault = False
        self._last_cmd_time: Optional[float] = None

        # Publishers
        self._cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)
        self._status_pub = self.create_publisher(
            String, '/safety/status', 10)

        # Subscribers
        self.create_subscription(
            Twist, '/cmd_vel_unsafe', self._cmd_cb, 10)
        self.create_subscription(
            Bool, '/e_stop', self._estop_cb, 10)
        self.create_subscription(
            Imu, '/imu/data', self._imu_cb, 10)
        self.create_subscription(
            Float32, '/safety/battery_pct', self._battery_cb, 10)

        rate = self.get_parameter('status_rate_hz').value
        self.create_timer(1.0 / rate, self._status_cb)
        self.create_timer(0.1, self._watchdog_check)

        self.get_logger().info('SafetySupervisor başlatıldı.')

    def _cmd_cb(self, msg: Twist) -> None:
        """Gelen cmd_vel'i filtreler ve güvenli ise /cmd_vel'e iletir."""
        self._last_cmd_time = time.monotonic()

        if self._any_fault():
            self._publish_zero()
            return

        self._cmd_pub.publish(msg)

    def _estop_cb(self, msg: Bool) -> None:
        if msg.data and not self._e_stop_active:
            self.get_logger().error('E-STOP AKTİF!')
            self._e_stop_active = True
            self._publish_zero()
        elif not msg.data and self._e_stop_active:
            self.get_logger().info('E-stop kaldırıldı.')
            self._e_stop_active = False

    def _imu_cb(self, msg: Imu) -> None:
        """Tilt açısını IMU'dan hesaplar."""
        q = msg.orientation
        # Roll (x-axis) tilt hesaplama
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = abs(math.atan2(sinr_cosp, cosr_cosp))

        # Pitch (y-axis) tilt hesaplama
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = abs(math.asin(max(-1.0, min(1.0, sinp))))

        tilt = max(roll, pitch)
        if tilt > self._max_tilt_rad:
            if not self._tilt_fault:
                self.get_logger().error(
                    f'TILT HATI! {math.degrees(tilt):.1f}° > '
                    f'{math.degrees(self._max_tilt_rad):.1f}°')
                self._tilt_fault = True
        else:
            self._tilt_fault = False

    def _battery_cb(self, msg: Float32) -> None:
        if msg.data < self._min_battery:
            if not self._battery_fault:
                self.get_logger().error(
                    f'DÜŞÜK BATARYA! %{msg.data:.1f} < %{self._min_battery}')
                self._battery_fault = True
        else:
            self._battery_fault = False

    def _watchdog_check(self) -> None:
        """Komut timeout'u kontrolü."""
        if self._last_cmd_time is None:
            return
        elapsed = time.monotonic() - self._last_cmd_time
        if elapsed > self._watchdog_timeout:
            if not self._any_fault():
                self.get_logger().warn(
                    f'Watchdog timeout ({elapsed:.1f}s) — Rover durduruluyor.')
            self._publish_zero()

    def _status_cb(self) -> None:
        """Safety durumunu yayınlar."""
        if self._e_stop_active:
            status = 'EMERGENCY_STOP'
        elif self._tilt_fault:
            status = 'TILT_FAULT'
        elif self._battery_fault:
            status = 'LOW_BATTERY'
        else:
            status = 'OK'
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)

    def _any_fault(self) -> bool:
        return self._e_stop_active or self._tilt_fault or self._battery_fault

    def _publish_zero(self) -> None:
        zero = Twist()
        self._cmd_pub.publish(zero)


def main(args=None):
    rclpy.init(args=args)
    node = SafetySupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
