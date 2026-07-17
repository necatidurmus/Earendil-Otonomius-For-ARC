#!/usr/bin/env python3
"""
Diagnostics Node — sistem sağlık izleme.

H723 link durumu + heartbeat age, GPS fix kalitesi, LiDAR sağlık, battery (TBD).

Subscribes:
  /stm/status (earendil_interfaces/StmStatus) — H723 link durumu
  /stm/fault_flags (earendil_interfaces/StmFaultFlags) — F411 hataları
  /gps/fix (sensor_msgs/NavSatFix) — GPS fix kalitesi
  /rtk/status (std_msgs/String) — RTK fix durumu
  /scan (sensor_msgs/LaserScan) — LiDAR heartbeat

Publishes:
  /diagnostics (diagnostic_msgs/DiagnosticArray) — sistem sağlık durumu
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import NavSatFix, LaserScan
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from earendil_interfaces.msg import StmStatus, StmFaultFlags

SENSOR_QOS = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class DiagnosticsNode(Node):
    """Sistem sağlık izleme — STM, GPS, LiDAR durumunu derler."""

    def __init__(self):
        super().__init__('diagnostics_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('stm_timeout_ms', 500)
        self.declare_parameter('gps_timeout_ms', 2000)
        self.declare_parameter('lidar_timeout_ms', 1000)

        self._rate = self.get_parameter('publish_rate_hz').value
        self._stm_timeout = self.get_parameter('stm_timeout_ms').value / 1000.0
        self._gps_timeout = self.get_parameter('gps_timeout_ms').value / 1000.0
        self._lidar_timeout = self.get_parameter('lidar_timeout_ms').value / 1000.0

        # ── State ───────────────────────────────────────────────────────────
        self._last_stm_time = 0.0
        self._last_gps_time = 0.0
        self._last_lidar_time = 0.0
        self._stm_status = None
        self._fault_flags = None
        self._gps_fix = None
        self._rtk_status = 'NO_FIX'
        self._stm_link_active = False

        # ── Publishers ──────────────────────────────────────────────────────
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        # ── Subscribers ─────────────────────────────────────────────────────
        self.create_subscription(StmStatus, '/stm/status', self._on_stm, SENSOR_QOS)
        self.create_subscription(StmFaultFlags, '/stm/fault_flags', self._on_fault, SENSOR_QOS)
        self.create_subscription(NavSatFix, '/gps/fix', self._on_gps, SENSOR_QOS)
        self.create_subscription(String, '/rtk/status', self._on_rtk, SENSOR_QOS)
        self.create_subscription(LaserScan, '/scan', self._on_lidar, SENSOR_QOS)

        # ── Timer ───────────────────────────────────────────────────────────
        self._timer = self.create_timer(1.0 / self._rate, self._publish_diagnostics)

        self.get_logger().info(f'Diagnostics node started — rate={self._rate}Hz')

    # ── Callbacks ───────────────────────────────────────────────────────────

    def _on_stm(self, msg: StmStatus):
        self._stm_status = msg
        self._last_stm_time = time.time()
        self._stm_link_active = msg.link_active

    def _on_fault(self, msg: StmFaultFlags):
        self._fault_flags = msg

    def _on_gps(self, msg: NavSatFix):
        self._gps_fix = msg
        self._last_gps_time = time.time()

    def _on_rtk(self, msg: String):
        self._rtk_status = msg.data

    def _on_lidar(self, msg: LaserScan):
        self._last_lidar_time = time.time()

    # ── Diagnostics publisher ───────────────────────────────────────────────

    def _publish_diagnostics(self):
        now = time.time()
        stamp = self.get_clock().now().to_msg()

        diag = DiagnosticArray()
        diag.header.stamp = stamp
        diag.status = []

        # ── STM Status ──
        stm_st = DiagnosticStatus()
        stm_st.name = 'STM H723'
        stm_st.hardware_id = 'stm32h723'

        if self._last_stm_time > 0:
            age = now - self._last_stm_time
            if age > self._stm_timeout:
                stm_st.level = DiagnosticStatus.ERROR
                stm_st.message = f'STM link lost ({age:.1f}s)'
            elif not self._stm_link_active:
                stm_st.level = DiagnosticStatus.WARN
                stm_st.message = 'STM link inactive'
            else:
                stm_st.level = DiagnosticStatus.OK
                stm_st.message = 'STM link OK'
                if self._stm_status:
                    mode_map = {0: 'DISARMED', 1: 'MANUAL', 2: 'AUTONOMOUS'}
                    mode = mode_map.get(self._stm_status.operating_mode, 'UNKNOWN')
                    stm_st.values.append(KeyValue(key='mode', value=mode))
                    stm_st.values.append(KeyValue(
                        key='link_age_ms', value=f'{age*1000:.0f}'))
                    stm_st.values.append(KeyValue(
                        key='commands_sent', value=str(self._stm_status.commands_sent)))
                    stm_st.values.append(KeyValue(
                        key='acks_received', value=str(self._stm_status.acks_received)))
                    stm_st.values.append(KeyValue(
                        key='timeouts', value=str(self._stm_status.timeouts)))
        else:
            stm_st.level = DiagnosticStatus.WARN
            stm_st.message = 'No STM data received'

        diag.status.append(stm_st)

        # ── F411 Faults ──
        if self._fault_flags and self._fault_flags.any_fault:
            fault_st = DiagnosticStatus()
            fault_st.name = 'F411 Motors'
            fault_st.hardware_id = 'stm32f411'
            fault_st.level = DiagnosticStatus.ERROR
            fault_st.message = f'{self._fault_flags.fault_count} motor fault(s)'
            for i in range(4):
                if self._fault_flags.driver_error[i]:
                    fault_st.values.append(KeyValue(
                        key=f'motor_{i}_fault', value='ERROR'))
            diag.status.append(fault_st)

        # ── GPS Status ──
        gps_st = DiagnosticStatus()
        gps_st.name = 'RTK GPS'
        gps_st.hardware_id = 'gps'

        if self._last_gps_time > 0:
            age = now - self._last_gps_time
            if age > self._gps_timeout:
                gps_st.level = DiagnosticStatus.ERROR
                gps_st.message = f'GPS data lost ({age:.1f}s)'
            else:
                gps_st.level = DiagnosticStatus.OK
                gps_st.message = f'GPS OK ({self._rtk_status})'
                if self._gps_fix:
                    gps_st.values.append(KeyValue(
                        key='lat', value=f'{self._gps_fix.latitude:.6f}'))
                    gps_st.values.append(KeyValue(
                        key='lon', value=f'{self._gps_fix.longitude:.6f}'))
                    gps_st.values.append(KeyValue(
                        key='alt', value=f'{self._gps_fix.altitude:.1f}'))
                gps_st.values.append(KeyValue(key='rtk_status', value=self._rtk_status))
        else:
            gps_st.level = DiagnosticStatus.WARN
            gps_st.message = 'No GPS data received'

        diag.status.append(gps_st)

        # ── LiDAR Status ──
        lidar_st = DiagnosticStatus()
        lidar_st.name = 'LiDAR'
        lidar_st.hardware_id = 'lidar'

        if self._last_lidar_time > 0:
            age = now - self._last_lidar_time
            if age > self._lidar_timeout:
                lidar_st.level = DiagnosticStatus.ERROR
                lidar_st.message = f'LiDAR data lost ({age:.1f}s)'
            else:
                lidar_st.level = DiagnosticStatus.OK
                lidar_st.message = 'LiDAR OK'
        else:
            lidar_st.level = DiagnosticStatus.WARN
            lidar_st.message = 'No LiDAR data received'

        diag.status.append(lidar_st)

        self._diag_pub.publish(diag)


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
