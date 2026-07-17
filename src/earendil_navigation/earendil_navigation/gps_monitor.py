#!/usr/bin/env python3
"""
GPS Quality Monitor — RTK quality tracking and navigation mode switching.

Monitors GPS fix quality from /gps/fix and /rtk/status.
Publishes /nav_mode (GPS/SLAM) for tf_mode_relay.
Per CLAUDE.md §8: FIXED→güvenilir; FLOAT/DGPS→40cm tolerans; SPS/NO_FIX→varış onayı verilmez.

Subscribes:
  /gps/fix (sensor_msgs/NavSatFix)
  /rtk/status (std_msgs/String)

Publishes:
  /nav_mode (std_msgs/String) — GPS or SLAM
  /gps_quality (std_msgs/Float32) — quality 0-1
  /rtk_fix_level (std_msgs/String) — FIXED/FLOAT/DGPS/SPS/NO_FIX
"""

import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Float32


# RTK status → quality weight
RTK_QUALITY_MAP = {
    'RTK_FIXED': 1.0,
    'FIXED': 1.0,
    'RTK_FLOAT': 0.5,
    'FLOAT': 0.5,
    'DGPS': 0.4,
    'SPS': 0.2,
    'NO_FIX': 0.0,
    'UNKNOWN': 0.0,
}


class GPSMonitor(Node):
    """GPS quality monitor with RTK-aware mode switching."""

    def __init__(self):
        super().__init__('gps_monitor')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('gps_to_slam_threshold', 0.3)
        self.declare_parameter('slam_to_gps_threshold', 0.6)
        self.declare_parameter('window_size', 10)
        self.declare_parameter('hysteresis_secs', 3.0)
        self.declare_parameter('gps_timeout_secs', 3.0)
        self.declare_parameter('confirmation_count', 3)
        self.declare_parameter('fix_topic', '/gps/fix')
        self.declare_parameter('rtk_status_topic', '/rtk/status')
        self.declare_parameter('verbose', True)

        self._thr_down = self.get_parameter('gps_to_slam_threshold').value
        self._thr_up = self.get_parameter('slam_to_gps_threshold').value
        self._window_size = self.get_parameter('window_size').value
        self._hysteresis = self.get_parameter('hysteresis_secs').value
        self._gps_timeout = self.get_parameter('gps_timeout_secs').value
        self._confirm_count = self.get_parameter('confirmation_count').value
        self._verbose = self.get_parameter('verbose').value

        # ── State ───────────────────────────────────────────────────────────
        self._current_mode = 'GPS'
        self._quality_window = deque(maxlen=self._window_size)
        self._last_switch_time = self.get_clock().now()
        self._last_navsat_time = 0.0
        self._pending_mode = None
        self._confirm_counter = 0
        self._rtk_status = 'NO_FIX'

        # ── Publishers ──────────────────────────────────────────────────────
        self._mode_pub = self.create_publisher(String, '/nav_mode', 10)
        self._quality_pub = self.create_publisher(Float32, '/gps_quality', 10)
        self._rtk_pub = self.create_publisher(String, '/rtk_fix_level', 10)

        # ── Subscribers ─────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        fix_topic = self.get_parameter('fix_topic').value
        self.create_subscription(NavSatFix, fix_topic, self._on_navsat, sensor_qos)

        rtk_topic = self.get_parameter('rtk_status_topic').value
        self.create_subscription(String, rtk_topic, self._on_rtk_status, sensor_qos)

        # ── Timers ──────────────────────────────────────────────────────────
        self.create_timer(0.2, self._publish_mode)

        self.get_logger().info(
            f'GPS Monitor started — thresholds: down={self._thr_down}, up={self._thr_up}'
        )

    def _on_rtk_status(self, msg: String):
        self._rtk_status = msg.data
        self._rtk_pub.publish(msg)

    def _on_navsat(self, msg: NavSatFix):
        self._last_navsat_time = time.time()

        # Quality heuristic: combine covariance + RTK status
        quality = 0.0

        # Base quality from covariance
        if msg.status.status >= 0:  # STATUS_FIX or better
            quality = 0.5
            if len(msg.position_covariance) >= 1:
                cov = msg.position_covariance[0]
                if cov > 0:
                    quality = max(0.0, 1.0 - cov / 100.0)

        # RTK status modifier
        rtk_quality = RTK_QUALITY_MAP.get(self._rtk_status, 0.0)
        quality = max(quality, rtk_quality)

        self._quality_window.append(quality)

        if len(self._quality_window) < 3:
            return

        avg_quality = sum(self._quality_window) / len(self._quality_window)
        self._quality_pub.publish(Float32(data=avg_quality))

        # Mode switching logic
        now = self.get_clock().now()
        elapsed = (now - self._last_switch_time).nanoseconds / 1e9

        if elapsed < self._hysteresis:
            return

        if self._current_mode == 'GPS' and avg_quality < self._thr_down:
            self._try_switch('SLAM')
        elif self._current_mode == 'SLAM' and avg_quality > self._thr_up:
            self._try_switch('GPS')

    def _try_switch(self, new_mode: str):
        if self._pending_mode == new_mode:
            self._confirm_counter += 1
        else:
            self._pending_mode = new_mode
            self._confirm_counter = 1

        if self._confirm_counter >= self._confirm_count:
            self._current_mode = new_mode
            self._last_switch_time = self.get_clock().now()
            self._pending_mode = None
            self._confirm_counter = 0
            self.get_logger().info(f'Nav mode switched to {new_mode}')

    def _publish_mode(self):
        # Check GPS timeout
        if self._last_navsat_time > 0:
            age = time.time() - self._last_navsat_time
            if age > self._gps_timeout and self._current_mode == 'GPS':
                self._current_mode = 'SLAM'
                self.get_logger().warn(
                    f'GPS timeout ({age:.1f}s) — switching to SLAM')

        self._mode_pub.publish(String(data=self._current_mode))


def main(args=None):
    rclpy.init(args=args)
    node = GPSMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
