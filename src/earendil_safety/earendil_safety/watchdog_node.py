#!/usr/bin/env python3
"""
Watchdog Node — STM ve RSCP heartbeat izleme.

STM heartbeat: /stm/status telemetri yaşı belirli süreyi aşarsa alarm.
RSCP heartbeat: /rscp/status bağlantı yaşı belirli süreyi aşarsa alarm.

Herhangi biri timeout aşarsa → /safety/watchdog_alarm topic'inde True publish.

Subscribes:
  /stm/status (earendil_interfaces/StmStatus)
  /rscp/status (earendil_interfaces/RscpStatus)

Publishes:
  /safety/watchdog_alarm (std_msgs/Bool) — watchdog alarm durumu
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool

from earendil_interfaces.msg import StmStatus, RscpStatus

SENSOR_QOS = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class WatchdogNode(Node):
    """Heartbeat watchdog — STM ve RSCP bağlantı sağlığını izler."""

    def __init__(self):
        super().__init__('watchdog_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('stm_timeout_ms', 400)
        self.declare_parameter('rscp_timeout_ms', 2000)
        self.declare_parameter('check_period_ms', 100)

        self._stm_timeout = self.get_parameter('stm_timeout_ms').value / 1000.0
        self._rscp_timeout = self.get_parameter('rscp_timeout_ms').value / 1000.0
        check_period = self.get_parameter('check_period_ms').value / 1000.0

        # ── State ───────────────────────────────────────────────────────────
        self._last_stm_time = 0.0
        self._last_rscp_time = 0.0
        self._stm_ok = True
        self._rscp_ok = True

        # ── Publishers ──────────────────────────────────────────────────────
        self._alarm_pub = self.create_publisher(Bool, '/safety/watchdog_alarm', 10)

        # ── Subscribers ─────────────────────────────────────────────────────
        self.create_subscription(StmStatus, '/stm/status', self._on_stm, SENSOR_QOS)
        self.create_subscription(RscpStatus, '/rscp/status', self._on_rscp, SENSOR_QOS)

        # ── Timer ───────────────────────────────────────────────────────────
        self._check_timer = self.create_timer(check_period, self._check)

        self.get_logger().info(
            f'Watchdog started — stm_timeout={self._stm_timeout*1000:.0f}ms, '
            f'rscp_timeout={self._rscp_timeout*1000:.0f}ms'
        )

    def _on_stm(self, msg: StmStatus):
        self._last_stm_time = time.time()

    def _on_rscp(self, msg: RscpStatus):
        self._last_rscp_time = time.time()

    def _check(self):
        now = time.time()
        alarm = False

        # STM check — fail-closed: no message ever = not OK
        if self._last_stm_time > 0:
            stm_age = now - self._last_stm_time
            self._stm_ok = stm_age < self._stm_timeout
        else:
            self._stm_ok = False
        if not self._stm_ok:
            alarm = True
            if self._last_stm_time > 0:
                stm_age = now - self._last_stm_time
                self.get_logger().warn(
                    f'STM heartbeat lost — age={stm_age*1000:.0f}ms '
                    f'(timeout={self._stm_timeout*1000:.0f}ms)'
                )
            else:
                self.get_logger().warn('STM heartbeat never received')

        # RSCP check — fail-closed: no message ever = not OK
        if self._last_rscp_time > 0:
            rscp_age = now - self._last_rscp_time
            self._rscp_ok = rscp_age < self._rscp_timeout
        else:
            self._rscp_ok = False
        if not self._rscp_ok:
            alarm = True
            if self._last_rscp_time > 0:
                rscp_age = now - self._last_rscp_time
                self.get_logger().warn(
                    f'RSCP heartbeat lost — age={rscp_age*1000:.0f}ms '
                    f'(timeout={self._rscp_timeout*1000:.0f}ms)'
                )
            else:
                self.get_logger().warn('RSCP heartbeat never received')

        # Publish alarm
        msg = Bool()
        msg.data = alarm
        self._alarm_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
