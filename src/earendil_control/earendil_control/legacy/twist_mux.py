#!/usr/bin/env python3
# Twist Mux — priority-based velocity command multiplexer
# Replaces ad-hoc direct /cmd_vel publishing.
#
# Input sources (highest priority first):
#   1. /cmd_vel_emergency  (emergency override)
#   2. /cmd_vel_manual     (web/teleop deadman)
#   3. /cmd_vel_nav        (Nav2 autonomous)
#
# Output: /cmd_vel_pre_safety → goes to safety_gate → /cmd_vel

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist


class TwistMuxNode(Node):
    """Priority-based velocity multiplexer."""

    SOURCES = [
        ('emergency', '/cmd_vel_emergency', 0),
        ('manual', '/cmd_vel_manual', 1),
        ('nav', '/cmd_vel_nav', 2),
    ]

    def __init__(self):
        super().__init__('twist_mux')

        self.declare_parameter('output_topic', '/cmd_vel_pre_safety')
        self.declare_parameter('timeout_sec', 0.5)

        self._output_topic = self.get_parameter('output_topic').value
        self._timeout = self.get_parameter('timeout_sec').value

        self._last_msg = {}
        self._last_time = {}

        cmd_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self._subs = {}
        for name, topic, prio in self.SOURCES:
            self._subs[name] = self.create_subscription(
                Twist, topic, self._make_cb(name), cmd_qos)

        self._pub = self.create_publisher(Twist, self._output_topic, cmd_qos)
        self._timer = self.create_timer(0.01, self._select)  # 100Hz

        self.get_logger().info(f'TwistMux: output={self._output_topic}')

    def _make_cb(self, name):
        def cb(msg):
            self._last_msg[name] = msg
            self._last_time[name] = time.time()
        return cb

    def _select(self):
        now = time.time()
        output = Twist()

        for name, _, _ in self.SOURCES:
            t = self._last_time.get(name, 0.0)
            if t > 0 and (now - t) < self._timeout:
                output = self._last_msg[name]
                break

        self._pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = TwistMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
