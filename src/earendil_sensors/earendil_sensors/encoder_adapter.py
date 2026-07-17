#!/usr/bin/env python3
# Encoder Adapter Node — hardware-abstracted wheel encoder interface
# Reads wheel encoder data and publishes:
#   /wheel_states  (custom or nav_msgs/Odometry)
#   /health/heartbeat

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

import earendil_interfaces.msg as eimsg
import math


class EncoderAdapterNode(Node):
    """Wheel encoder adapter. Publishes raw encoder odometry."""

    def __init__(self):
        super().__init__('encoder_adapter')

        self.declare_parameter('use_hardware', False)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('wheel_base', 0.825)
        self.declare_parameter('wheel_radius', 0.125)
        self.declare_parameter('encoder_topic', '/wheel_odom')
        self.declare_parameter('frame_id', 'base_link')

        self._use_hw = self.get_parameter('use_hardware').value
        self._rate = self.get_parameter('publish_rate_hz').value

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._odom_pub = self.create_publisher(Odometry, '/wheel_odom', sensor_qos)
        self._health_pub = self.create_publisher(
            eimsg.HealthDiag, '/health/heartbeat', 10)

        if self._use_hw:
            self.get_logger().info('Encoder adapter: hardware mode')
            # TODO: start hardware encoder reader
        else:
            self._timer = self.create_timer(1.0 / self._rate, self._publish_sim)
            self.get_logger().info('Encoder adapter: SIMULATION mode')

        self._health_timer = self.create_timer(1.0, self._publish_health)

    def _publish_sim(self):
        """No-op in simulation — encoders require hardware."""
        self.get_logger().warn_once(
            'Encoder: use_hardware=false — no encoder data published. '
            'Set use_hardware=true for real encoders.'
        )

    def _publish_health(self):
        msg = eimsg.HealthDiag()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.component_name = 'encoder_adapter'
        msg.level = eimsg.HealthDiag.LEVEL_OK
        msg.message = 'SIM' if not self._use_hw else 'OK'
        self._health_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
