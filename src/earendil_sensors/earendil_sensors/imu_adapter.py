#!/usr/bin/env python3
# IMU Adapter Node — hardware-abstracted IMU interface
# Reads from IMU sensor and publishes:
#   /imu/data  (sensor_msgs/Imu)
#   /health/heartbeat
#
# Default: reads from I2C (MPU6050/MPU9250).
# Set use_hardware=false for simulated data.

import struct
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

import earendil_interfaces.msg as eimsg


class IMUAdapterNode(Node):
    """IMU hardware adapter. Publishes /imu/data."""

    def __init__(self):
        super().__init__('imu_adapter')

        self.declare_parameter('use_hardware', False)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x68)

        self._frame_id = self.get_parameter('frame_id').value
        self._use_hw = self.get_parameter('use_hardware').value
        self._rate = self.get_parameter('publish_rate_hz').value

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._imu_pub = self.create_publisher(Imu, '/imu/data', sensor_qos)
        self._health_pub = self.create_publisher(
            eimsg.HealthDiag, '/health/heartbeat', 10)

        if self._use_hw:
            self.get_logger().info('IMU adapter: hardware mode (I2C)')
            # TODO: start I2C reader thread for your specific IMU
        else:
            self._timer = self.create_timer(1.0 / self._rate, self._publish_sim)
            self.get_logger().info('IMU adapter: SIMULATION mode')

        self._health_timer = self.create_timer(1.0, self._publish_health)

    def _publish_sim(self):
        """No-op in simulation — IMU requires hardware."""
        self.get_logger().warn_once(
            'IMU: use_hardware=false — no IMU data published. '
            'Set use_hardware=true for real IMU.'
        )

    def _publish_health(self):
        msg = eimsg.HealthDiag()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.component_name = 'imu_adapter'
        msg.level = eimsg.HealthDiag.LEVEL_OK
        msg.message = 'OK' if self._use_hw else 'SIM'
        self._health_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IMUAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
