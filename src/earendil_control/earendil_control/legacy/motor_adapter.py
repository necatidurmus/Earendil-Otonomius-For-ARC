#!/usr/bin/env python3
# Motor Adapter Node — abstract motor driver interface
# Receives /cmd_vel (from safety gate) and translates to hardware-specific commands.
#
# This is an ADAPTER: change only this node when switching motor hardware.
# The rest of the pipeline (safety, mux, nav2) stays unchanged.
#
# Default implementation: serial protocol for common motor controllers.
# Replace the _send_to_hardware() method for your specific driver.

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

import earendil_interfaces.msg as eimsg


class MotorAdapterNode(Node):
    """Abstract motor driver adapter.

    Subscribes to /cmd_vel (after safety gate) and:
      1. Translates to hardware commands
      2. Reads encoder feedback
      3. Publishes /odom and /wheel_states
    """

    def __init__(self):
        super().__init__('motor_adapter')

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('wheel_base', 0.38)          # m (distance between wheel centers)
        self.declare_parameter('wheel_radius', 0.0625)       # m
        self.declare_parameter('encoder_ticks_per_rev', 1024)
        self.declare_parameter('max_speed_mps', 0.5)         # m/s
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_odom_tf', True)
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('use_hardware', False)  # True = real serial, False = simulated

        self._wheel_base = self.get_parameter('wheel_base').value
        self._wheel_radius = self.get_parameter('wheel_radius').value
        self._max_speed = self.get_parameter('max_speed_mps').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._publish_tf = self.get_parameter('publish_odom_tf').value
        self._use_hardware = self.get_parameter('use_hardware').value

        # ── State ───────────────────────────────────────────────────
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last_time = self.get_clock().now()
        self._vx = 0.0
        self._vth = 0.0

        # ── Subscribers ─────────────────────────────────────────────
        cmd_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            self._on_cmd_vel,
            cmd_qos,
        )

        # ── Publishers ──────────────────────────────────────────────
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._health_pub = self.create_publisher(
            eimsg.HealthDiag, '/health/heartbeat', 10)

        # ── TF broadcaster (optional) ───────────────────────────────
        if self._publish_tf:
            import tf2_ros
            self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── Serial connection (if hardware) ─────────────────────────
        self._serial = None
        if self._use_hardware:
            self._connect_serial()

        # ── Odometry timer at 50Hz ──────────────────────────────────
        self._odom_timer = self.create_timer(0.02, self._publish_odom)
        # ── Health heartbeat at 2Hz ─────────────────────────────────
        self._health_timer = self.create_timer(0.5, self._publish_health)

        self.get_logger().info('Motor adapter started')
        if not self._use_hardware:
            self.get_logger().info('Running in SIMULATION mode (no hardware)')

    def _connect_serial(self):
        try:
            import serial
            port = self.get_parameter('serial_port').value
            baud = self.get_parameter('serial_baud').value
            self._serial = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Serial connected: {port}@{baud}')
        except Exception as e:
            self.get_logger().error(f'Serial connection failed: {e}')
            self._serial = None

    def _on_cmd_vel(self, msg: Twist):
        """Translate /cmd_vel to motor commands."""
        linear = msg.linear.x
        angular = msg.angular.z

        self._vx = linear
        self._vth = angular

        if self._use_hardware and self._serial:
            self._send_to_hardware(linear, angular)

    def _send_to_hardware(self, linear: float, angular: float):
        """Send velocity commands to motor controller via serial.

        Protocol: simple text-based, replace with your hardware protocol.
        Format: "V <left_speed> <right_speed>\n"
        Speeds in m/s.
        """
        # Differential drive kinematics
        v_left = linear - (angular * self._wheel_base / 2.0)
        v_right = linear + (angular * self._wheel_base / 2.0)

        # Clamp
        v_left = max(-self._max_speed, min(self._max_speed, v_left))
        v_right = max(-self._max_speed, min(self._max_speed, v_right))

        try:
            cmd = f'V {v_left:.3f} {v_right:.3f}\n'
            self._serial.write(cmd.encode())
        except Exception as e:
            self.get_logger().error(f'Serial write failed: {e}')

    def _publish_odom(self):
        """Publish odometry based on commanded velocities (open-loop).

        Replace with encoder feedback for closed-loop control.
        """
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now
        if dt <= 0 or dt > 1.0:
            return

        # Simple dead-reckoning (replace with encoder data)
        self._x += self._vx * math.cos(self._theta) * dt
        self._y += self._vx * math.sin(self._theta) * dt
        self._theta += self._vth * dt

        # Normalize theta
        self._theta = math.atan2(math.sin(self._theta), math.cos(self._theta))

        # Publish odom
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = math.sin(self._theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self._theta / 2.0)
        odom.twist.twist.linear.x = self._vx
        odom.twist.twist.angular.z = self._vth
        self._odom_pub.publish(odom)

        # Publish TF
        if self._publish_tf:
            import tf2_ros
            from geometry_msgs.msg import TransformStamped
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self._odom_frame
            t.child_frame_id = self._base_frame
            t.transform.translation.x = self._x
            t.transform.translation.y = self._y
            t.transform.rotation.z = math.sin(self._theta / 2.0)
            t.transform.rotation.w = math.cos(self._theta / 2.0)
            self._tf_broadcaster.sendTransform(t)

    def _publish_health(self):
        msg = eimsg.HealthDiag()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.component_name = 'motor_driver'
        if self._use_hardware and self._serial and not self._serial.is_open:
            msg.level = eimsg.HealthDiag.LEVEL_ERROR
            msg.message = 'Serial disconnected'
        else:
            msg.level = eimsg.HealthDiag.LEVEL_OK
            msg.message = 'OK'
        self._health_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotorAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop motors on shutdown
        node._send_to_hardware(0.0, 0.0) if node._use_hardware and node._serial else None
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
