#!/usr/bin/env python3
"""
STM Bridge Node — cmd_vel → RPM → H723 ASCII + telemetri → ROS topics.

Subscribes:
  /cmd_vel_safe (geometry_msgs/Twist) — safe velocity command from safety_mux

Publishes:
  /stm/imu/data (sensor_msgs/Imu) — MPU9250 IMU from H723
  /stm/magnetic_field (sensor_msgs/MagneticField) — QMC5883P mag from H723
  /stm/wheel_odom (nav_msgs/Odometry) — wheel odometry from F411 RPM
  /stm/status (earendil_interfaces/StmStatus) — H723 link + mode
  /stm/fault_flags (earendil_interfaces/StmFaultFlags) — F411 faults

Sends to H723:
  FL rpm <signed>\r\n, FR rpm <signed>\r\n, RL rpm <signed>\r\n, RR rpm <signed>\r\n

Safety: Only this node writes to H723 serial. Only safety_mux writes to /cmd_vel_safe.
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, MagneticField
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time

from earendil_interfaces.msg import StmStatus, StmFaultFlags, RscpCommand

from .telemetry_parser import (
    parse_line,
    TelemetryFrame,
    MOTOR_NAMES,
    MODE_DISARMED,
    MODE_MANUAL,
    MODE_AUTONOMOUS,
    MG_TO_MS2,
)

import logging

logger = logging.getLogger(__name__)

SENSOR_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class StmBridgeNode(Node):
    """STM bridge — cmd_vel → H723 ASCII commands + telemetry parsing."""

    def __init__(self):
        super().__init__('stm_bridge')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_safe')
        self.declare_parameter('serial_port', '/dev/earendil_h7')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('ack_timeout_ms', 500)
        self.declare_parameter('max_retries', 3)
        self.declare_parameter('link_loss_timeout_ms', 3000)
        self.declare_parameter('wheel_base', 0.825)
        self.declare_parameter('track_width', 1.10)
        self.declare_parameter('wheel_radius', 0.125)
        self.declare_parameter('wheel_circumference', 0.785)
        self.declare_parameter('hall_pulses_per_motor_rev', 90)
        self.declare_parameter('gear_ratio', 1.0)
        self.declare_parameter('max_motor_rpm', 300)
        self.declare_parameter('use_hardware', False)

        self._cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self._serial_port = self.get_parameter('serial_port').value
        self._serial_baud = self.get_parameter('serial_baud').value
        self._ack_timeout = self.get_parameter('ack_timeout_ms').value / 1000.0
        self._max_retries = self.get_parameter('max_retries').value
        self._link_loss_timeout = self.get_parameter('link_loss_timeout_ms').value / 1000.0
        self._wheel_base = self.get_parameter('wheel_base').value
        self._track_width = self.get_parameter('track_width').value
        self._wheel_radius = self.get_parameter('wheel_radius').value
        self._wheel_circumference = self.get_parameter('wheel_circumference').value
        self._hall_pulses = self.get_parameter('hall_pulses_per_motor_rev').value
        self._gear_ratio = self.get_parameter('gear_ratio').value
        self._max_rpm = self.get_parameter('max_motor_rpm').value
        self._use_hardware = self.get_parameter('use_hardware').value

        # ── State ───────────────────────────────────────────────────────────
        self._serial = None
        self._serial_write_lock = threading.Lock()
        self._serial_read_lock = threading.Lock()
        self._running = True
        self._operating_mode = MODE_DISARMED
        self._last_telemetry_time = 0.0
        self._link_active = False
        self._commands_sent = 0
        self._acks_received = 0
        self._timeouts = 0

        # Wheel odom state
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_theta = 0.0
        self._last_odom_time = None

        # Latest motor RPMs for odom
        self._latest_rpms = [0.0, 0.0, 0.0, 0.0]  # FL, FR, RL, RR

        # ── Publishers ──────────────────────────────────────────────────────
        self._imu_pub = self.create_publisher(Imu, '/stm/imu/data', SENSOR_QOS)
        self._mag_pub = self.create_publisher(MagneticField, '/stm/magnetic_field', SENSOR_QOS)
        self._odom_pub = self.create_publisher(Odometry, '/stm/wheel_odom', SENSOR_QOS)
        self._status_pub = self.create_publisher(StmStatus, '/stm/status', SENSOR_QOS)
        self._fault_pub = self.create_publisher(StmFaultFlags, '/stm/fault_flags', SENSOR_QOS)

        # ── Subscribers ─────────────────────────────────────────────────────
        self.create_subscription(Twist, self._cmd_vel_topic, self._on_cmd_vel, 10)
        self.create_subscription(RscpCommand, '/rscp/command', self._on_rscp_command, 10)

        # ── Timers ──────────────────────────────────────────────────────────
        self._status_timer = self.create_timer(1.0, self._publish_status)

        # ── Serial thread ───────────────────────────────────────────────────
        if self._use_hardware:
            self._serial_thread = threading.Thread(target=self._serial_loop, daemon=True)
            self._serial_thread.start()
        else:
            self.get_logger().info('stm_bridge started in SIMULATION mode (no serial)')

        self.get_logger().info(
            f'stm_bridge started — port={self._serial_port}, '
            f'wheel_base={self._wheel_base}, track_width={self._track_width}, '
            f'wheel_radius={self._wheel_radius}, max_rpm={self._max_rpm}, '
            f'use_hardware={self._use_hardware}'
        )

    # ── cmd_vel → RPM conversion ────────────────────────────────────────────

    def _on_cmd_vel(self, msg: Twist):
        """Convert cmd_vel to motor RPMs and send to H723."""
        vx = msg.linear.x    # m/s forward
        wz = msg.angular.z    # rad/s turn

        # Skid-steer kinematics (track_width = distance between left/right wheel centers)
        # left_rpm  = (vx - wz * track_width/2) / circumference * 60
        # right_rpm = (vx + wz * track_width/2) / circumference * 60
        half_track = self._track_width / 2.0
        left_v = vx - wz * half_track
        right_v = vx + wz * half_track

        left_rpm = (left_v / self._wheel_circumference) * 60.0
        right_rpm = (right_v / self._wheel_circumference) * 60.0

        # Clamp
        left_rpm = max(-self._max_rpm, min(self._max_rpm, left_rpm))
        right_rpm = max(-self._max_rpm, min(self._max_rpm, right_rpm))

        # Motor mapping: FL=left, FR=right, RL=left, RR=right
        rpms = [left_rpm, right_rpm, left_rpm, right_rpm]

        self._send_motor_commands(rpms)

    def _on_rscp_command(self, msg: RscpCommand):
        """Handle RSCP arm/disarm → send H723 mode command."""
        CMD_ARM = 1
        CMD_DISARM = 2
        if msg.command_type == CMD_ARM:
            self._send_h723_mode('auto')
        elif msg.command_type == CMD_DISARM:
            self._send_h723_mode('disarm')
            # Also send zero RPM
            self._send_serial('stop\n')

    def _send_h723_mode(self, mode: str):
        """Send mode command to H723: disarm, manual, or auto."""
        if mode in ('disarm', 'manual', 'auto'):
            self._send_serial(f'mode {mode}\n')
            self.get_logger().info(f'H723 mode command: {mode}')

    def _send_motor_commands(self, rpms):
        """Send RPM commands to H723 for all 4 motors."""
        commands = []
        for i, (name, rpm) in enumerate(zip(MOTOR_NAMES, rpms)):
            cmd = f'{name} rpm {int(round(rpm))}\r\n'
            commands.append(cmd)

        self._send_serial(''.join(commands))
        self._commands_sent += 1

    # ── Serial communication ────────────────────────────────────────────────

    def _serial_loop(self):
        """Main serial read loop with reconnection."""
        while self._running:
            try:
                self._open_serial()
                self._read_loop()
            except Exception as e:
                if self._running:
                    self.get_logger().warn(f'STM serial error: {e}, reconnecting...')
                    self._close_serial()
                    time.sleep(1.0)

    def _open_serial(self):
        """Open serial port to H723."""
        import serial
        with self._serial_write_lock:
            self._serial = serial.Serial(
                port=self._serial_port,
                baudrate=self._serial_baud,
                timeout=0.1,
            )
        self.get_logger().info(f'STM serial opened: {self._serial_port}')

    def _close_serial(self):
        """Close serial port safely."""
        with self._serial_write_lock:
            with self._serial_read_lock:
                if self._serial and self._serial.is_open:
                    try:
                        self._serial.close()
                    except Exception:
                        pass
                self._serial = None

    def _send_serial(self, data: str):
        """Send ASCII data to H723 (non-blocking for reads)."""
        with self._serial_write_lock:
            if self._serial and self._serial.is_open:
                try:
                    self._serial.write(data.encode('ascii'))
                except Exception as e:
                    self.get_logger().warn(f'STM write error: {e}')

    def _read_loop(self):
        """Read lines from H723 and process telemetry."""
        buffer = ''
        while self._running:
            with self._serial_read_lock:
                if self._serial is None or not self._serial.is_open:
                    break
                try:
                    data = self._serial.read(256)
                except Exception as e:
                    self.get_logger().warn(f'STM read error: {e}')
                    break

            if data:
                buffer += data.decode('ascii', errors='replace')
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        self._process_telemetry_line(line)
            else:
                # Check link loss
                if (self._last_telemetry_time > 0 and
                        time.time() - self._last_telemetry_time > self._link_loss_timeout):
                    self._link_active = False
                    self.get_logger().warn('STM link loss detected')

    def _process_telemetry_line(self, line: str):
        """Process a single telemetry line from H723."""
        frame = parse_line(line)
        self._last_telemetry_time = time.time()
        self._link_active = True

        if frame.ack:
            self._acks_received += 1
            return

        if frame.nack:
            self.get_logger().warn('STM NACK received')
            return

        if frame.mode is not None:
            self._operating_mode = frame.mode
            return

        if frame.imu:
            self._publish_imu(frame.imu)

        if frame.mag:
            self._publish_mag(frame.mag)

        if frame.motors:
            for motor_id, mt in frame.motors.items():
                self._latest_rpms[motor_id] = mt.rpm
            self._update_wheel_odom(frame.motors)
            self._check_faults(frame.motors)

    # ── ROS publishing ──────────────────────────────────────────────────────

    def _publish_imu(self, imu_data):
        """Publish IMU data as sensor_msgs/Imu."""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Linear acceleration (m/s²)
        msg.linear_acceleration.x = imu_data.accel_x
        msg.linear_acceleration.y = imu_data.accel_y
        msg.linear_acceleration.z = imu_data.accel_z

        # Angular velocity (rad/s)
        msg.angular_velocity.x = imu_data.gyro_x
        msg.angular_velocity.y = imu_data.gyro_y
        msg.angular_velocity.z = imu_data.gyro_z

        # Orientation unknown from raw IMU — mark as invalid
        msg.orientation.w = 1.0
        msg.orientation_covariance[0] = -1.0  # orientation unknown

        self._imu_pub.publish(msg)

    def _publish_mag(self, mag_data):
        """Publish magnetometer data."""
        msg = MagneticField()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Convert µT to Tesla
        msg.magnetic_field.x = mag_data.mag_x * 1e-6
        msg.magnetic_field.y = mag_data.mag_y * 1e-6
        msg.magnetic_field.z = mag_data.mag_z * 1e-6

        self._mag_pub.publish(msg)

    def _update_wheel_odom(self, motors):
        """Compute wheel odometry from motor RPMs."""
        now = self.get_clock().now()
        if self._last_odom_time is None:
            self._last_odom_time = now
            return

        dt = (now - self._last_odom_time).nanoseconds / 1e9
        if dt <= 0 or dt > 1.0:
            self._last_odom_time = now
            return

        self._last_odom_time = now

        # Average left/right RPM
        left_rpm = (self._latest_rpms[0] + self._latest_rpms[2]) / 2.0  # FL + RL
        right_rpm = (self._latest_rpms[1] + self._latest_rpms[3]) / 2.0  # FR + RR

        # Convert to m/s
        left_v = left_rpm * self._wheel_circumference / 60.0
        right_v = right_rpm * self._wheel_circumference / 60.0

        # Linear and angular velocity (skid-steer)
        v = (left_v + right_v) / 2.0
        omega = (right_v - left_v) / self._track_width

        # Integrate pose
        self._odom_theta += omega * dt
        self._odom_x += v * math.cos(self._odom_theta) * dt
        self._odom_y += v * math.sin(self._odom_theta) * dt

        # Publish odometry
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position.x = self._odom_x
        msg.pose.pose.position.y = self._odom_y
        msg.pose.pose.position.z = 0.0

        # Quaternion from yaw
        msg.pose.pose.orientation.z = math.sin(self._odom_theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(self._odom_theta / 2.0)

        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = omega

        self._odom_pub.publish(msg)

    def _check_faults(self, motors):
        """Check for motor faults and publish fault flags."""
        faults = StmFaultFlags()
        faults.header.stamp = self.get_clock().now().to_msg()

        any_fault = False
        fault_count = 0

        for motor_id, mt in motors.items():
            if mt.fault_code != 0:
                any_fault = True
                fault_count += 1
                # Map fault codes to flags (exact mapping depends on F411 firmware)
                # For now, just report the fault code
                if motor_id < 4:
                    faults.driver_error[motor_id] = True

        faults.any_fault = any_fault
        faults.fault_count = fault_count

        if any_fault:
            self._fault_pub.publish(faults)

    def _publish_status(self):
        """Publish STM status."""
        status = StmStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.operating_mode = self._operating_mode
        status.link_active = self._link_active
        status.link_age_sec = (
            time.time() - self._last_telemetry_time
            if self._last_telemetry_time > 0 else -1.0
        )
        status.commands_sent = self._commands_sent
        status.acks_received = self._acks_received
        status.timeouts = self._timeouts

        self._status_pub.publish(status)

    # ── Cleanup ─────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._running = False
        # Send zero RPM before shutdown
        if self._use_hardware:
            self._send_serial('FL rpm 0\r\nFR rpm 0\r\nRL rpm 0\r\nRR rpm 0\r\n')
        self._close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StmBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
