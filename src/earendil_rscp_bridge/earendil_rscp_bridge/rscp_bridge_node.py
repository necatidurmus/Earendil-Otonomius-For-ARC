#!/usr/bin/env python3
"""
RSCP Bridge Node — COBS + protobuf serial bridge to ARC Competition Module.

Architecture (CLAUDE.md §2):
  CM → serial → COBS decode → RequestEnvelope → WhichOneof('request') → dispatch
  rover state → ResponseEnvelope → SerializeToString → COBS encode + 0x00 → serial

This node does NOT publish to /cmd_vel_* topics. Motor commands go through:
  RSCP → mission_manager → Nav2 → /cmd_vel_nav → safety_mux → /cmd_vel_safe → stm_bridge

Subscribes:
  /mission/result (std_msgs/String) — mission completion signals
  /mission/status (earendil_interfaces/MissionStatus) — mission progress
  /gps/fix (sensor_msgs/NavSatFix) — RTK GPS for coordinate responses
  /rtk/status (std_msgs/String) — RTK fix quality
  /stm/imu/data (sensor_msgs/Imu) — heading for RoverStatus
  /stm/wheel_odom (nav_msgs/Odometry) — distance for exploration

Publishes:
  /rscp/current_stage (std_msgs/UInt32) — current ARC stage
  /rscp/command (earendil_interfaces/RscpCommand) — parsed RSCP commands
  /rscp/status (earendil_interfaces/RscpStatus) — bridge health
"""

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import UInt32, String
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry

from earendil_interfaces.msg import RscpCommand, RscpStatus, MissionStage

from .rscp_parser import (
    FrameReader,
    decode_frame,
    encode_frame,
    parse_request,
    create_acknowledge,
    create_task_finished,
    create_gps_coordinate,
    create_distance,
    create_message,
    create_rover_status,
    CMD_SET_STAGE,
    CMD_ARM,
    CMD_DISARM,
    CMD_NAVIGATE_TO_GPS,
    CMD_SEARCH_AREA,
    CMD_START_EXPLORATION,
)

import logging

logger = logging.getLogger(__name__)

# QoS for sensor topics (best_effort, volatile)
SENSOR_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class RscpBridgeNode(Node):
    """RSCP bridge — serial COBS+protobuf communication with ARC Competition Module."""

    def __init__(self):
        super().__init__('rscp_bridge')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('serial_port', '/dev/earendil_rscp')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('serial_timeout_s', 1.0)
        self.declare_parameter('rover_status_period_s', 1.0)
        self.declare_parameter('frame_buffer_timeout_s', 2.0)
        self.declare_parameter('reconnect_delay_s', 1.0)
        self.declare_parameter('log_level', 'INFO')

        self._serial_port = self.get_parameter('serial_port').value
        self._serial_baud = self.get_parameter('serial_baud').value
        self._serial_timeout = self.get_parameter('serial_timeout_s').value
        self._rover_status_period = self.get_parameter('rover_status_period_s').value
        self._frame_buffer_timeout = self.get_parameter('frame_buffer_timeout_s').value
        self._reconnect_delay = self.get_parameter('reconnect_delay_s').value

        # ── State ───────────────────────────────────────────────────────────
        self._current_stage = 0
        self._rover_state = 0  # DISARMED
        self._armed = False
        self._frame_reader = FrameReader()
        self._serial = None
        self._serial_write_lock = threading.Lock()
        self._serial_read_lock = threading.Lock()
        self._running = True
        self._MAX_FRAME_SIZE = 1024  # prevent unbounded buffer growth

        # ── Latest sensor data (for RoverStatus) ────────────────────────────
        self._latest_gps = None      # NavSatFix
        self._latest_rtk = None      # str
        self._latest_imu = None      # Imu
        self._latest_odom = None     # Odometry

        # ── Counters ────────────────────────────────────────────────────────
        self._frames_received = 0
        self._frames_sent = 0
        self._parse_errors = 0
        self._cobs_errors = 0
        self._last_command_time = 0.0

        # ── Publishers ──────────────────────────────────────────────────────
        self._stage_pub = self.create_publisher(UInt32, '/rscp/current_stage', 10)
        self._command_pub = self.create_publisher(RscpCommand, '/rscp/command', 10)
        self._status_pub = self.create_publisher(RscpStatus, '/rscp/status', 10)

        # ── Subscribers ─────────────────────────────────────────────────────
        self.create_subscription(String, '/mission/result', self._on_mission_result, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self._on_gps_fix, SENSOR_QOS)
        self.create_subscription(String, '/rtk/status', self._on_rtk_status, SENSOR_QOS)
        self.create_subscription(Imu, '/stm/imu/data', self._on_imu, SENSOR_QOS)
        self.create_subscription(Odometry, '/stm/wheel_odom', self._on_wheel_odom, SENSOR_QOS)

        # ── Timers ──────────────────────────────────────────────────────────
        self._rover_status_timer = self.create_timer(
            self._rover_status_period, self._send_rover_status
        )
        self._status_timer = self.create_timer(1.0, self._publish_status)

        # ── Serial thread ───────────────────────────────────────────────────
        self._serial_thread = threading.Thread(target=self._serial_loop, daemon=True)
        self._serial_thread.start()

        self.get_logger().info(
            f'RSCP bridge started — port={self._serial_port}, baud={self._serial_baud}'
        )

    # ── Serial communication ────────────────────────────────────────────────

    def _serial_loop(self):
        """Main serial read loop with exponential backoff reconnection."""
        backoff = self._reconnect_delay
        max_backoff = 30.0
        while self._running:
            try:
                self._open_serial()
                self._read_loop()
                backoff = self._reconnect_delay  # reset on clean exit
            except Exception as e:
                if self._running:
                    self.get_logger().warn(f'Serial error: {e}, reconnecting in {backoff:.1f}s')
                    self._close_serial()
                    time.sleep(backoff)
                    backoff = min(backoff * 2, max_backoff)

    def _open_serial(self):
        """Open serial port."""
        import serial
        with self._serial_write_lock:
            self._serial = serial.Serial(
                port=self._serial_port,
                baudrate=self._serial_baud,
                timeout=self._serial_timeout,
            )
        self.get_logger().info(f'Serial port opened: {self._serial_port}')

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

    def _read_loop(self):
        """Read bytes from serial, feed to FrameReader, process complete frames."""
        while self._running:
            with self._serial_read_lock:
                if self._serial is None or not self._serial.is_open:
                    break
                try:
                    data = self._serial.read(1)
                except Exception as e:
                    self.get_logger().warn(f'Serial read error: {e}')
                    break

            if not data:
                continue

            byte = data[0]
            frame = self._frame_reader.feed(byte)

            # Frame buffer overflow protection
            if self._frame_reader.buffer_size > self._MAX_FRAME_SIZE:
                self.get_logger().warn(
                    f'Frame buffer overflow ({self._frame_reader.buffer_size} bytes) — resetting')
                self._frame_reader.reset()
                continue

            if frame is not None:
                self._frames_received += 1
                self._process_frame(frame)

    def _send_bytes(self, data: bytes):
        """Send COBS-encoded bytes over serial (non-blocking for reads)."""
        with self._serial_write_lock:
            if self._serial and self._serial.is_open:
                try:
                    self._serial.write(data)
                    self._frames_sent += 1
                except Exception as e:
                    self.get_logger().warn(f'Serial write error: {e}')

    # ── Frame processing ────────────────────────────────────────────────────

    def _process_frame(self, raw_frame: bytes):
        """Process a complete COBS frame: decode, parse, dispatch."""
        # COBS decode
        try:
            decoded = decode_frame(raw_frame)
        except Exception as e:
            self._cobs_errors += 1
            self.get_logger().warn(f'COBS decode error: {e}')
            return

        # Protobuf parse
        envelope, field_name, cmd = parse_request(decoded)

        if envelope is None:
            self._parse_errors += 1
            return

        if field_name is None or cmd is None:
            self._parse_errors += 1
            self.get_logger().warn('Unknown or empty RSCP request')
            self._send_acknowledge()
            self._send_message('unknown request type')
            return

        self._last_command_time = time.time()

        # Validate and dispatch — ACK only after successful dispatch
        if self._validate_command(field_name, cmd):
            self._dispatch_command(field_name, cmd)
            self._send_acknowledge()
        else:
            self.get_logger().warn(f'Command validation failed: {field_name}')
            self._send_message(f'invalid command: {field_name}')

    def _validate_command(self, field_name: str, cmd) -> bool:
        """Validate RSCP command before dispatch."""
        if field_name == 'set_stage':
            return hasattr(cmd, 'stage_value') and 1 <= cmd.stage_value <= 4
        elif field_name == 'arm_disarm':
            return hasattr(cmd, 'arm_value') and cmd.arm_value is not None
        elif field_name == 'navigate_to_gps':
            return (hasattr(cmd, 'latitude') and hasattr(cmd, 'longitude') and
                    -90 <= cmd.latitude <= 90 and -180 <= cmd.longitude <= 180)
        elif field_name == 'search_area':
            return (hasattr(cmd, 'latitude') and hasattr(cmd, 'longitude') and
                    -90 <= cmd.latitude <= 90 and -180 <= cmd.longitude <= 180 and
                    cmd.search_radius > 0)
        elif field_name == 'start_exploration':
            return True
        return False

    def _dispatch_command(self, field_name: str, cmd):
        """Dispatch parsed command to appropriate internal topics."""
        if field_name == 'set_stage':
            self._handle_set_stage(cmd)
        elif field_name == 'arm_disarm':
            self._handle_arm_disarm(cmd)
        elif field_name == 'navigate_to_gps':
            self._handle_navigate_to_gps(cmd)
        elif field_name == 'search_area':
            self._handle_search_area(cmd)
        elif field_name == 'start_exploration':
            self._handle_start_exploration(cmd)
        else:
            self.get_logger().warn(f'Unhandled command type: {field_name}')

    def _handle_set_stage(self, cmd):
        """Handle SetStage command."""
        self._current_stage = cmd.stage_value
        self.get_logger().info(f'SetStage: stage={cmd.stage_value}')

        # Publish stage
        stage_msg = UInt32()
        stage_msg.data = cmd.stage_value
        self._stage_pub.publish(stage_msg)

        # Publish command
        rscp_cmd = RscpCommand()
        rscp_cmd.header.stamp = self.get_clock().now().to_msg()
        rscp_cmd.command_type = RscpCommand.CMD_SET_STAGE
        rscp_cmd.stage_value = cmd.stage_value
        self._command_pub.publish(rscp_cmd)

    def _handle_arm_disarm(self, cmd):
        """Handle ArmDisarm command."""
        if cmd.arm_value is None:
            self.get_logger().warn('ArmDisarm: value_wrapper not set')
            return

        self._armed = cmd.arm_value
        self._rover_state = 1 if cmd.arm_value else 0  # AUTONOMOUS or DISARMED
        action = 'ARM' if cmd.arm_value else 'DISARM'
        self.get_logger().info(f'ArmDisarm: {action}')

        rscp_cmd = RscpCommand()
        rscp_cmd.header.stamp = self.get_clock().now().to_msg()
        rscp_cmd.command_type = RscpCommand.CMD_ARM if cmd.arm_value else RscpCommand.CMD_DISARM
        self._command_pub.publish(rscp_cmd)

    def _handle_navigate_to_gps(self, cmd):
        """Handle NavigateToGPS command."""
        self.get_logger().info(
            f'NavigateToGPS: lat={cmd.latitude:.6f}, lon={cmd.longitude:.6f}, alt={cmd.altitude:.1f}'
        )

        rscp_cmd = RscpCommand()
        rscp_cmd.header.stamp = self.get_clock().now().to_msg()
        rscp_cmd.command_type = RscpCommand.CMD_NAVIGATE_TO_GPS
        rscp_cmd.latitude = cmd.latitude
        rscp_cmd.longitude = cmd.longitude
        rscp_cmd.altitude = cmd.altitude
        self._command_pub.publish(rscp_cmd)

    def _handle_search_area(self, cmd):
        """Handle SearchArea command."""
        self.get_logger().info(
            f'SearchArea: lat={cmd.latitude:.6f}, lon={cmd.longitude:.6f}, r={cmd.search_radius:.1f}m'
        )

        rscp_cmd = RscpCommand()
        rscp_cmd.header.stamp = self.get_clock().now().to_msg()
        rscp_cmd.command_type = RscpCommand.CMD_SEARCH_AREA
        rscp_cmd.latitude = cmd.latitude
        rscp_cmd.longitude = cmd.longitude
        rscp_cmd.search_radius = cmd.search_radius
        self._command_pub.publish(rscp_cmd)

    def _handle_start_exploration(self, cmd):
        """Handle StartExploration command."""
        self.get_logger().info('StartExploration')

        rscp_cmd = RscpCommand()
        rscp_cmd.header.stamp = self.get_clock().now().to_msg()
        rscp_cmd.command_type = RscpCommand.CMD_START_EXPLORATION
        self._command_pub.publish(rscp_cmd)

    # ── Response sending ────────────────────────────────────────────────────

    def _send_acknowledge(self):
        """Send Acknowledge response to CM."""
        self._send_bytes(encode_frame(create_acknowledge()))

    def _send_task_finished(self):
        """Send TaskFinished response to CM."""
        self._send_bytes(encode_frame(create_task_finished()))
        self.get_logger().info('Sent TaskFinished')

    def _send_gps_coordinate(self, lat: float, lon: float, alt: float):
        """Send GPSCoordinate response to CM."""
        self._send_bytes(encode_frame(create_gps_coordinate(lat, lon, alt)))
        self.get_logger().info(f'Sent GPSCoordinate: lat={lat:.6f}, lon={lon:.6f}, alt={alt:.1f}')

    def _send_distance(self, distance_m: float):
        """Send distance response to CM."""
        self._send_bytes(encode_frame(create_distance(distance_m)))
        self.get_logger().info(f'Sent distance: {distance_m:.2f}m')

    def _send_message(self, text: str):
        """Send message response to CM."""
        self._send_bytes(encode_frame(create_message(text)))
        self.get_logger().info(f'Sent message: {text}')

    def _send_rover_status(self):
        """Send RoverStatus response (≤1 Hz)."""
        lat, lon, alt = 0.0, 0.0, 0.0
        heading = 0.0

        if self._latest_gps is not None:
            lat = self._latest_gps.latitude
            lon = self._latest_gps.longitude
            alt = self._latest_gps.altitude

        if self._latest_imu is not None:
            # Extract yaw from quaternion
            import math
            q = self._latest_imu.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            heading = math.degrees(math.atan2(siny_cosp, cosy_cosp))
            if heading < 0:
                heading += 360.0

        # Battery: H723 gap — no battery telemetry available
        status_bytes = create_rover_status(
            state=self._rover_state,
            latitude=lat,
            longitude=lon,
            altitude=alt,
            heading=heading,
            battery_voltage=0.0,
            battery_current=0.0,
            battery_soc=0.0,
        )
        self._send_bytes(encode_frame(status_bytes))

    # ── Subscriber callbacks ────────────────────────────────────────────────

    def _on_mission_result(self, msg: String):
        """Handle mission result from mission manager."""
        result = msg.data.lower().strip()
        if result == 'task_finished':
            self._send_task_finished()
        elif result.startswith('gps_coordinate'):
            # Format: "gps_coordinate:lat,lon,alt"
            try:
                parts = result.split(':')[1].split(',')
                lat, lon, alt = float(parts[0]), float(parts[1]), float(parts[2])
                self._send_gps_coordinate(lat, lon, alt)
            except (IndexError, ValueError) as e:
                self.get_logger().warn(f'Invalid gps_coordinate format: {msg.data} ({e})')
        elif result.startswith('distance'):
            # Format: "distance:42.5"
            try:
                dist = float(result.split(':')[1])
                self._send_distance(dist)
            except (IndexError, ValueError) as e:
                self.get_logger().warn(f'Invalid distance format: {msg.data} ({e})')
        else:
            self.get_logger().warn(f'Unknown mission result: {msg.data}')

    def _on_gps_fix(self, msg: NavSatFix):
        self._latest_gps = msg

    def _on_rtk_status(self, msg: String):
        self._latest_rtk = msg.data

    def _on_imu(self, msg: Imu):
        self._latest_imu = msg

    def _on_wheel_odom(self, msg: Odometry):
        self._latest_odom = msg

    # ── Status publishing ───────────────────────────────────────────────────

    def _publish_status(self):
        """Publish bridge health status."""
        status = RscpStatus()
        status.header.stamp = self.get_clock().now().to_msg()

        if self._serial and self._serial.is_open:
            status.connection_state = RscpStatus.STATE_CONNECTED
        else:
            status.connection_state = RscpStatus.STATE_DISCONNECTED

        status.serial_port = self._serial_port
        status.frames_received = self._frames_received
        status.frames_sent = self._frames_sent
        status.parse_errors = self._parse_errors
        status.cobs_errors = self._cobs_errors
        status.last_command_age_sec = (
            time.time() - self._last_command_time if self._last_command_time > 0 else -1.0
        )

        self._status_pub.publish(status)

    # ── Cleanup ─────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._running = False
        self._close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RscpBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
