#!/usr/bin/env python3
"""
LiDAR Adapter Node — DFRobot D800 (LDRobot LD06) hardware interface.

Reads from LD06 serial protocol and publishes:
  /scan  (sensor_msgs/LaserScan) — 360° 2D laser scan
  /health/heartbeat

LD06 Protocol:
  Baud: 230400, 8N1
  Packet (47 bytes):
    [0x54] [VerLen=0x2C] [Speed_L] [Speed_H]
    [StartAngle_L] [StartAngle_H]
    12x [Dist_L] [Dist_H] [Confidence]
    [EndAngle_L] [EndAngle_H]
    [Timestamp_L] [Timestamp_H]
    [CRC8]
  Angle unit: 0.01 degree
  Distance unit: mm
  Scan rate: 5-12 Hz

Reference: DFRobot D800 wiki, LDRobot LD06 datasheet.
"""

import math
import struct
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan

import earendil_interfaces.msg as eimsg

# ── LD06 Protocol Constants ──────────────────────────────────────────────────

HEADER_BYTE = 0x54
VERLEN_BYTE = 0x2C  # 44 = 12 points x 3 + 12 trailer bytes
POINTS_PER_PACKET = 12
PACKET_SIZE = 47  # 1(header) + 1(verlen) + 2(speed) + 2(start_angle)
                  # + 12x3(points) + 2(end_angle) + 2(timestamp) + 1(crc)

# CRC8 lookup table for LD06 (polynomial 0x4D)
CRC_TABLE = [
    0x00, 0x4D, 0x9A, 0xD7, 0x79, 0x34, 0xE3, 0xAE,
    0xF2, 0xBF, 0x68, 0x25, 0x8B, 0xC6, 0x11, 0x5C,
    0xA9, 0xE4, 0x33, 0x7E, 0xD0, 0x9D, 0x4A, 0x07,
    0x5B, 0x16, 0xC1, 0x8C, 0x22, 0x6F, 0xB8, 0xF5,
    0x1F, 0x52, 0x85, 0xC8, 0x66, 0x2B, 0xFC, 0xB1,
    0xED, 0xA0, 0x77, 0x3A, 0x94, 0xD9, 0x0E, 0x43,
    0xB6, 0xFB, 0x2C, 0x61, 0xCF, 0x82, 0x55, 0x18,
    0x44, 0x09, 0xDE, 0x93, 0x3D, 0x70, 0xA7, 0xEA,
    0x3E, 0x73, 0xA4, 0xE9, 0x47, 0x0A, 0xDD, 0x90,
    0xCC, 0x81, 0x56, 0x1B, 0xB5, 0xF8, 0x2F, 0x62,
    0x97, 0xDA, 0x0D, 0x40, 0xEE, 0xA3, 0x74, 0x39,
    0x65, 0x28, 0xFF, 0xB2, 0x1C, 0x51, 0x86, 0xCB,
    0x21, 0x6C, 0xBB, 0xF6, 0x58, 0x15, 0xC2, 0x8F,
    0xD3, 0x9E, 0x49, 0x04, 0xAA, 0xE7, 0x30, 0x7D,
    0x88, 0xC5, 0x12, 0x5F, 0xF1, 0xBC, 0x6B, 0x26,
    0x7A, 0x37, 0xE0, 0xAD, 0x03, 0x4E, 0x99, 0xD4,
    0x7C, 0x31, 0xE6, 0xAB, 0x05, 0x48, 0x9F, 0xD2,
    0x8E, 0xC3, 0x14, 0x59, 0xF7, 0xBA, 0x6D, 0x20,
    0xD5, 0x98, 0x4F, 0x02, 0xAC, 0xE1, 0x36, 0x7B,
    0x27, 0x6A, 0xBD, 0xF0, 0x5E, 0x13, 0xC4, 0x89,
    0x63, 0x2E, 0xF9, 0xB4, 0x1A, 0x57, 0x80, 0xCD,
    0x91, 0xDC, 0x0B, 0x46, 0xE8, 0xA5, 0x72, 0x3F,
    0xCA, 0x87, 0x50, 0x1D, 0xB3, 0xFE, 0x29, 0x64,
    0x38, 0x75, 0xA2, 0xEF, 0x41, 0x0C, 0xDB, 0x96,
    0x42, 0x0F, 0xD8, 0x95, 0x3B, 0x76, 0xA1, 0xEC,
    0xB0, 0xFD, 0x2A, 0x67, 0xC9, 0x84, 0x53, 0x1E,
    0xEB, 0xA6, 0x71, 0x3C, 0x92, 0xDF, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xCE, 0x60, 0x2D, 0xFA, 0xB7,
    0x5D, 0x10, 0xC7, 0x8A, 0x24, 0x69, 0xBE, 0xF3,
    0xAF, 0xE2, 0x35, 0x78, 0xD6, 0x9B, 0x4C, 0x01,
    0xF4, 0xB9, 0x6E, 0x23, 0x8D, 0xC0, 0x17, 0x5A,
    0x06, 0x4B, 0x9C, 0xD1, 0x7F, 0x32, 0xE5, 0xA8,
]


def crc8_ld06(data: bytes) -> int:
    """Compute CRC8 for LD06 packet (polynomial 0x4D)."""
    crc = 0x00
    for byte in data:
        crc = CRC_TABLE[(crc ^ byte) & 0xFF]
    return crc


class LiDARAdapterNode(Node):
    """LiDAR adapter - D800/LD06 serial -> /scan LaserScan."""

    def __init__(self):
        super().__init__('lidar_adapter')

        # -- Parameters --
        self.declare_parameter('serial_port', '/dev/earendil_lidar')
        self.declare_parameter('serial_baud', 230400)
        self.declare_parameter('use_hardware', False)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('frame_id', 'lidar_link')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('min_range_m', 0.02)
        self.declare_parameter('max_range_m', 12.0)
        self.declare_parameter('min_confidence', 10)

        self._frame_id = self.get_parameter('frame_id').value
        self._use_hw = self.get_parameter('use_hardware').value
        self._min_range = self.get_parameter('min_range_m').value
        self._max_range = self.get_parameter('max_range_m').value
        self._min_confidence = self.get_parameter('min_confidence').value

        # -- Scan accumulator --
        # Collect angle->distance points until a full 360 deg rotation
        self._scan_points = {}  # angle_bin (int degrees) -> (distance_m, confidence)
        self._last_end_angle = None
        self._scan_lock = threading.Lock()

        # -- Publishers --
        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._scan_pub = self.create_publisher(LaserScan, '/scan', sensor_qos)
        self._health_pub = self.create_publisher(
            eimsg.HealthDiag, '/health/heartbeat', 10
        )

        # -- Serial reader thread --
        self._serial = None
        self._running = True
        self._thread = None
        if self._use_hw:
            self._start_serial_reader()
        else:
            self.get_logger().info('LiDAR adapter: SIMULATION mode')
            self._sim_timer = self.create_timer(0.1, self._publish_sim_scan)

        # -- Health timer --
        self._health_timer = self.create_timer(1.0, self._publish_health)

        self.get_logger().info(
            f'LiDAR adapter started - port={self.get_parameter("serial_port").value}, '
            f'baud={self.get_parameter("serial_baud").value}, '
            f'hardware={self._use_hw}, frame={self._frame_id}'
        )

    # -- Serial Reader --

    def _start_serial_reader(self):
        """Open serial port and start reader thread."""
        try:
            import serial
            port = self.get_parameter('serial_port').value
            baud = self.get_parameter('serial_baud').value
            self._serial = serial.Serial(
                port, baud, timeout=1.0,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()
            self.get_logger().info(f'LiDAR serial: {port}@{baud}')
        except Exception as e:
            self.get_logger().error(f'LiDAR serial open failed: {e}')

    def _read_loop(self):
        """Read LD06 packets from serial and build LaserScan."""
        buf = bytearray()

        while self._running:
            try:
                if self._serial is None or not self._serial.is_open:
                    self._reconnect_serial()
                    continue

                # Read available bytes
                chunk = self._serial.read(256)
                if not chunk:
                    continue
                buf.extend(chunk)

                # Process packets in buffer
                while len(buf) >= PACKET_SIZE:
                    # Find header byte
                    hdr_idx = buf.find(HEADER_BYTE)
                    if hdr_idx < 0:
                        buf.clear()
                        break
                    if hdr_idx > 0:
                        del buf[:hdr_idx]  # discard bytes before header

                    if len(buf) < PACKET_SIZE:
                        break

                    # Check verlen byte
                    if buf[1] != VERLEN_BYTE:
                        del buf[:1]  # skip this byte, look for next header
                        continue

                    # Extract packet
                    packet = bytes(buf[:PACKET_SIZE])
                    del buf[:PACKET_SIZE]

                    # Parse packet
                    self._parse_packet(packet)

            except Exception as e:
                if self._running:
                    self.get_logger().warn(f'LiDAR serial error: {e}, reconnecting...')
                    self._close_serial()

    def _parse_packet(self, packet: bytes):
        """Parse a single LD06 packet and accumulate scan points."""
        # CRC check over packet[0:46] vs packet[46]
        computed_crc = crc8_ld06(packet[:PACKET_SIZE - 1])
        if computed_crc != packet[PACKET_SIZE - 1]:
            return  # CRC mismatch - discard

        # Speed (not used for scan, but parsed for completeness)
        # speed = struct.unpack_from('<H', packet, 2)[0]

        # Start angle (0.01 degree units)
        start_angle_raw = struct.unpack_from('<H', packet, 4)[0]
        start_angle = start_angle_raw / 100.0  # degrees

        # End angle
        end_angle_raw = struct.unpack_from('<H', packet, 40)[0]
        end_angle = end_angle_raw / 100.0  # degrees

        # Timestamp (ms)
        # timestamp = struct.unpack_from('<H', packet, 42)[0]

        # Interpolate angles for 12 points
        if end_angle < start_angle:
            end_angle += 360.0
        angle_step = (end_angle - start_angle) / (POINTS_PER_PACKET - 1)

        with self._scan_lock:
            for i in range(POINTS_PER_PACKET):
                offset = 6 + i * 3
                dist_mm = struct.unpack_from('<H', packet, offset)[0]
                confidence = packet[offset + 2]

                angle = start_angle + i * angle_step
                if angle >= 360.0:
                    angle -= 360.0

                # Filter by confidence and range
                dist_m = dist_mm / 1000.0
                if confidence < self._min_confidence:
                    continue
                if dist_m < self._min_range or dist_m > self._max_range:
                    continue

                # Bin angle to nearest degree for scan accumulation
                angle_bin = int(angle) % 360
                self._scan_points[angle_bin] = (dist_m, confidence)

            # Detect full rotation: if end_angle wrapped past start
            if self._last_end_angle is not None:
                if self._last_end_angle > 300 and end_angle < 60:
                    self._publish_scan()
            self._last_end_angle = end_angle

    def _publish_scan(self):
        """Publish accumulated scan points as LaserScan message."""
        with self._scan_lock:
            if not self._scan_points:
                return

            now = self.get_clock().now().to_msg()
            scan = LaserScan()
            scan.header.stamp = now
            scan.header.frame_id = self._frame_id

            # Full 360 deg scan, 1 deg resolution
            num_readings = 360
            scan.angle_min = 0.0
            scan.angle_max = 2.0 * math.pi
            scan.angle_increment = (2.0 * math.pi) / num_readings
            scan.time_increment = 0.0
            scan.scan_time = 0.1
            scan.range_min = self._min_range
            scan.range_max = self._max_range

            ranges = []
            intensities = []
            for deg in range(num_readings):
                if deg in self._scan_points:
                    dist_m, conf = self._scan_points[deg]
                    ranges.append(dist_m)
                    intensities.append(float(conf))
                else:
                    ranges.append(float('inf'))
                    intensities.append(0.0)

            scan.ranges = ranges
            scan.intensities = intensities

            # Clear for next scan
            self._scan_points.clear()

        self._scan_pub.publish(scan)

    # -- Serial Reconnect --

    def _reconnect_serial(self):
        """Reconnect LiDAR serial with backoff."""
        time.sleep(2.0)
        try:
            import serial
            port = self.get_parameter('serial_port').value
            baud = self.get_parameter('serial_baud').value
            self._serial = serial.Serial(
                port, baud, timeout=1.0,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            self.get_logger().info(f'LiDAR serial reconnected: {port}')
        except Exception as e:
            self.get_logger().warn(f'LiDAR reconnect failed: {e}')

    def _close_serial(self):
        """Close LiDAR serial safely."""
        if self._serial and self._serial.is_open:
            try:
                self._serial.close()
            except Exception:
                pass
        self._serial = None

    # -- Simulation --

    def _publish_sim_scan(self):
        """Publish simulated LaserScan for testing (360 deg, 1 deg resolution)."""
        now = self.get_clock().now().to_msg()
        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = self._frame_id

        num_readings = 360
        scan.angle_min = 0.0
        scan.angle_max = 2.0 * math.pi
        scan.angle_increment = (2.0 * math.pi) / num_readings
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self._min_range
        scan.range_max = self._max_range

        # Simulated: 2m wall in front, 3m on sides
        ranges = []
        for i in range(num_readings):
            angle = i * scan.angle_increment
            cos_a = abs(math.cos(angle))
            sin_a = abs(math.sin(angle))
            if cos_a > 0.01:
                dist_x = 2.0 / cos_a
            else:
                dist_x = 100.0
            if sin_a > 0.01:
                dist_y = 3.0 / sin_a
            else:
                dist_y = 100.0
            dist = min(dist_x, dist_y, self._max_range)
            ranges.append(dist)

        scan.ranges = ranges
        scan.intensities = [100.0] * num_readings
        self._scan_pub.publish(scan)

    # -- Health --

    def _publish_health(self):
        msg = eimsg.HealthDiag()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.component_name = 'lidar_adapter'
        msg.level = eimsg.HealthDiag.LEVEL_OK
        msg.message = 'SIM' if not self._use_hw else 'OK'
        self._health_pub.publish(msg)

    # -- Cleanup --

    def destroy_node(self):
        self._running = False
        self._close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LiDARAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
