#!/usr/bin/env python3
"""
GPS Adapter Node — hardware-abstracted GPS interface.

Reads from serial GPS receiver (NMEA/UBX) and publishes:
  /gps/fix (sensor_msgs/NavSatFix) — RTK GPS position
  /rtk/status (std_msgs/String) — fix quality: FIXED/FLOAT/DGPS/SPS/NO_FIX

Change this file only when switching GPS hardware.
Default: reads NMEA sentences from serial port.
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String

import earendil_interfaces.msg as eimsg

# NMEA fix quality → RTK status string
FIX_QUALITY_MAP = {
    0: 'NO_FIX',    # Invalid
    1: 'SPS',       # Standard Positioning Service
    2: 'DGPS',      # Differential GPS
    4: 'RTK_FIXED', # RTK Fixed (some receivers)
    5: 'RTK_FLOAT', # RTK Float (some receivers)
}


class GPSAdapterNode(Node):
    """GPS hardware adapter. Publishes /gps/fix and /rtk/status from serial NMEA."""

    def __init__(self):
        super().__init__('gps_adapter')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('serial_port', '/dev/earendil_rtk')
        self.declare_parameter('serial_baud', 115200)  # LC25H EA RTK baud
        self.declare_parameter('use_hardware', False)
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('fix_topic', '/gps/fix')
        self.declare_parameter('rtk_status_topic', '/rtk/status')
        self.declare_parameter('frame_id', 'gps_link')

        self._frame_id = self.get_parameter('frame_id').value
        self._use_hw = self.get_parameter('use_hardware').value
        self._rate = self.get_parameter('publish_rate_hz').value

        # ── State ───────────────────────────────────────────────────────────
        self._lat = 0.0
        self._lon = 0.0
        self._alt = 0.0
        self._fix_quality = 0
        self._num_sats = 0
        self._hdop = 99.9

        # ── Publishers ──────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._fix_pub = self.create_publisher(
            NavSatFix,
            self.get_parameter('fix_topic').value,
            sensor_qos
        )
        self._rtk_pub = self.create_publisher(
            String,
            self.get_parameter('rtk_status_topic').value,
            sensor_qos
        )
        self._health_pub = self.create_publisher(
            eimsg.HealthDiag, '/health/heartbeat', 10
        )

        # ── Serial reader thread ────────────────────────────────────────────
        self._serial = None
        self._running = True
        if self._use_hw:
            self._start_serial_reader()
        else:
            # Simulation mode: publish dummy GPS (Ankara)
            self._sim_timer = self.create_timer(1.0 / self._rate, self._publish_sim)

        # ── Health timer ────────────────────────────────────────────────────
        self._health_timer = self.create_timer(1.0, self._publish_health)

        self.get_logger().info(
            f'GPS adapter started — port={self.get_parameter("serial_port").value}, '
            f'hardware={self._use_hw}, frame={self._frame_id}'
        )

    def _start_serial_reader(self):
        try:
            import serial
            port = self.get_parameter('serial_port').value
            baud = self.get_parameter('serial_baud').value
            self._serial = serial.Serial(port, baud, timeout=1.0)
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()
            self.get_logger().info(f'GPS serial: {port}@{baud}')
        except Exception as e:
            self.get_logger().error(f'GPS serial failed: {e}')

    def _read_loop(self):
        """Read NMEA sentences from serial and parse GGA with reconnection."""
        while self._running:
            try:
                if self._serial is None or not self._serial.is_open:
                    self._reconnect_serial()
                    continue
                line = self._serial.readline().decode('ascii', errors='ignore').strip()
                if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                    if self._validate_nmea_checksum(line):
                        self._parse_gga(line)
                    else:
                        self.get_logger().warn(f'NMEA checksum failed: {line[:20]}...')
            except Exception as e:
                if self._running:
                    self.get_logger().warn(f'GPS serial error: {e}, reconnecting...')
                    self._close_serial()

    def _validate_nmea_checksum(self, sentence: str) -> bool:
        """Validate NMEA sentence checksum (XOR between $ and *)."""
        if '*' not in sentence:
            return False
        body, checksum_str = sentence.split('*', 1)
        if body.startswith('$'):
            body = body[1:]
        calculated = 0
        for ch in body:
            calculated ^= ord(ch)
        try:
            expected = int(checksum_str[:2], 16)
            return calculated == expected
        except ValueError:
            return False

    def _reconnect_serial(self):
        """Reconnect GPS serial with backoff."""
        import time
        time.sleep(2.0)
        try:
            import serial
            port = self.get_parameter('serial_port').value
            baud = self.get_parameter('serial_baud').value
            self._serial = serial.Serial(port, baud, timeout=1.0)
            self.get_logger().info(f'GPS serial reconnected: {port}')
        except Exception as e:
            self.get_logger().warn(f'GPS reconnect failed: {e}')

    def _close_serial(self):
        """Close GPS serial safely."""
        if self._serial and self._serial.is_open:
            try:
                self._serial.close()
            except Exception:
                pass
        self._serial = None

    def _parse_gga(self, sentence: str):
        """Parse NMEA GGA sentence."""
        try:
            parts = sentence.split(',')
            if len(parts) < 15:
                return

            raw_lat = parts[2]
            lat_dir = parts[3]
            raw_lon = parts[4]
            lon_dir = parts[5]
            fix_quality = int(parts[6]) if parts[6] else 0
            num_sats = int(parts[7]) if parts[7] else 0
            hdop = float(parts[8]) if parts[8] else 99.9

            if raw_lat and raw_lon:
                lat_deg = int(raw_lat[:2])
                lat_min = float(raw_lat[2:])
                self._lat = lat_deg + lat_min / 60.0
                if lat_dir == 'S':
                    self._lat = -self._lat

                lon_deg = int(raw_lon[:3])
                lon_min = float(raw_lon[3:])
                self._lon = lon_deg + lon_min / 60.0
                if lon_dir == 'W':
                    self._lon = -self._lon

            if parts[9]:
                self._alt = float(parts[9])

            self._fix_quality = fix_quality
            self._num_sats = num_sats
            self._hdop = hdop

            # Publish
            self._publish_fix()
            self._publish_rtk_status()

        except (ValueError, IndexError):
            pass

    def _publish_fix(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.latitude = self._lat
        msg.longitude = self._lon
        msg.altitude = self._alt

        if self._fix_quality > 0:
            msg.status.status = NavSatStatus.STATUS_FIX
            msg.status.service = NavSatStatus.SERVICE_GPS
        else:
            msg.status.status = NavSatStatus.STATUS_NO_FIX

        # Position covariance from HDOP
        # Approximate: variance = (hdop * 2.5)^2 meters^2
        var = (self._hdop * 2.5) ** 2
        msg.position_covariance = [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, var]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self._fix_pub.publish(msg)

    def _publish_rtk_status(self):
        """Publish RTK fix quality as string."""
        msg = String()
        status = FIX_QUALITY_MAP.get(self._fix_quality, 'UNKNOWN')
        msg.data = status
        self._rtk_pub.publish(msg)

    def _publish_sim(self):
        """Publish fake GPS in simulation — Ankara coordinates, RTK_FIXED."""
        self._lat = 39.9250
        self._lon = 32.8660
        self._alt = 890.0
        self._fix_quality = 4  # RTK_FIXED
        self._num_sats = 14
        self._hdop = 0.8
        self._publish_fix()
        self._publish_rtk_status()

    def _publish_health(self):
        msg = eimsg.HealthDiag()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.component_name = 'gps_adapter'
        msg.level = eimsg.HealthDiag.LEVEL_OK
        msg.message = 'SIM' if not self._use_hw else 'OK'
        self._health_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GPSAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
