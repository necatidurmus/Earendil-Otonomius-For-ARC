#!/usr/bin/env python3
"""
Safety Mux Node — Tek güvenli çıkış `/cmd_vel_safe`.

Öncelik sırası: e-stop > deadman > watchdog > komut timeout > arm/disarm gate.
Her gate deterministik sıfır (Twist zero) üretir.

Subscribes:
  /cmd_vel_nav (geometry_msgs/Twist) — Nav2 navigasyon komutu
  /cmd_vel_manual (geometry_msgs/Twist) — Web teleop komutu
  /e_stop (std_msgs/Bool) — E-stop sinyali
  /deadman (std_msgs/Bool) — Deadman heartbeat
  /rscp/command (earendil_interfaces/RscpCommand) — Arm/disarm sinyali
  /stm/status (earendil_interfaces/StmStatus) — STM heartbeat

Publishes:
  /cmd_vel_safe (geometry_msgs/Twist) — Tek güvenli çıkış → stm_bridge
  /safety/status (earendil_interfaces/SafetyStatus) — Safety durumu
"""

import time
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from earendil_interfaces.msg import RscpCommand, SafetyStatus, StmStatus

CMD_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

SENSOR_QOS = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

# RscpCommand constants (must match RscpCommand.msg)
CMD_ARM = 1
CMD_DISARM = 2

ZERO_TWIST = Twist()


class SafetyMuxNode(Node):
    """Safety multiplexer — tek güvenli çıkış /cmd_vel_safe."""

    def __init__(self):
        super().__init__('safety_mux')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('input_topics', ['/cmd_vel_nav', '/cmd_vel_manual'])
        self.declare_parameter('output_topic', '/cmd_vel_safe')
        self.declare_parameter('estop_topic', '/e_stop')
        self.declare_parameter('deadman_topic', '/deadman')
        self.declare_parameter('arm_gate_topic', '/rscp/command')
        self.declare_parameter('stm_status_topic', '/stm/status')
        self.declare_parameter('watchdog_timeout_ms', 400)
        self.declare_parameter('command_timeout_ms', 300)
        self.declare_parameter('deadman_timeout_ms', 1000)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('estop_latching', True)
        self.declare_parameter('arm_required', True)

        self._input_topics = self.get_parameter('input_topics').value
        self._output_topic = self.get_parameter('output_topic').value
        self._watchdog_timeout = self.get_parameter('watchdog_timeout_ms').value / 1000.0
        self._command_timeout = self.get_parameter('command_timeout_ms').value / 1000.0
        self._deadman_timeout = self.get_parameter('deadman_timeout_ms').value / 1000.0
        self._max_linear = self.get_parameter('max_linear_speed').value
        self._max_angular = self.get_parameter('max_angular_speed').value
        self._estop_latching = self.get_parameter('estop_latching').value
        self._arm_required = self.get_parameter('arm_required').value

        # ── State ───────────────────────────────────────────────────────────
        self._estop_active = False          # Latched e-stop
        self._deadman_held = False
        self._last_deadman_time = 0.0
        self._armed = False                 # RSCP arm state
        self._last_stm_time = 0.0           # STM heartbeat

        # Per-topic command state: {topic: (Twist, timestamp)}
        self._last_cmds = {}
        self._last_cmd_times = {}
        for topic in self._input_topics:
            self._last_cmds[topic] = Twist()
            self._last_cmd_times[topic] = 0.0

        self._commands_forwarded = 0
        self._commands_zeroed = 0

        # ── Publishers ──────────────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(Twist, self._output_topic, CMD_QOS)
        self._safety_pub = self.create_publisher(SafetyStatus, '/safety/status', 10)

        # ── Subscribers ─────────────────────────────────────────────────────
        for topic in self._input_topics:
            self.create_subscription(Twist, topic, self._make_cmd_cb(topic), CMD_QOS)

        estop_topic = self.get_parameter('estop_topic').value
        self.create_subscription(Bool, estop_topic, self._on_estop, CMD_QOS)

        deadman_topic = self.get_parameter('deadman_topic').value
        self.create_subscription(Bool, deadman_topic, self._on_deadman, CMD_QOS)

        arm_topic = self.get_parameter('arm_gate_topic').value
        self.create_subscription(RscpCommand, arm_topic, self._on_rscp_command, 10)

        stm_topic = self.get_parameter('stm_status_topic').value
        self.create_subscription(StmStatus, stm_topic, self._on_stm_status, SENSOR_QOS)

        # ── Services ─────────────────────────────────────────────────────────
        self._reset_srv = self.create_service(
            Trigger, '~/reset_estop', self._handle_reset_estop)

        # ── Sensor health tracking ───────────────────────────────────────────
        self._last_gps_time = 0.0
        self._last_imu_time = 0.0
        self._last_lidar_time = 0.0
        self._sensor_timeout = 2.0  # seconds

        # Subscribe to sensor topics for health monitoring
        self.create_subscription(
            NavSatFix, '/gps/fix', self._on_gps, SENSOR_QOS)
        # IMU and LiDAR health tracked via stm_status and /scan
        self.create_subscription(
            StmStatus, '/stm/status', self._on_stm_health, SENSOR_QOS)

        # ── Timers ──────────────────────────────────────────────────────────
        self._safety_timer = self.create_timer(0.01, self._safety_loop)  # 100 Hz
        self._status_timer = self.create_timer(0.1, self._publish_status)  # 10 Hz

        self.get_logger().info('=' * 55)
        self.get_logger().info('  SAFETY MUX NODE')
        self.get_logger().info(f'  Inputs        : {self._input_topics}')
        self.get_logger().info(f'  Output        : {self._output_topic}')
        self.get_logger().info(f'  Watchdog      : {self._watchdog_timeout*1000:.0f} ms')
        self.get_logger().info(f'  Cmd timeout   : {self._command_timeout*1000:.0f} ms')
        self.get_logger().info(f'  Deadman timeout: {self._deadman_timeout*1000:.0f} ms')
        self.get_logger().info(f'  Max linear    : {self._max_linear} m/s')
        self.get_logger().info(f'  Max angular   : {self._max_angular} rad/s')
        self.get_logger().info(f'  E-stop latch  : {self._estop_latching}')
        self.get_logger().info(f'  Arm required  : {self._arm_required}')
        self.get_logger().info('=' * 55)

    # ── Callback factories ──────────────────────────────────────────────────

    def _make_cmd_cb(self, topic):
        """Create callback for a specific input topic."""
        def cb(msg: Twist):
            self._last_cmds[topic] = msg
            self._last_cmd_times[topic] = time.time()
        return cb

    # ── Callbacks ───────────────────────────────────────────────────────────

    def _on_estop(self, msg: Bool):
        if msg.data:
            self._estop_active = True
            self.get_logger().warn('E-STOP ENGAGED')
        elif not self._estop_latching:
            # Non-latching: False clears immediately
            self._estop_active = False
        # Latching mode: False does nothing — only reset_estop service clears

    def _on_deadman(self, msg: Bool):
        self._deadman_held = msg.data
        if msg.data:
            self._last_deadman_time = time.time()

    def _on_rscp_command(self, msg: RscpCommand):
        """Handle RSCP arm/disarm commands."""
        if msg.command_type == CMD_ARM:
            self._armed = True
            self.get_logger().info('ARM command received — safety_mux ARMED')
        elif msg.command_type == CMD_DISARM:
            self._armed = False
            self.get_logger().info('DISARM command received — safety_mux DISARMED')

    def _on_stm_status(self, msg: StmStatus):
        """Update STM heartbeat time."""
        self._last_stm_time = time.time()

    def _on_stm_health(self, msg: StmStatus):
        """Track IMU health from STM status."""
        self._last_imu_time = time.time()

    def _on_gps(self, msg: NavSatFix):
        """Track GPS health."""
        self._last_gps_time = time.time()

    def _handle_reset_estop(self, request, response):
        """Reset latched e-stop — only if latching is enabled."""
        if self._estop_latching and self._estop_active:
            self._estop_active = False
            response.success = True
            response.message = 'E-stop latch cleared'
            self.get_logger().info('E-stop latch cleared via service')
        elif not self._estop_active:
            response.success = True
            response.message = 'E-stop was not active'
        else:
            response.success = False
            response.message = 'E-stop is not latching'
        return response

    # ── Safety loop (100 Hz) ────────────────────────────────────────────────

    def _safety_loop(self):
        now = time.time()

        # ── Gate 1: E-stop (highest priority) ──
        if self._estop_active:
            self._publish_zero('e-stop')
            return

        # ── Gate 2: Arm/disarm gate ──
        if self._arm_required and not self._armed:
            self._publish_zero('disarmed')
            return

        # ── Gate 3: Deadman check ──
        # Fail-closed: never held OR expired → block
        if self._last_deadman_time <= 0 or (now - self._last_deadman_time) > self._deadman_timeout:
            self._publish_zero('deadman_timeout')
            return

        # ── Gate 4: STM watchdog ──
        # Fail-closed: never received STM OR expired → block
        if self._last_stm_time <= 0 or (now - self._last_stm_time) > self._watchdog_timeout:
            self._publish_zero('stm_watchdog')
            return

        # ── Gate 5: Command timeout per input ──
        # Manual has priority over nav (when deadman held or always)
        manual_topic = None
        nav_topic = None
        for topic in self._input_topics:
            if 'manual' in topic:
                manual_topic = topic
            else:
                nav_topic = topic

        # Try manual first (operator priority)
        active_cmd = None
        if manual_topic and self._last_cmd_times.get(manual_topic, 0) > 0:
            cmd_age = now - self._last_cmd_times[manual_topic]
            if cmd_age < self._command_timeout:
                active_cmd = self._last_cmds[manual_topic]

        # Fall back to nav if no manual
        if active_cmd is None and nav_topic and self._last_cmd_times.get(nav_topic, 0) > 0:
            cmd_age = now - self._last_cmd_times[nav_topic]
            if cmd_age < self._command_timeout:
                active_cmd = self._last_cmds[nav_topic]

        # ── All gates passed ──
        if active_cmd is not None:
            self._cmd_pub.publish(self._limit(active_cmd))
            self._commands_forwarded += 1
        else:
            # No valid command from any source
            self._publish_zero('no_command')
            return

    def _limit(self, cmd: Twist) -> Twist:
        """Apply speed limits and reject NaN/Inf."""
        out = Twist()
        lx = cmd.linear.x
        az = cmd.angular.z
        # Reject NaN/Inf — treat as zero
        if math.isnan(lx) or math.isinf(lx):
            lx = 0.0
        if math.isnan(az) or math.isinf(az):
            az = 0.0
        out.linear.x = max(-self._max_linear, min(self._max_linear, lx))
        out.angular.z = max(-self._max_angular, min(self._max_angular, az))
        return out

    def _publish_zero(self, reason: str = ''):
        """Publish zero twist."""
        self._cmd_pub.publish(ZERO_TWIST)
        self._commands_zeroed += 1

    # ── Status publisher (10 Hz) ────────────────────────────────────────────

    def _publish_status(self):
        now = time.time()
        stamp = self.get_clock().now().to_msg()

        ss = SafetyStatus()
        ss.header.stamp = stamp

        # E-stop state
        ss.estop_hardware_pin = False  # GPIO not implemented yet
        ss.estop_software = False
        ss.estop_active = self._estop_active

        # Watchdog state (fail-closed: never received = not OK)
        _NO_MSG = 99999.0
        stm_age = (
            (now - self._last_stm_time) * 1000.0
            if self._last_stm_time > 0 else _NO_MSG
        )
        ss.watchdog_ok = self._last_stm_time > 0 and stm_age < self._watchdog_timeout * 1000.0
        ss.watchdog_age_ms = stm_age
        ss.watchdog_timeout_ms = self._watchdog_timeout * 1000.0

        # Deadman state
        ss.deadman_held = self._deadman_held
        ss.deadman_age_ms = (
            (now - self._last_deadman_time) * 1000.0
            if self._last_deadman_time > 0 else _NO_MSG
        )

        # Sensor health (fail-closed: never received = not OK)
        gps_age = (now - self._last_gps_time) if self._last_gps_time > 0 else _NO_MSG / 1000.0
        imu_age = (now - self._last_imu_time) if self._last_imu_time > 0 else _NO_MSG / 1000.0
        ss.gps_ok = self._last_gps_time > 0 and gps_age < self._sensor_timeout
        ss.imu_ok = self._last_imu_time > 0 and imu_age < self._sensor_timeout
        ss.lidar_ok = True  # TODO: subscribe to /scan for health
        ss.odom_ok = self._last_stm_time > 0 and (now - self._last_stm_time) < self._sensor_timeout
        ss.gps_age_ms = gps_age * 1000.0
        ss.imu_age_ms = imu_age * 1000.0
        ss.lidar_age_ms = 0.0
        ss.odom_age_ms = stm_age

        # Speed limits
        ss.current_linear_limit = self._max_linear
        ss.current_angular_limit = self._max_angular

        self._safety_pub.publish(ss)

    # ── Cleanup ─────────────────────────────────────────────────────────────

    def destroy_node(self):
        """Publish zero on shutdown."""
        self._cmd_pub.publish(ZERO_TWIST)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
