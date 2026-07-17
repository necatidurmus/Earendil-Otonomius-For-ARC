#!/usr/bin/env python3
# Safety Gate + E-Stop + Deadman + Watchdog + Sensor Health
# This is the MANDATORY safety node for the real vehicle.
#
# Command pipeline:
#   /cmd_vel_nav  ──┐
#   /cmd_vel_manual ─┤── [twist_mux] ── /cmd_vel_pre_safety ── [THIS] ── /cmd_vel
#   /cmd_vel_emergency┘
#
# The twist_mux handles priority selection.
# This node handles safety enforcement:
#   1. e-stop active → zero velocity output
#   2. deadman not held → zero velocity output
#   3. command timeout (watchdog) → zero velocity output
#   4. sensor critical failure → safe stop
#   5. speed limits enforced on all outputs

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry

import earendil_interfaces.msg as eimsg
import earendil_interfaces.srv as eisrv


class SafetyGateNode(Node):
    """Central safety node: e-stop, watchdog, deadman, sensor health, speed limiter."""

    def __init__(self):
        super().__init__('safety_gate_node')

        # -- Parameters --
        self.declare_parameter('watchdog_timeout_ms', 500.0)
        self.declare_parameter('deadman_timeout_ms', 1000.0)
        self.declare_parameter('sensor_timeout_ms', 2000.0)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('estop_gpio_pin', -1)  # -1 = disabled
        self.declare_parameter('estop_gpio_active_low', True)
        self.declare_parameter('input_topic', '/cmd_vel_pre_safety')
        self.declare_parameter('output_topic', '/cmd_vel')

        self._watchdog_timeout = self.get_parameter('watchdog_timeout_ms').value / 1000.0
        self._deadman_timeout = self.get_parameter('deadman_timeout_ms').value / 1000.0
        self._sensor_timeout = self.get_parameter('sensor_timeout_ms').value / 1000.0
        self._max_linear = self.get_parameter('max_linear_speed').value
        self._max_angular = self.get_parameter('max_angular_speed').value
        self._estop_pin = self.get_parameter('estop_gpio_pin').value
        self._estop_active_low = self.get_parameter('estop_gpio_active_low').value

        # -- State --
        self._estop_hardware = False
        self._estop_software = False
        self._deadman_held = False
        self._last_deadman_time = 0.0
        self._last_cmd_time = 0.0
        self._last_cmd = Twist()

        # Sensor timestamps
        self._last_gps_time = 0.0
        self._last_imu_time = 0.0
        self._last_lidar_time = 0.0
        self._last_odom_time = 0.0
        self._gps_ok = True
        self._imu_ok = True
        self._lidar_ok = True
        self._odom_ok = True

        # -- QoS --
        cmd_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # -- Publishers --
        output_topic = self.get_parameter('output_topic').value
        self._cmd_pub = self.create_publisher(Twist, output_topic, cmd_qos)
        self._state_pub = self.create_publisher(
            eimsg.VehicleState, '/vehicle_state', 10)
        self._safety_pub = self.create_publisher(
            eimsg.SafetyStatus, '/safety_status', 10)

        # -- Subscribers --
        # Single input from twist_mux output (priority already resolved)
        input_topic = self.get_parameter('input_topic').value
        self.create_subscription(Twist, input_topic, self._on_cmd, cmd_qos)
        self.create_subscription(Bool, '/deadman', self._on_deadman, cmd_qos)
        # Sensor topics for health monitoring
        self.create_subscription(NavSatFix, '/gps/fix', self._on_gps, sensor_qos)
        self.create_subscription(Imu, '/imu/data', self._on_imu, sensor_qos)
        self.create_subscription(Twist, '/scan', self._on_lidar_heartbeat, sensor_qos)
        self.create_subscription(Odometry, '/odom', self._on_odom, sensor_qos)

        # -- Services --
        self.create_service(
            eisrv.EmergencyStop, '/emergency_stop', self._on_estop_service)
        self.create_service(
            eisrv.SetSpeedLimit, '/set_speed_limit', self._on_set_speed_limit)

        # -- GPIO e-stop (optional) --
        self._gpio_available = False
        if self._estop_pin >= 0:
            try:
                import gpiod
                self._chip = gpiod.Chip('0')
                self._line = self._chip.get_line(self._estop_pin)
                self._line.request(
                    consumer='earendil_estop',
                    type=gpiod.LINE_REQ_DIR_IN,
                    default_vals=[0],
                )
                self._gpio_available = True
                self.get_logger().info(
                    f'GPIO e-stop on pin {self._estop_pin}')
            except Exception as e:
                self.get_logger().warn(
                    f'GPIO e-stop unavailable: {e}. Using software-only e-stop.')

        # -- Safety loop at 100Hz (10ms) --
        self._safety_timer = self.create_timer(0.01, self._safety_loop)
        # -- Status publish at 10Hz --
        self._status_timer = self.create_timer(0.1, self._publish_status)

        self.get_logger().info('=' * 55)
        self.get_logger().info('  SAFETY GATE NODE')
        self.get_logger().info(f'  Input topic      : {input_topic}')
        self.get_logger().info(f'  Output topic     : {output_topic}')
        self.get_logger().info(f'  Watchdog timeout : {self._watchdog_timeout*1000:.0f} ms')
        self.get_logger().info(f'  Deadman timeout  : {self._deadman_timeout*1000:.0f} ms')
        self.get_logger().info(f'  Sensor timeout   : {self._sensor_timeout*1000:.0f} ms')
        self.get_logger().info(f'  Max linear       : {self._max_linear} m/s')
        self.get_logger().info(f'  Max angular      : {self._max_angular} rad/s')
        self.get_logger().info(f'  GPIO e-stop pin  : {self._estop_pin}')
        self.get_logger().info('=' * 55)

    # -- E-stop hardware (GPIO polling) --
    def _poll_estop_gpio(self) -> bool:
        if not self._gpio_available:
            return False
        try:
            val = self._line.get_value()
            if self._estop_active_low:
                return val == 0
            return val == 1
        except Exception:
            return False

    # -- Callbacks --
    def _on_cmd(self, msg: Twist):
        """Receive velocity command from twist_mux output."""
        self._last_cmd = msg
        self._last_cmd_time = time.time()

    def _on_deadman(self, msg: Bool):
        self._deadman_held = msg.data
        if msg.data:
            self._last_deadman_time = time.time()

    def _on_gps(self, msg: NavSatFix):
        self._last_gps_time = time.time()

    def _on_imu(self, msg: Imu):
        self._last_imu_time = time.time()

    def _on_lidar_heartbeat(self, msg):
        self._last_lidar_time = time.time()

    def _on_odom(self, msg: Odometry):
        self._last_odom_time = time.time()

    # -- Services --
    def _on_estop_service(self, req, res):
        self._estop_software = req.engage
        if req.engage:
            self.get_logger().warn('SOFTWARE E-STOP ENGAGED')
            self._publish_zero()
        else:
            self.get_logger().info('Software e-stop released')
        res.success = True
        res.message = 'E-stop engaged' if req.engage else 'E-stop released'
        return res

    def _on_set_speed_limit(self, req, res):
        if req.max_linear > 0:
            self._max_linear = req.max_linear
        if req.max_angular > 0:
            self._max_angular = req.max_angular
        self.get_logger().info(
            f'Speed limits: linear={self._max_linear} m/s, '
            f'angular={self._max_angular} rad/s')
        res.success = True
        res.message = 'Speed limits updated'
        return res

    # -- Safety loop (100Hz) --
    def _safety_loop(self):
        now = time.time()

        # 1. Check hardware e-stop
        self._estop_hardware = self._poll_estop_gpio()

        # 2. Any e-stop active?
        estop_active = self._estop_hardware or self._estop_software

        # 3. Watchdog: is the command source (twist_mux) alive?
        cmd_age = now - self._last_cmd_time if self._last_cmd_time > 0 else float('inf')
        watchdog_ok = cmd_age < self._watchdog_timeout

        # 4. Deadman check
        deadman_age = now - self._last_deadman_time if self._last_deadman_time > 0 else float('inf')
        deadman_ok = self._deadman_held and deadman_age < self._deadman_timeout

        # 5. Sensor health
        self._gps_ok = (now - self._last_gps_time) < self._sensor_timeout if self._last_gps_time > 0 else True
        self._imu_ok = (now - self._last_imu_time) < self._sensor_timeout if self._last_imu_time > 0 else True
        self._lidar_ok = (now - self._last_lidar_time) < self._sensor_timeout if self._last_lidar_time > 0 else True
        self._odom_ok = (now - self._last_odom_time) < self._sensor_timeout if self._last_odom_time > 0 else True

        # Store for status publisher
        self._watchdog_ok = watchdog_ok
        self._watchdog_age = cmd_age

        # 6. Determine output
        if estop_active:
            self._publish_zero()
            return

        if not watchdog_ok:
            self._publish_zero()
            return

        # If deadman is required and not held, force zero
        if not deadman_ok:
            self._publish_zero()
            return

        # Apply speed limits and forward
        self._cmd_pub.publish(self._limit(self._last_cmd))

    def _limit(self, cmd: Twist) -> Twist:
        out = Twist()
        out.linear.x = max(-self._max_linear, min(self._max_linear, cmd.linear.x))
        out.angular.z = max(-self._max_angular, min(self._max_angular, cmd.angular.z))
        return out

    def _publish_zero(self):
        self._cmd_pub.publish(Twist())

    # -- Status publisher (10Hz) --
    def _publish_status(self):
        now = time.time()
        stamp = self.get_clock().now().to_msg()

        # SafetyStatus
        ss = eimsg.SafetyStatus()
        ss.header.stamp = stamp
        ss.estop_hardware_pin = self._estop_hardware
        ss.estop_software = self._estop_software
        ss.estop_active = self._estop_hardware or self._estop_software
        ss.watchdog_ok = getattr(self, '_watchdog_ok', True)
        ss.watchdog_age_ms = getattr(self, '_watchdog_age', 0.0) * 1000.0
        ss.watchdog_timeout_ms = self._watchdog_timeout * 1000.0
        ss.deadman_held = self._deadman_held
        ss.deadman_age_ms = (now - self._last_deadman_time) * 1000.0 if self._last_deadman_time > 0 else float('inf')
        ss.gps_ok = self._gps_ok
        ss.imu_ok = self._imu_ok
        ss.lidar_ok = self._lidar_ok
        ss.odom_ok = self._odom_ok
        ss.gps_age_ms = (now - self._last_gps_time) * 1000.0 if self._last_gps_time > 0 else float('inf')
        ss.imu_age_ms = (now - self._last_imu_time) * 1000.0 if self._last_imu_time > 0 else float('inf')
        ss.lidar_age_ms = (now - self._last_lidar_time) * 1000.0 if self._last_lidar_time > 0 else float('inf')
        ss.odom_age_ms = (now - self._last_odom_time) * 1000.0 if self._last_odom_time > 0 else float('inf')
        ss.current_linear_limit = self._max_linear
        ss.current_angular_limit = self._max_angular
        self._safety_pub.publish(ss)

        # VehicleState
        vs = eimsg.VehicleState()
        vs.header.stamp = stamp
        vs.e_stop_active = ss.estop_active
        vs.deadman_active = self._deadman_held
        vs.sensor_fault = not (self._gps_ok and self._imu_ok and self._lidar_ok and self._odom_ok)
        vs.max_linear_speed = self._max_linear
        vs.max_angular_speed = self._max_angular
        self._state_pub.publish(vs)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_zero()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
