#!/usr/bin/env python3
"""
Mission Manager — RSCP stage-aware ARC 2026 mission orchestrator.

Converts RSCP commands into autonomous mission flow.
Stage state machine per ROADMAP.md §7:
  Stage 1: Antenna Installation (SearchArea → GPSCoordinate → TaskFinished)
  Stage 2: Shackleton Crater (SearchArea → GPSCoordinate → TaskFinished)
  Stage 3: Lava Tube (NavigateToGPS → TaskFinished → StartExploration → distance → TaskFinished)
  Stage 4: Return to Airlock (NavigateToGPS → TaskFinished → ArmDisarm(false) → Ack)

Subscribes:
  /rscp/command (earendil_interfaces/RscpCommand) — RSCP commands
  /rscp/current_stage (std_msgs/UInt32) — current stage
  /mission/command (earendil_interfaces/RscpCommand) — mission commands from RSCP bridge
  /gps/fix (sensor_msgs/NavSatFix) — RTK GPS
  /rtk/status (std_msgs/String) — RTK quality
  /stm/wheel_odom (nav_msgs/Odometry) — wheel odometry for distance
  /jetson/aruco_detections (earendil_interfaces/JetsonArucoDetections) — ArUco tags

Publishes:
  /mission/result (std_msgs/String) — "task_finished", "gps_coordinate:lat,lon,alt", "distance:X"
  /mission/status (earendil_interfaces/MissionStatus) — mission state
  /cmd_vel_nav (geometry_msgs/Twist) — Nav2 commands (or direct velocity)
"""

import math
import time
import os
import csv
import json
import logging
from enum import IntEnum
from typing import Optional, List, Dict

from earendil_navigation.gps_utils import GPSOrigin, gps_to_local
from earendil_navigation.search_executor import SearchExecutor
from earendil_navigation.exploration_executor import ExplorationExecutor
from earendil_navigation.mission_report import (
    PathSample, WaypointRecord,
    compute_metrics, build_mission_report,
    import_waypoints_csv, export_waypoints_csv,
    load_preset_yaml, parse_sweep_config,
)

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, UInt32
from std_srvs.srv import Empty as EmptySrv

from earendil_interfaces.msg import (
    RscpCommand, MissionStatus, MissionCommand,
    JetsonArucoDetections, StmStatus,
)

# RscpCommand constants (RSCP kökenli, sabit — must match RscpCommand.msg + rscp.proto)
CMD_SET_STAGE = 0
CMD_ARM = 1
CMD_DISARM = 2
CMD_NAVIGATE_TO_GPS = 3
CMD_SEARCH_AREA = 4
CMD_START_EXPLORATION = 5

# MissionCommand constants (dahili lifecycle, RSCP'den bağımsız)
MCMD_PAUSE = 0
MCMD_RESUME = 1
MCMD_CANCEL = 2
MCMD_RESET = 3
MCMD_SKIP = 4
MCMD_LOAD_PRESET = 5
MCMD_LOAD_CSV = 6
MCMD_START_SWEEP = 7

logger = logging.getLogger('mission_manager')


SENSOR_QOS = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class StageState(IntEnum):
    """Mission state per ARC stage."""
    IDLE = 0
    # Stage 1-2
    WAIT_STAGE = 10
    WAIT_SEARCH = 11
    SEARCHING = 12
    FOUND = 13
    # Stage 3
    WAIT_NAV = 20
    NAVIGATING = 21
    REACHED_ENTRY = 22
    WAIT_EXPLORE = 23
    EXPLORING = 24
    FOUND_EXIT = 25
    # Stage 4
    DOCKING = 30
    REACHED = 31
    WAIT_DISARM = 32
    # Common
    DONE = 99
    ERROR = 100


class LifecycleState(IntEnum):
    """Üst lifecycle katmanı — RSCP'den bağımsız, operatör kontrolü."""
    RUNNING = 0
    PAUSED = 1
    CANCELLED = 2


class MissionManagerNode(Node):
    """RSCP stage-aware mission manager for ARC 2026."""

    def __init__(self):
        super().__init__('mission_manager')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('arrival_tolerance_m', 1.0)
        self.declare_parameter('rtk_fixed_tolerance_m', 0.3)
        self.declare_parameter('rtk_float_tolerance_m', 0.4)
        self.declare_parameter('rtk_no_fix_action', 'stop')
        self.declare_parameter('search_pattern', 'spiral')
        self.declare_parameter('search_speed_mps', 0.3)
        self.declare_parameter('exploration_timeout_s', 300)
        self.declare_parameter('waypoint_timeout_s', 120)
        self.declare_parameter('max_retry_count', 3)
        self.declare_parameter('map_frame', 'map')

        # Lifecycle params (M7)
        self.declare_parameter('stuck_window_s', 10.0)
        self.declare_parameter('stuck_min_progress_m', 0.2)
        self.declare_parameter('stuck_check_source', 'odom')
        self.declare_parameter('report_sample_hz', 2.0)
        self.declare_parameter('log_path', '/var/log/earendil/missions/')
        self.declare_parameter('rtk_degrade_timeout_s', 5.0)
        self.declare_parameter('rtk_recover_timeout_s', 5.0)
        self.declare_parameter('presets_dir', '')

        self._arrival_tol = self.get_parameter('arrival_tolerance_m').value
        self._rtk_fixed_tol = self.get_parameter('rtk_fixed_tolerance_m').value
        self._rtk_float_tol = self.get_parameter('rtk_float_tolerance_m').value
        self._rtk_no_fix = self.get_parameter('rtk_no_fix_action').value
        self._search_pattern = self.get_parameter('search_pattern').value
        self._search_speed = self.get_parameter('search_speed_mps').value
        self._explore_timeout = self.get_parameter('exploration_timeout_s').value
        self._wp_timeout = self.get_parameter('waypoint_timeout_s').value
        self._max_retry = self.get_parameter('max_retry_count').value
        self._map_frame = self.get_parameter('map_frame').value
        self._stuck_window = self.get_parameter('stuck_window_s').value
        self._stuck_min_progress = self.get_parameter('stuck_min_progress_m').value
        self._stuck_source = self.get_parameter('stuck_check_source').value
        self._report_sample_hz = self.get_parameter('report_sample_hz').value
        self._log_path = self.get_parameter('log_path').value
        self._rtk_degrade_timeout = self.get_parameter('rtk_degrade_timeout_s').value
        self._rtk_recover_timeout = self.get_parameter('rtk_recover_timeout_s').value
        self._presets_dir = self.get_parameter('presets_dir').value


        # ── State ───────────────────────────────────────────────────────────
        self._current_stage = 0
        self._stage_state = StageState.IDLE
        self._lifecycle = LifecycleState.RUNNING
        self._armed = False

        # Current mission target
        self._target_lat = 0.0
        self._target_lon = 0.0
        self._target_alt = 0.0
        self._search_center_lat = 0.0
        self._search_center_lon = 0.0
        self._search_radius = 0.0

        # GPS state
        self._gps_lat = 0.0
        self._gps_lon = 0.0
        self._gps_alt = 0.0
        self._rtk_status = 'NO_FIX'
        self._last_gps_time = 0.0
        self._gps_origin = None  # GPSOrigin set on first fix

        # Exploration state — delegated to ExplorationExecutor

        # Waypoint tracking
        self._current_wp_idx = 0
        self._search_waypoints = []
        self._retry_count = 0
        self._mission_start_time = 0.0

        # ArUco
        self._last_aruco = None

        # Lifecycle state (RSCP'den bağımsız)
        self._paused_target = None  # pause'ta kayıtlı hedef (lat, lon, alt)

        # Stuck algılama
        self._stuck_check_pos = None  # (x, y) veya (lat, lon)
        self._stuck_timer_start = 0.0

        # Odom (stuck + report için)
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_vx = 0.0
        self._odom_vy = 0.0
        self._last_odom_time = 0.0

        # Mission report
        self._mission_id = ''
        self._mission_counter = 0
        self._report_path_samples: List[PathSample] = []
        self._report_waypoints: List[WaypointRecord] = []
        self._report_result_messages: List[str] = []
        self._report_last_sample_time = 0.0
        self._report_written = False

        # GPS↔SLAM mode
        self._loc_mode = 'gps'
        self._rtk_bad_since = 0.0
        self._rtk_good_since = 0.0

        # Sweep
        self._sweep_config = None      # {"param":..., "values":[...]}
        self._sweep_idx = 0
        self._sweep_reports: List[Dict] = []
        self._sweep_pending_cmd = None  # mission başlatmak için bekleyen RSCP komutu

        # ── Nav2 Action Client ──────────────────────────────────────────────
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ── Executors ───────────────────────────────────────────────────────
        self._search_executor = SearchExecutor(
            pattern=self._search_pattern, speed_mps=self._search_speed)
        self._exploration_executor = ExplorationExecutor(
            timeout_s=self._explore_timeout)
        self._goal_handle = None

        # ── Publishers ──────────────────────────────────────────────────────
        self._result_pub = self.create_publisher(String, '/mission/result', 10)
        self._status_pub = self.create_publisher(MissionStatus, '/mission/status', 10)
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self._loc_mode_pub = self.create_publisher(String, '/localization/mode', 10)

        # ── Subscribers ─────────────────────────────────────────────────────
        self.create_subscription(RscpCommand, '/rscp/command', self._on_rscp_command, 10)
        self.create_subscription(UInt32, '/rscp/current_stage', self._on_stage_update, 10)
        self.create_subscription(RscpCommand, '/mission/command', self._on_mission_command, 10)
        self.create_subscription(MissionCommand, '/mission/control', self._on_mission_control, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self._on_gps, SENSOR_QOS)
        self.create_subscription(String, '/rtk/status', self._on_rtk, SENSOR_QOS)
        self.create_subscription(
            Odometry, '/stm/wheel_odom', self._on_odom, SENSOR_QOS
        )
        self.create_subscription(
            JetsonArucoDetections, '/jetson/aruco_detections',
            self._on_aruco, SENSOR_QOS
        )

        # ── Costmap clearing service clients (phase transition cleanup) ────
        self._clear_local_costmap = self.create_client(
            EmptySrv, '/local_costmap/clear_entirely_local_costmap')
        self._clear_global_costmap = self.create_client(
            EmptySrv, '/global_costmap/clear_entirely_global_costmap')

        # ── Timers ──────────────────────────────────────────────────────────
        self._status_timer = self.create_timer(0.5, self._publish_status)
        self._mission_timer = self.create_timer(0.1, self._mission_tick)

        self.get_logger().info(
            f'Mission Manager started — arrival_tol={self._arrival_tol}m, '
            f'search={self._search_pattern}'
        )

    # ── Callbacks ───────────────────────────────────────────────────────────

    def _on_rscp_command(self, msg: RscpCommand):
        """Handle RSCP commands (direct from bridge)."""
        self._dispatch_command(msg)

    def _on_mission_command(self, msg: RscpCommand):
        """Handle mission commands (from bridge via /mission/command)."""
        self._dispatch_command(msg)

    def _on_stage_update(self, msg: UInt32):
        """Handle stage update from RSCP bridge."""
        self._current_stage = msg.data
        self.get_logger().info(f'Stage updated to {self._current_stage}')

    def _on_gps(self, msg: NavSatFix):
        self._gps_lat = msg.latitude
        self._gps_lon = msg.longitude
        self._gps_alt = msg.altitude
        self._last_gps_time = time.time()
        # Set datum on first valid GPS fix
        if self._gps_origin is None and msg.latitude != 0.0:
            self._gps_origin = GPSOrigin(lat0=msg.latitude, lon0=msg.longitude)
            self.get_logger().info(
                f'GPS datum set: lat={msg.latitude:.8f}, lon={msg.longitude:.8f}')

    def _on_rtk(self, msg: String):
        self._rtk_status = msg.data

    def _on_aruco(self, msg: JetsonArucoDetections):
        self._last_aruco = msg

    def _on_odom(self, msg: Odometry):
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        self._odom_vx = msg.twist.twist.linear.x
        self._odom_vy = msg.twist.twist.linear.y
        self._last_odom_time = time.time()

    # ── Lifecycle commands (dahili, RSCP'den bağımsız) ───────────────────────

    def _on_mission_control(self, msg: MissionCommand):
        """Handle lifecycle commands from /mission/control (web/operator)."""
        ct = msg.command_type
        if ct == MCMD_PAUSE:
            self._handle_pause()
        elif ct == MCMD_RESUME:
            self._handle_resume()
        elif ct == MCMD_CANCEL:
            self._handle_cancel()
        elif ct == MCMD_RESET:
            self._handle_reset()
        elif ct == MCMD_SKIP:
            self._handle_skip()
        elif ct == MCMD_LOAD_PRESET:
            self._handle_load_preset(msg.payload)
        elif ct == MCMD_LOAD_CSV:
            self._handle_load_csv(msg.payload)
        elif ct == MCMD_START_SWEEP:
            self._handle_start_sweep(msg.payload)
        else:
            self.get_logger().warn(f'Unknown MissionCommand type: {ct}')

    def _handle_pause(self):
        if self._lifecycle != LifecycleState.RUNNING:
            self.get_logger().warn(f'Pause ignored — lifecycle={self._lifecycle.name}')
            return
        self._lifecycle = LifecycleState.PAUSED
        self._cancel_current_goal()
        # Defense-in-depth: /cmd_vel_nav'e sıfır bas (safety_mux'tan bağımsız)
        self._cmd_vel_pub.publish(Twist())
        # Resume için hedefi kaydet
        self._paused_target = (
            self._target_lat, self._target_lon, self._target_alt)
        self.get_logger().info('PAUSED — goal cancelled, holding position')

    def _handle_resume(self):
        if self._lifecycle != LifecycleState.PAUSED:
            self.get_logger().warn(f'Resume ignored — lifecycle={self._lifecycle.name}')
            return
        self._lifecycle = LifecycleState.RUNNING
        self._retry_count = 0
        self.get_logger().info('RESUMED — re-sending last target')
        if self._paused_target is not None:
            lat, lon, alt = self._paused_target
            self._target_lat, self._target_lon, self._target_alt = (
                lat, lon, alt)
            # Stage-aware resume: search'te aynı idx, nav'da hedef
            if self._stage_state == StageState.SEARCHING:
                self._send_search_waypoint()
            else:
                self._start_navigation()
        else:
            self.get_logger().warn('Resume — no paused target, idle')

    def _handle_cancel(self):
        self._lifecycle = LifecycleState.CANCELLED
        self._cancel_current_goal()
        self._cmd_vel_pub.publish(Twist())  # Motor stop (defense-in-depth)
        self._send_result('cancelled')
        self.get_logger().info('CANCELLED — mission aborted, motor stop')
        self._finalize_report('cancelled')

    def _handle_reset(self):
        self._cancel_current_goal()
        self._cmd_vel_pub.publish(Twist())
        self._current_stage = 0
        self._stage_state = StageState.IDLE
        self._lifecycle = LifecycleState.RUNNING
        self._current_wp_idx = 0
        self._retry_count = 0
        self._exploration_executor.reset()
        self._search_waypoints = []
        self._report_path_samples = []
        self._report_waypoints = []
        self._report_result_messages = []
        self._report_written = False
        self._paused_target = None
        # Mode GPS'e sıfırla
        self._set_loc_mode('gps')
        self.get_logger().info('RESET — all mission state cleared')

    def _handle_skip(self):
        if self._lifecycle != LifecycleState.RUNNING:
            self.get_logger().warn(f'Skip ignored — lifecycle={self._lifecycle.name}')
            return
        self.get_logger().info(f'SKIP — skipping current sub-task '
                               f'(stage_state={self._stage_state.name})')
        self._cancel_current_goal()
        if self._stage_state == StageState.SEARCHING:
            idx = self._current_wp_idx
            if idx < len(self._report_waypoints):
                self._report_waypoints[idx].skipped = True
            self._retry_count = 0
            self._current_wp_idx += 1
            self._send_search_waypoint()
        else:
            # Nav/explore: arrival gibi say
            self._handle_arrival()

    # ── Command dispatch ────────────────────────────────────────────────────


    def _dispatch_command(self, cmd: RscpCommand):
        """Dispatch RSCP command to appropriate handler.

        RSCP komutları CM'den gelir — lifecycle'ı kırar (override pause).
        ArmDisarm(false) güçlü stop, her şeyden üstün.
        """
        # RSCP override: CM komutu pause'u/cancel'i kırar
        if self._lifecycle in (LifecycleState.PAUSED, LifecycleState.CANCELLED):
            if cmd.command_type in (CMD_SET_STAGE, CMD_NAVIGATE_TO_GPS,
                                    CMD_SEARCH_AREA, CMD_START_EXPLORATION):
                self.get_logger().info(
                    f'RSCP override — breaking {self._lifecycle.name} for '
                    f'CM command {cmd.command_type}')
                self._lifecycle = LifecycleState.RUNNING
                self._cancel_current_goal()

        if cmd.command_type == CMD_SET_STAGE:
            self._handle_set_stage(cmd.stage_value)
        elif cmd.command_type == CMD_ARM:
            self._handle_arm(True)
        elif cmd.command_type == CMD_DISARM:
            self._handle_arm(False)
        elif cmd.command_type == CMD_NAVIGATE_TO_GPS:
            self._handle_navigate(cmd.latitude, cmd.longitude, cmd.altitude)
        elif cmd.command_type == CMD_SEARCH_AREA:
            self._handle_search(cmd.latitude, cmd.longitude, cmd.search_radius)
        elif cmd.command_type == CMD_START_EXPLORATION:
            self._handle_start_exploration()
        else:
            self.get_logger().warn(f'Unknown command type: {cmd.command_type}')

    def _handle_set_stage(self, stage: int):
        """Set current ARC stage."""
        if stage < 1 or stage > 4:
            self.get_logger().error(f'Invalid stage: {stage}')
            self._send_result('message:invalid_stage')
            return

        self._current_stage = stage
        self._stage_state = StageState.WAIT_STAGE
        self._retry_count = 0
        self._start_new_mission_report()
        self._clear_costmaps()
        self.get_logger().info(f'Stage set to {stage}')
        # Acknowledge is sent by RSCP bridge

    def _handle_arm(self, arm: bool):
        """Handle arm/disarm command.

        ArmDisarm(false) = disarm = güçlü stop, lifecycle'dan üstün.
        """
        self._armed = arm
        if arm:
            self.get_logger().info('ARMED — autonomous mode active')
        else:
            self.get_logger().info('DISARMED — stopping all motion')
            self._cancel_current_goal()
            self._cmd_vel_pub.publish(Twist())  # Zero velocity
            self._stage_state = StageState.IDLE
            self._finalize_report('disarmed')

    def _handle_navigate(self, lat: float, lon: float, alt: float):
        """Handle NavigateToGPS command."""
        self._target_lat = lat
        self._target_lon = lon
        self._target_alt = alt
        self._stage_state = StageState.WAIT_NAV
        self._retry_count = 0
        self._mission_start_time = time.time()
        self._record_waypoint(lat, lon)
        self._clear_costmaps()
        self.get_logger().info(
            f'NavigateToGPS: ({lat:.6f}, {lon:.6f}, {alt:.1f})')
        self._start_navigation()

    def _handle_search(self, center_lat: float, center_lon: float, radius: float):
        """Handle SearchArea command."""
        self._search_center_lat = center_lat
        self._search_center_lon = center_lon
        self._search_radius = radius
        self._stage_state = StageState.WAIT_SEARCH
        self._retry_count = 0
        self._mission_start_time = time.time()
        self.get_logger().info(
            f'SearchArea: center=({center_lat:.6f}, {center_lon:.6f}), '
            f'radius={radius:.1f}m')
        self._generate_search_waypoints()
        # Search waypoint'lerini report'a kaydet
        self._report_waypoints = [
            WaypointRecord(idx=i, lat=wp[0], lon=wp[1])
            for i, wp in enumerate(self._search_waypoints)
        ]
        self._start_search()


    def _handle_start_exploration(self):
        """Handle StartExploration command."""
        self._stage_state = StageState.WAIT_EXPLORE
        self._exploration_executor.start(
            self._gps_lat, self._gps_lon, timestamp=time.time())
        self.get_logger().info('Exploration started — measuring distance')

    # ── Navigation ──────────────────────────────────────────────────────────

    def _start_navigation(self):
        """Send GPS goal to Nav2."""
        if not self._check_gps_ok():
            self.get_logger().warn('GPS not reliable — cannot navigate')
            return

        self._stage_state = StageState.NAVIGATING

        # Convert GPS to local ENU frame relative to datum
        if self._gps_origin is None:
            self.get_logger().error('No GPS datum — cannot navigate')
            self._stage_state = StageState.ERROR
            return

        x, y = gps_to_local(self._target_lat, self._target_lon, self._gps_origin)

        # Heading: face target from current position
        yaw = self._compute_heading_to(x, y)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self._map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 not available!')
            self._stage_state = StageState.ERROR
            return

        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_nav_goal_response)
        self.get_logger().info('Nav2 goal sent')

    def _on_nav_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Nav2 goal rejected')
            self._handle_nav_failure()
            return
        self._goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        result = future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('Nav2 goal reached')
            self._handle_arrival()
        else:
            self.get_logger().warn(f'Nav2 goal failed: status={result.status}')
            self._handle_nav_failure()

    def _handle_arrival(self):
        """Handle arrival at navigation target."""
        if self._current_stage in (1, 2):
            # Stage 1/2: after search, report GPS coordinate
            self._stage_state = StageState.FOUND
            self._report_gps_coordinate()
        elif self._current_stage == 3:
            if self._stage_state == StageState.NAVIGATING:
                self._stage_state = StageState.REACHED_ENTRY
                self._send_result('task_finished')
            elif self._stage_state == StageState.EXPLORING:
                self._stage_state = StageState.FOUND_EXIT
                self._report_distance()
        elif self._current_stage == 4:
            self._stage_state = StageState.REACHED
            self._send_result('task_finished')

    def _handle_nav_failure(self):
        """Handle navigation failure with retry."""
        self._retry_count += 1
        if self._retry_count <= self._max_retry:
            self.get_logger().info(
                f'Retrying navigation ({self._retry_count}/{self._max_retry})')
            # Retry after 1 second via one-shot timer (non-blocking)
            self._retry_timer = self.create_timer(1.0, self._retry_nav_once)
        else:
            self.get_logger().error('Navigation failed after max retries')
            self._send_result(f'navigation_failed:stage_{self._current_stage}')
            self._stage_state = StageState.ERROR

    def _retry_nav_once(self):
        """One-shot timer callback for navigation retry."""
        if hasattr(self, '_retry_timer') and self._retry_timer is not None:
            self._retry_timer.cancel()
            self._retry_timer = None
        self._start_navigation()

    def _cancel_current_goal(self):
        """Cancel current Nav2 goal."""
        if self._goal_handle:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._goal_handle = None

    def _compute_heading_to(self, target_x: float, target_y: float) -> float:
        """Compute yaw angle from current odom position to target in map frame.

        Returns:
            yaw in radians (quaternion-ready: orientation.z=sin(yaw/2), w=cos(yaw/2))
        """
        dx = target_x - self._odom_x
        dy = target_y - self._odom_y
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return 0.0  # Already at target, keep current heading
        return math.atan2(dy, dx)

    # ── Search ──────────────────────────────────────────────────────────────

    def _generate_search_waypoints(self):
        """Generate search waypoints using SearchExecutor."""
        self._search_waypoints = list(self._search_executor.generate(
            self._search_center_lat, self._search_center_lon,
            self._search_radius))

        self.get_logger().info(
            f'Generated {len(self._search_waypoints)} search waypoints '
            f'(pattern={self._search_pattern})')

    def _start_search(self):
        """Start search pattern navigation."""
        if not self._search_waypoints:
            self.get_logger().warn('No search waypoints')
            self._send_result('task_finished')
            return

        self._stage_state = StageState.SEARCHING
        self._current_wp_idx = 0
        self._send_search_waypoint()

    def _send_search_waypoint(self):
        """Send next search waypoint."""
        if self._current_wp_idx >= len(self._search_waypoints):
            self.get_logger().info('Search complete — no target found')
            self._send_result('task_finished')
            self._stage_state = StageState.DONE
            self._finalize_report('task_finished')
            return

        wp = self._search_waypoints[self._current_wp_idx]
        self.get_logger().info(
            f'Search waypoint {self._current_wp_idx + 1}/'
            f'{len(self._search_waypoints)}: ({wp[0]:.6f}, {wp[1]:.6f})')

        self._target_lat = wp[0]
        self._target_lon = wp[1]
        # Stuck pencereyi sıfırla
        self._stuck_check_pos = None
        self._stuck_timer_start = time.time()
        self._start_navigation()


    # ── Exploration ─────────────────────────────────────────────────────────

    def _update_exploration(self):
        """Update exploration distance from GPS — cumulative path length."""
        if not self._exploration_executor.is_running:
            return

        self._exploration_executor.update_gps(self._gps_lat, self._gps_lon)

        # Check timeout
        now = time.time()
        if self._exploration_executor.is_timeout(now):
            dist = self._exploration_executor.get_distance()
            self.get_logger().info(
                f'Exploration timeout — distance={dist:.1f}m')
            self._report_distance()
            self._send_result('task_finished')
            self._stage_state = StageState.DONE

    # ── Results ─────────────────────────────────────────────────────────────

    def _send_result(self, result: str):
        """Publish mission result for RSCP bridge."""
        msg = String()
        msg.data = result
        self._result_pub.publish(msg)
        self._report_result_messages.append(result)
        self.get_logger().info(f'Mission result: {result}')

    def _report_gps_coordinate(self):
        """Report found GPS coordinate."""
        result = f'gps_coordinate:{self._gps_lat:.8f},{self._gps_lon:.8f},{self._gps_alt:.2f}'
        self._send_result(result)
        self._send_result('task_finished')
        self._stage_state = StageState.DONE
        self._finalize_report('task_finished')

    def _report_distance(self):
        """Report exploration distance."""
        result = f'distance:{self._exploration_executor.get_distance():.2f}'
        self._send_result(result)
        self._stage_state = StageState.DONE
        self._finalize_report('task_finished')

    # ── RTK quality check ───────────────────────────────────────────────────

    def _check_gps_ok(self) -> bool:
        """Check if GPS quality is sufficient for navigation."""
        if self._last_gps_time == 0:
            return False
        age = time.time() - self._last_gps_time
        if age > 3.0:
            return False
        if self._rtk_status in ('NO_FIX', 'UNKNOWN'):
            if self._rtk_no_fix == 'stop':
                return False
        return True

    def _get_arrival_tolerance(self) -> float:
        """Get arrival tolerance based on RTK quality."""
        if self._rtk_status in ('RTK_FIXED', 'FIXED'):
            return self._rtk_fixed_tol
        elif self._rtk_status in ('RTK_FLOAT', 'FLOAT', 'DGPS'):
            return self._rtk_float_tol
        else:
            return self._arrival_tol

    # ── Stuck algılama ──────────────────────────────────────────────────────

    def _get_progress_pos(self):
        """Stuck için mevcut pozisyon — odom tercih, GPS fallback."""
        if self._stuck_source == 'gps':
            return (self._gps_lat, self._gps_lon)
        if self._last_odom_time > 0:
            return (self._odom_x, self._odom_y)
        return (self._gps_lat, self._gps_lon)

    def _check_stuck(self):
        """İlerleme eşiği altında kalırsa stuck → retry, dolunca skip."""
        if self._mission_start_time == 0:
            return
        now = time.time()
        pos = self._get_progress_pos()
        if self._stuck_check_pos is None:
            self._stuck_check_pos = pos
            self._stuck_timer_start = now
            return

        elapsed = now - self._stuck_timer_start
        if elapsed < self._stuck_window:
            return

        dx = pos[0] - self._stuck_check_pos[0]
        dy = pos[1] - self._stuck_check_pos[1]
        progress = math.hypot(dx, dy)

        if progress < self._stuck_min_progress:
            self.get_logger().warn(
                f'STUCK — progress={progress:.3f}m in {elapsed:.1f}s '
                f'< {self._stuck_min_progress}m')
            self._stuck_check_pos = None
            self._retry_count += 1
            if self._retry_count <= self._max_retry:
                self.get_logger().info(
                    f'Stuck retry ({self._retry_count}/{self._max_retry})')
                if self._stage_state == StageState.SEARCHING:
                    self._send_search_waypoint()
                else:
                    self._start_navigation()
            else:
                self.get_logger().info(
                    f'Stuck max retries exceeded — skipping waypoint')
                idx = self._current_wp_idx
                if idx < len(self._report_waypoints):
                    self._report_waypoints[idx].skipped = True
                    self._report_waypoints[idx].retries = self._retry_count
                self._send_result(f'skipped_stuck:wp_{idx}')
                self._retry_count = 0
                if self._stage_state == StageState.SEARCHING:
                    self._current_wp_idx += 1
                    self._send_search_waypoint()
                else:
                    self._handle_arrival()
        else:
            # İlerleme var — pencereyi kaydır
            self._stuck_check_pos = pos
            self._stuck_timer_start = now

    # ── GPS↔SLAM mode geçişi (arayüz + trigger) ──────────────────────────────

    def _update_loc_mode(self):
        """RTK kalitesine göre /localization/mode publish."""
        now = time.time()
        bad = self._rtk_status in ('NO_FIX', 'UNKNOWN', 'SPS')
        good = self._rtk_status in ('RTK_FIXED', 'FIXED', 'RTK_FLOAT', 'FLOAT', 'DGPS')

        if bad:
            if self._rtk_bad_since == 0:
                self._rtk_bad_since = now
            self._rtk_good_since = 0
            if (self._loc_mode == 'gps' and
                    now - self._rtk_bad_since > self._rtk_degrade_timeout):
                self._set_loc_mode('slam')
        elif good:
            if self._rtk_good_since == 0:
                self._rtk_good_since = now
            self._rtk_bad_since = 0
            if (self._loc_mode == 'slam' and
                    now - self._rtk_good_since > self._rtk_recover_timeout):
                self._set_loc_mode('gps')

    def _set_loc_mode(self, mode: str):
        if mode == self._loc_mode:
            return
        self._loc_mode = mode
        msg = String()
        msg.data = mode
        self._loc_mode_pub.publish(msg)
        self.get_logger().info(f'Localization mode → {mode}')

    # ── Mission report ─────────────────────────────────────────────────────

    def _start_new_mission_report(self):
        """Yeni mission için report state sıfırla + ID oluştur."""
        self._mission_counter += 1
        ts = self.get_clock().now().nanoseconds
        self._mission_id = f'stage{self._current_stage}_{ts}_{self._mission_counter}'
        self._report_path_samples = []
        self._report_waypoints = []
        self._report_result_messages = []
        self._report_written = False
        self._report_last_sample_time = 0.0

    def _record_waypoint(self, lat: float, lon: float):
        """NavigateToGPS için tek waypoint kaydı (search kendi listesini kurar)."""
        if self._gps_origin is not None:
            tx, ty = gps_to_local(lat, lon, self._gps_origin)
        else:
            tx, ty = 0.0, 0.0
        self._report_waypoints.append(
            WaypointRecord(idx=len(self._report_waypoints), lat=lat, lon=lon,
                           target_x=tx, target_y=ty))

    def _sample_report_path(self):
        """Report path örnekleme — report_sample_hz ile sınırlı."""
        if self._mission_start_time == 0 or self._mission_id == '':
            return
        now = time.time()
        if self._report_sample_hz <= 0:
            return
        dt = 1.0 / self._report_sample_hz
        if now - self._report_last_sample_time < dt:
            return
        self._report_last_sample_time = now
        t = now - self._mission_start_time
        self._report_path_samples.append(
            PathSample(t=t, x=self._odom_x, y=self._odom_y,
                       vx=self._odom_vx, vy=self._odom_vy,
                       rtk_status=self._rtk_status))

    def _finalize_report(self, end_reason: str):
        """Mission bitince JSON report yaz."""
        if self._report_written or self._mission_id == '':
            return
        self._report_written = True
        try:
            report = build_mission_report(
                mission_id=self._mission_id,
                stage=self._current_stage,
                start_time=self._mission_start_time,
                end_time=time.time(),
                end_reason=end_reason,
                waypoints=self._report_waypoints,
                path_samples=self._report_path_samples,
                result_messages=self._report_result_messages,
            )
            self._write_report(report)
            # CSV waypoint export
            self._export_waypoints_csv()
            self.get_logger().info(
                f'Mission report written: {self._mission_id} '
                f'({end_reason})')
        except Exception as e:
            self.get_logger().error(f'Report write failed: {e}')

    def _write_report(self, report: dict):
        os.makedirs(self._log_path, exist_ok=True)
        path = os.path.join(self._log_path, f'{self._mission_id}.json')
        with open(path, 'w') as f:
            json.dump(report, f, indent=2)

    def _export_waypoints_csv(self):
        if not self._report_waypoints:
            return
        os.makedirs(self._log_path, exist_ok=True)
        path = os.path.join(self._log_path, f'{self._mission_id}_waypoints.csv')
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['idx', 'lat', 'lon', 'reached', 'retries', 'skipped',
                        'elapsed_s'])
            for wp in self._report_waypoints:
                w.writerow([wp.idx, wp.lat, wp.lon, int(wp.reached),
                            wp.retries, int(wp.skipped), wp.elapsed_s])

    def _clear_costmaps(self):
        """Clear Nav2 local and global costmaps on phase transitions.

        Prevents stale obstacle data from previous phase affecting navigation.
        Transferred from Otonomius friend project (mission_manager.py).
        """
        for name, cli in [('local', self._clear_local_costmap),
                          ('global', self._clear_global_costmap)]:
            if cli.service_is_ready():
                cli.call_async(EmptySrv.Request())
                self.get_logger().info(f'Costmap cleared: {name}')
            else:
                self.get_logger().debug(
                    f'Costmap service not ready: {name}')

    # ── Preset / CSV import / Sweep ─────────────────────────────────────────

    def _handle_load_preset(self, preset_name: str):
        if not preset_name:
            self.get_logger().error('LOAD_PRESET — no preset name in payload')
            return
        # Presets dizini: param veya share altında default
        presets_dir = self._presets_dir
        if not presets_dir:
            # colcon share: install/earendil_navigation/share/earendil_navigation/config/mission_presets
            share = os.path.join(
                os.path.dirname(os.path.dirname(os.path.dirname(
                    os.path.abspath(__file__)))),
                'config', 'mission_presets')
            presets_dir = share
        path = os.path.join(presets_dir, f'{preset_name}.yaml')
        if not os.path.isfile(path):
            self.get_logger().error(f'Preset not found: {path}')
            return
        try:
            params = load_preset_yaml(path)
        except Exception as e:
            self.get_logger().error(f'Preset load failed: {e}')
            return
        for name, value in params.items():
            try:
                self.set_parameters(
                    [rclpy.parameter.Parameter(name, value=value)])
            except Exception as e:
                self.get_logger().warn(f'Preset param {name} skip: {e}')
        # Reload cached
        self._arrival_tol = self.get_parameter('arrival_tolerance_m').value
        self._rtk_fixed_tol = self.get_parameter('rtk_fixed_tolerance_m').value
        self._rtk_float_tol = self.get_parameter('rtk_float_tolerance_m').value
        self._search_speed = self.get_parameter('search_speed_mps').value
        self._wp_timeout = self.get_parameter('waypoint_timeout_s').value
        self._explore_timeout = self.get_parameter('exploration_timeout_s').value
        self._max_retry = self.get_parameter('max_retry_count').value
        self._stuck_window = self.get_parameter('stuck_window_s').value
        self._stuck_min_progress = self.get_parameter('stuck_min_progress_m').value
        self.get_logger().info(f'Preset loaded: {preset_name} ({len(params)} params)')

    def _handle_load_csv(self, csv_path: str):
        if not csv_path or not os.path.isfile(csv_path):
            self.get_logger().error(f'CSV not found: {csv_path}')
            return
        try:
            wps = import_waypoints_csv(csv_path)
        except Exception as e:
            self.get_logger().error(f'CSV import failed: {e}')
            return
        self._search_waypoints = [(wp[0], wp[1]) for wp in wps]
        self._report_waypoints = [
            WaypointRecord(idx=i, lat=wp[0], lon=wp[1], ) for i, wp in
            enumerate(self._search_waypoints)
        ]
        self.get_logger().info(
            f'CSV loaded: {len(wps)} waypoints from {csv_path}')

    def _handle_start_sweep(self, config_json: str):
        try:
            cfg = json.loads(config_json)
        except Exception as e:
            self.get_logger().error(f'Sweep config parse failed: {e}')
            return
        if 'param' not in cfg or 'values' not in cfg:
            self.get_logger().error('Sweep config needs "param" and "values"')
            return
        self._sweep_config = cfg
        self._sweep_idx = 0
        self._sweep_reports = []
        self.get_logger().info(
            f'Sweep armed: param={cfg["param"]}, '
            f'{len(cfg["values"])} values')
        # İlk run — sweep_pending_cmd RSCP komutla tetiklenir (M11 saha)

    def _mission_tick(self):
        """Periodic mission state update."""
        # Pause/cancel'da no-op
        if self._lifecycle != LifecycleState.RUNNING:
            return

        # Mode geçişi + report örnekleme her tick
        self._update_loc_mode()
        self._sample_report_path()

        if self._stage_state == StageState.SEARCHING:
            # Check if current nav goal reached, send next
            pass  # Handled by Nav2 callbacks
            self._check_stuck()

        elif self._stage_state == StageState.EXPLORING:
            self._update_exploration()
            self._check_stuck()

        elif self._stage_state == StageState.NAVIGATING:
            # Check timeout
            elapsed = time.time() - self._mission_start_time
            if elapsed > self._wp_timeout:
                self.get_logger().warn('Waypoint timeout')
                self._handle_nav_failure()
            self._check_stuck()

    # ── Status publisher ────────────────────────────────────────────────────

    def _publish_status(self):
        msg = MissionStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = int(self._stage_state)
        msg.mission_name = f'stage_{self._current_stage}'
        msg.total_waypoints = len(self._search_waypoints)
        msg.current_waypoint_index = self._current_wp_idx
        msg.succeeded_count = 0
        msg.failed_count = self._retry_count
        msg.lifecycle_state = int(self._lifecycle)
        msg.localization_mode = self._loc_mode
        if self._mission_start_time > 0:
            msg.elapsed_time_sec = time.time() - self._mission_start_time
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
