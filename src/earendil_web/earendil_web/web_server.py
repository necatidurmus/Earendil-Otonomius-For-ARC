#!/usr/bin/env python3
"""
Web Dashboard Node — Vehicle monitoring and control panel.

aiohttp + WebSocket for real-time updates. Vanilla JS + minimal CSS.
No heavy frontend frameworks.

Subscribes:
  /safety/status (SafetyStatus) — safety gate state
  /stm/status (StmStatus) — H723 link + mode
  /stm/fault_flags (StmFaultFlags) — F411 motor faults
  /stm/wheel_odom (Odometry) — wheel odometry
  /rscp/status (RscpStatus) — RSCP bridge health
  /rscp/current_stage (UInt32) — ARC stage 1-4
  /rscp/command (RscpCommand) — RSCP command log
  /gps/fix (NavSatFix) — RTK GPS
  /rtk/status (String) — RTK fix quality
  /mission/status (MissionStatus) — mission progress

Publishes:
  /cmd_vel_manual (Twist) — deadman teleop
  /deadman (Bool) — deadman heartbeat
  /e_stop (Bool) — e-stop engage/release (safety_mux input)

Safety: This node ONLY publishes to /cmd_vel_manual and /e_stop.
       NEVER to /cmd_vel_safe (CI audit enforced).
"""

import asyncio
import json
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String, UInt32
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

from earendil_interfaces.msg import (
    SafetyStatus, StmStatus, StmFaultFlags,
    RscpCommand, RscpStatus, MissionStatus,
)

try:
    from aiohttp import web
except ImportError:
    print('ERROR: aiohttp not installed. Run: pip3 install aiohttp')
    raise

import logging

logger = logging.getLogger(__name__)

CMD_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)
SENSOR_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)

CMD_NAMES = {
    0: 'SET_STAGE', 1: 'ARM', 2: 'DISARM',
    3: 'NAVIGATE_TO_GPS', 4: 'SEARCH_AREA', 5: 'START_EXPLORATION',
}
STAGE_NAMES = {1: 'Antenna', 2: 'Crater', 3: 'Lava Tube', 4: 'Airlock'}
MODE_NAMES = {0: 'DISARMED', 1: 'MANUAL', 2: 'AUTONOMOUS'}
CONN_NAMES = {0: 'DISCONNECTED', 1: 'CONNECTING', 2: 'CONNECTED', 3: 'ERROR'}
RTK_COLORS = {'RTK_FIXED': 'ok', 'RTK_FLOAT': 'warn', 'DGPS': 'warn',
              'SPS': 'err', 'NO_FIX': 'err', 'NONE': 'err'}


class WebDashboardNode(Node):
    """ROS 2 web dashboard — vehicle monitoring + deadman teleop + e-stop."""

    def __init__(self):
        super().__init__('web_dashboard')

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 0.5)
        self.declare_parameter('deadman_interval_ms', 500)
        self.declare_parameter('log_max_entries', 50)

        self._host = self.get_parameter('host').value
        self._port = self.get_parameter('port').value
        self._max_linear = self.get_parameter('max_linear_speed').value
        self._max_angular = self.get_parameter('max_angular_speed').value
        self._log_max = self.get_parameter('log_max_entries').value

        # ── State cache ─────────────────────────────────────────────
        self._safety = {}
        self._stm_status = {}
        self._stm_faults = {}
        self._rscp_status = {}
        self._rscp_stage = 0
        self._rtk_status = 'NONE'
        self._gps = {}
        self._odom = {}
        self._mission = {}
        self._fault_log = deque(maxlen=self._log_max)
        self._command_log = deque(maxlen=20)
        self._websockets = set()
        self._ws_lock = threading.Lock()

        self._deadman_held = False
        self._last_deadman_time = 0.0

        # ── ROS Publishers ──────────────────────────────────────────
        self._teleop_pub = self.create_publisher(Twist, '/cmd_vel_manual', CMD_QOS)
        self._deadman_pub = self.create_publisher(Bool, '/deadman', CMD_QOS)
        self._estop_pub = self.create_publisher(Bool, '/e_stop', CMD_QOS)

        # ── ROS Subscribers ─────────────────────────────────────────
        self.create_subscription(SafetyStatus, '/safety/status', self._on_safety, 10)
        self.create_subscription(StmStatus, '/stm/status', self._on_stm_status, SENSOR_QOS)
        self.create_subscription(StmFaultFlags, '/stm/fault_flags', self._on_stm_faults, SENSOR_QOS)
        self.create_subscription(RscpStatus, '/rscp/status', self._on_rscp_status, 10)
        self.create_subscription(UInt32, '/rscp/current_stage', self._on_rscp_stage, 10)
        self.create_subscription(RscpCommand, '/rscp/command', self._on_rscp_command, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self._on_gps, SENSOR_QOS)
        self.create_subscription(String, '/rtk/status', self._on_rtk_status, SENSOR_QOS)
        self.create_subscription(Odometry, '/stm/wheel_odom', self._on_odom, SENSOR_QOS)
        self.create_subscription(MissionStatus, '/mission/status', self._on_mission, 10)

        self.create_timer(0.1, self._check_deadman)

        self.get_logger().info(f'Web dashboard — http://{self._host}:{self._port}')

    # ── ROS callbacks ──────────────────────────────────────────────

    def _on_safety(self, msg: SafetyStatus):
        self._safety = {
            'type': 'safety',
            'estop_active': msg.estop_active,
            'watchdog_ok': msg.watchdog_ok,
            'watchdog_age_ms': msg.watchdog_age_ms,
            'deadman_held': msg.deadman_held,
            'deadman_age_ms': msg.deadman_age_ms,
            'gps_ok': msg.gps_ok, 'gps_age_ms': msg.gps_age_ms,
            'imu_ok': msg.imu_ok, 'imu_age_ms': msg.imu_age_ms,
            'lidar_ok': msg.lidar_ok, 'lidar_age_ms': msg.lidar_age_ms,
            'odom_ok': msg.odom_ok, 'odom_age_ms': msg.odom_age_ms,
            'linear_limit': msg.current_linear_limit,
            'angular_limit': msg.current_angular_limit,
        }
        self._broadcast(self._safety)

    def _on_stm_status(self, msg: StmStatus):
        self._stm_status = {
            'type': 'stm_status',
            'mode': MODE_NAMES.get(msg.operating_mode, 'UNKNOWN'),
            'mode_id': msg.operating_mode,
            'link_active': msg.link_active,
            'link_age_sec': round(msg.link_age_sec, 1),
            'commands_sent': msg.commands_sent,
            'acks_received': msg.acks_received,
            'timeouts': msg.timeouts,
        }
        self._broadcast(self._stm_status)

    def _on_stm_faults(self, msg: StmFaultFlags):
        motor_names = ['FL', 'FR', 'RL', 'RR']
        details = []
        for i in range(4):
            for flag_name in ['overcurrent', 'overtemperature', 'hall_error',
                              'undervoltage', 'overvoltage', 'driver_error', 'stall']:
                flags = getattr(msg, flag_name, [False] * 4)
                if i < len(flags) and flags[i]:
                    details.append(f'{motor_names[i]}:{flag_name}')
                    self._fault_log.append({
                        'time': time.strftime('%H:%M:%S'),
                        'motor': motor_names[i], 'fault': flag_name,
                    })
        self._stm_faults = {
            'type': 'stm_faults',
            'any_fault': msg.any_fault,
            'fault_count': msg.fault_count,
            'details': details,
        }
        self._broadcast(self._stm_faults)

    def _on_rscp_status(self, msg: RscpStatus):
        self._rscp_status = {
            'type': 'rscp_status',
            'connection': CONN_NAMES.get(msg.connection_state, 'UNKNOWN'),
            'connection_id': msg.connection_state,
            'port': msg.serial_port,
            'frames_rx': msg.frames_received,
            'frames_tx': msg.frames_sent,
            'parse_errors': msg.parse_errors,
            'cobs_errors': msg.cobs_errors,
            'last_cmd_age': round(msg.last_command_age_sec, 1),
        }
        self._broadcast(self._rscp_status)

    def _on_rscp_stage(self, msg: UInt32):
        self._rscp_stage = msg.data
        self._broadcast({
            'type': 'rscp_stage',
            'stage': msg.data,
            'stage_name': STAGE_NAMES.get(msg.data, f'Unknown({msg.data})'),
        })

    def _on_rscp_command(self, msg: RscpCommand):
        entry = {
            'time': time.strftime('%H:%M:%S'),
            'cmd': CMD_NAMES.get(msg.command_type, f'UNKNOWN({msg.command_type})'),
            'stage': msg.stage_value if msg.command_type == 0 else None,
            'lat': msg.latitude if msg.command_type in (3, 4) else None,
            'lon': msg.longitude if msg.command_type in (3, 4) else None,
        }
        self._command_log.append(entry)
        self._broadcast({'type': 'rscp_command', **entry})

    def _on_gps(self, msg: NavSatFix):
        self._gps = {
            'type': 'gps',
            'lat': msg.latitude, 'lon': msg.longitude, 'alt': msg.altitude,
        }
        self._broadcast(self._gps)

    def _on_rtk_status(self, msg: String):
        self._rtk_status = msg.data
        self._broadcast({
            'type': 'rtk',
            'quality': msg.data,
            'color': RTK_COLORS.get(msg.data, 'err'),
        })

    def _on_odom(self, msg: Odometry):
        self._odom = {
            'type': 'odom',
            'x': round(msg.pose.pose.position.x, 2),
            'y': round(msg.pose.pose.position.y, 2),
            'vx': round(msg.twist.twist.linear.x, 2),
            'vz': round(msg.twist.twist.angular.z, 2),
        }
        self._broadcast(self._odom)

    def _on_mission(self, msg: MissionStatus):
        states = ['IDLE', 'RUNNING', 'PAUSED', 'SUCCEEDED', 'FAILED', 'CANCELLED']
        self._mission = {
            'type': 'mission',
            'status': states[msg.status] if msg.status < len(states) else 'UNKNOWN',
            'name': msg.mission_name,
            'total_wp': msg.total_waypoints,
            'current_wp': msg.current_waypoint_index,
            'elapsed': round(msg.elapsed_time_sec, 1),
            'succeeded': msg.succeeded_count,
            'failed': msg.failed_count,
            'last_error': msg.last_error,
        }
        self._broadcast(self._mission)

    # ── Deadman ─────────────────────────────────────────────────────

    def _on_deadman_heartbeat(self):
        self._last_deadman_time = time.time()
        self._deadman_held = True
        self._deadman_pub.publish(Bool(data=True))

    def _on_deadman_release(self):
        self._deadman_held = False
        self._deadman_pub.publish(Bool(data=False))

    def _check_deadman(self):
        if not self._deadman_held:
            return
        if (time.time() - self._last_deadman_time) > 1.0:
            self._deadman_held = False
            self._deadman_pub.publish(Bool(data=False))
            self.get_logger().warn('Deadman timeout — releasing')

    # ── WebSocket ───────────────────────────────────────────────────

    def _broadcast(self, data: dict):
        msg = json.dumps(data)
        with self._ws_lock:
            dead = []
            for ws in self._websockets:
                try:
                    asyncio.run_coroutine_threadsafe(ws.send_str(msg), self._loop)
                except Exception:
                    dead.append(ws)
            for ws in dead:
                self._websockets.discard(ws)

    # ── Web handlers ────────────────────────────────────────────────

    async def _handle_index(self, request):
        return web.Response(text=self._get_index_html(), content_type='text/html')

    async def _handle_ws(self, request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        with self._ws_lock:
            self._websockets.add(ws)
        try:
            snapshot = {
                'type': 'snapshot',
                'safety': self._safety,
                'stm_status': self._stm_status,
                'stm_faults': self._stm_faults,
                'rscp_status': self._rscp_status,
                'rscp_stage': self._rscp_stage,
                'rtk_status': self._rtk_status,
                'gps': self._gps,
                'odom': self._odom,
                'mission': self._mission,
                'fault_log': list(self._fault_log),
                'command_log': list(self._command_log),
            }
            await ws.send_str(json.dumps(snapshot))

            async for msg in ws:
                if msg.type == web.WSMsgType.TEXT:
                    await self._handle_ws_command(msg.data)
                elif msg.type == web.WSMsgType.ERROR:
                    break
        finally:
            with self._ws_lock:
                self._websockets.discard(ws)
        return ws

    async def _handle_ws_command(self, data: str):
        try:
            cmd = json.loads(data)
        except json.JSONDecodeError:
            return

        action = cmd.get('action', '')

        if action == 'teleop':
            twist = Twist()
            lx = float(cmd.get('linear', 0.0))
            az = float(cmd.get('angular', 0.0))
            twist.linear.x = max(-self._max_linear, min(self._max_linear, lx))
            twist.angular.z = max(-self._max_angular, min(self._max_angular, az))
            self._teleop_pub.publish(twist)
            self._on_deadman_heartbeat()

        elif action == 'deadman':
            if cmd.get('held', False):
                self._on_deadman_heartbeat()
            else:
                self._on_deadman_release()

        elif action == 'estop':
            engage = bool(cmd.get('engage', True))
            self._estop_pub.publish(Bool(data=engage))
            self.get_logger().info(f'E-stop: {"ENGAGE" if engage else "RELEASE"}')

    # ── HTML Dashboard ──────────────────────────────────────────────

    def _get_index_html(self) -> str:
        return '''<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Earendil Rover Control</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:system-ui,-apple-system,sans-serif;background:#0d1117;color:#c9d1d9;padding:12px;max-width:1400px;margin:auto}
h1{text-align:center;color:#e94560;font-size:20px;margin-bottom:12px;letter-spacing:1px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:10px}
.card{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:14px}
.card h3{color:#e94560;font-size:12px;text-transform:uppercase;letter-spacing:1px;margin-bottom:10px;display:flex;align-items:center;gap:6px}
.card h3 .dot{width:8px;height:8px;border-radius:50%;display:inline-block}
.kv{display:flex;justify-content:space-between;padding:3px 0;font-size:13px;border-bottom:1px solid #21262d}
.kv:last-child{border:none}
.kv .label{color:#8b949e}
.kv .value{font-weight:600;font-family:monospace}
.ok{color:#3fb950}.warn{color:#d29922}.err{color:#f85149}
.btn{padding:8px 14px;border:1px solid #30363d;border-radius:6px;cursor:pointer;font-size:13px;font-weight:600;background:#21262d;color:#c9d1d9;transition:all .15s}
.btn:hover{background:#30363d}.btn:active{transform:scale(0.97)}
.btn-danger{background:#da3633;border-color:#f85149;color:#fff}
.btn-danger:hover{background:#f85149}
.btn-success{background:#238636;border-color:#3fb950;color:#fff}
.btn-success:hover{background:#3fb950}
.btn-sm{padding:5px 10px;font-size:11px}
.btn:disabled{opacity:0.4;cursor:not-allowed}
.bar-container{display:flex;align-items:center;gap:6px;font-size:12px;margin:2px 0}
.bar-label{width:22px;text-align:right;color:#8b949e;font-family:monospace}
.bar-bg{flex:1;height:14px;background:#21262d;border-radius:3px;overflow:hidden}
.bar-fill{height:100%;border-radius:3px;transition:width .3s}
.bar-val{width:50px;text-align:right;font-family:monospace;font-size:11px}
.joystick-wrap{display:flex;flex-direction:column;align-items:center;gap:10px;padding:10px}
#joystick{touch-action:none;border:2px solid #30363d;border-radius:50%;background:radial-gradient(circle,#161b22 60%,#21262d)}
.deadman-btn{width:100%;height:56px;font-size:16px;border-radius:10px;letter-spacing:1px}
.speed-display{font-family:monospace;font-size:14px;color:#58a6ff}
.log-pane{height:100px;overflow-y:auto;font-size:11px;font-family:monospace;background:#0d1117;padding:6px;border-radius:4px;border:1px solid #21262d}
.log-entry{border-bottom:1px solid #161b22;padding:1px 0}
.log-time{color:#8b949e;margin-right:6px}
</style>
</head>
<body>
<h1>EARENDIL ROVER CONTROL PANEL</h1>
<div class="grid">

  <div class="card">
    <h3><span class="dot" id="sa-dot"></span> Safety</h3>
    <div class="kv"><span class="label">E-Stop</span><span class="value" id="sa-estop">OFF</span></div>
    <div class="kv"><span class="label">Watchdog</span><span class="value" id="sa-watchdog">--</span></div>
    <div class="kv"><span class="label">Deadman</span><span class="value" id="sa-deadman">--</span></div>
    <div class="kv"><span class="label">GPS</span><span class="value" id="sa-gps">--</span></div>
    <div class="kv"><span class="label">IMU</span><span class="value" id="sa-imu">--</span></div>
    <div class="kv"><span class="label">LiDAR</span><span class="value" id="sa-lidar">--</span></div>
    <div class="kv"><span class="label">Odom</span><span class="value" id="sa-odom">--</span></div>
    <div class="kv"><span class="label">Speed Limit</span><span class="value" id="sa-speed">--</span></div>
    <div style="margin-top:8px;display:flex;gap:8px">
      <button class="btn btn-danger" onclick="estop(true)">E-STOP</button>
      <button class="btn btn-success btn-sm" onclick="estop(false)">Release</button>
    </div>
  </div>

  <div class="card">
    <h3><span class="dot" id="rscp-dot"></span> RSCP Bridge</h3>
    <div class="kv"><span class="label">Stage</span><span class="value" id="rscp-stage">--</span></div>
    <div class="kv"><span class="label">Connection</span><span class="value" id="rscp-conn">--</span></div>
    <div class="kv"><span class="label">Frames RX/TX</span><span class="value" id="rscp-frames">0 / 0</span></div>
    <div class="kv"><span class="label">Parse Errors</span><span class="value" id="rscp-errors">0</span></div>
    <div class="kv"><span class="label">Last Cmd</span><span class="value" id="rscp-last-cmd">--</span></div>
    <div class="kv"><span class="label">RTK</span><span class="value" id="rscp-rtk">--</span></div>
  </div>

  <div class="card">
    <h3><span class="dot" id="stm-dot"></span> STM (H723)</h3>
    <div class="kv"><span class="label">Mode</span><span class="value" id="stm-mode">--</span></div>
    <div class="kv"><span class="label">Link</span><span class="value" id="stm-link">--</span></div>
    <div class="kv"><span class="label">CMD / ACK / TMO</span><span class="value" id="stm-stats">0/0/0</span></div>
    <div style="margin-top:8px">
      <div class="bar-container"><span class="bar-label">FL</span><div class="bar-bg"><div class="bar-fill" id="rpm-fl" style="width:0%;background:#3fb950"></div></div><span class="bar-val" id="rpm-fl-v">0</span></div>
      <div class="bar-container"><span class="bar-label">FR</span><div class="bar-bg"><div class="bar-fill" id="rpm-fr" style="width:0%;background:#388bfd"></div></div><span class="bar-val" id="rpm-fr-v">0</span></div>
      <div class="bar-container"><span class="bar-label">RL</span><div class="bar-bg"><div class="bar-fill" id="rpm-rl" style="width:0%;background:#3fb950"></div></div><span class="bar-val" id="rpm-rl-v">0</span></div>
      <div class="bar-container"><span class="bar-label">RR</span><div class="bar-bg"><div class="bar-fill" id="rpm-rr" style="width:0%;background:#388bfd"></div></div><span class="bar-val" id="rpm-rr-v">0</span></div>
    </div>
  </div>

  <div class="card">
    <h3><span class="dot" id="mi-dot"></span> Mission</h3>
    <div class="kv"><span class="label">Status</span><span class="value" id="mi-status">IDLE</span></div>
    <div class="kv"><span class="label">Mission</span><span class="value" id="mi-name">--</span></div>
    <div class="kv"><span class="label">Waypoint</span><span class="value" id="mi-wp">0/0</span></div>
    <div class="kv"><span class="label">Time</span><span class="value" id="mi-time">0.0s</span></div>
    <div class="kv"><span class="label">OK / Fail</span><span class="value" id="mi-results">0/0</span></div>
    <div class="kv"><span class="label">Error</span><span class="value" id="mi-error">--</span></div>
  </div>

  <div class="card">
    <h3>GPS & Position</h3>
    <div class="kv"><span class="label">Latitude</span><span class="value" id="gps-lat">--</span></div>
    <div class="kv"><span class="label">Longitude</span><span class="value" id="gps-lon">--</span></div>
    <div class="kv"><span class="label">Altitude</span><span class="value" id="gps-alt">--</span></div>
    <div class="kv"><span class="label">X (odom)</span><span class="value" id="pos-x">0.00</span></div>
    <div class="kv"><span class="label">Y (odom)</span><span class="value" id="pos-y">0.00</span></div>
    <div class="kv"><span class="label">Speed</span><span class="value" id="pos-vx">0.00 m/s</span></div>
    <div class="kv"><span class="label">Turn</span><span class="value" id="pos-vz">0.00 rad/s</span></div>
  </div>

  <div class="card">
    <h3>Manual Control</h3>
    <div class="joystick-wrap">
      <canvas id="joystick" width="180" height="180"></canvas>
      <div class="speed-display">
        <span id="joy-lin">0.00</span> m/s &nbsp;|&nbsp;
        <span id="joy-ang">0.00</span> rad/s
      </div>
      <button id="deadman-btn" class="btn btn-danger deadman-btn"
        onmousedown="deadman(true)" onmouseup="deadman(false)"
        onmouseleave="deadman(false)"
        ontouchstart="deadman(true)" ontouchend="deadman(false)">
        HOLD FOR DEADMAN
      </button>
    </div>
  </div>

  <div class="card" style="grid-column:span 2">
    <h3>Log</h3>
    <div style="display:flex;gap:10px">
      <div style="flex:1">
        <div style="font-size:11px;color:#8b949e;margin-bottom:4px">RSCP Commands</div>
        <div id="cmd-log" class="log-pane"></div>
      </div>
      <div style="flex:1">
        <div style="font-size:11px;color:#8b949e;margin-bottom:4px">Fault Log</div>
        <div id="fault-log" class="log-pane"></div>
      </div>
    </div>
  </div>

</div>

<script>
const ws = new WebSocket(`ws://${location.host}/ws`);
ws.onopen = () => addCmdLog('SYS','Connected');
ws.onclose = () => addCmdLog('SYS','Disconnected');

ws.onmessage = (e) => {
  const d = JSON.parse(e.data), t = d.type;
  if (t==='snapshot') {
    if (d.safety?.type) updSafety(d.safety);
    if (d.stm_status?.type) updStm(d.stm_status);
    if (d.rscp_status?.type) updRscp(d.rscp_status);
    if (d.rscp_stage) updStage({stage:d.rscp_stage,stage_name:stageName(d.rscp_stage)});
    if (d.rtk_status) updRtk({quality:d.rtk_status,color:rtkCol(d.rtk_status)});
    if (d.gps?.type) updGps(d.gps);
    if (d.odom?.type) updOdom(d.odom);
    if (d.mission?.type) updMission(d.mission);
    (d.fault_log||[]).forEach(f=>addFaultLog(f));
    (d.command_log||[]).forEach(c=>addCmdLog(c.cmd,''));
  } else if (t==='safety') updSafety(d);
  else if (t==='stm_status') updStm(d);
  else if (t==='stm_faults') d.details.forEach(f=>addFaultLog({time:ts(),motor:f.split(':')[0],fault:f.split(':')[1]}));
  else if (t==='rscp_status') updRscp(d);
  else if (t==='rscp_stage') updStage(d);
  else if (t==='rscp_command') addCmdLog(d.cmd,'stage='+(d.stage||'-'));
  else if (t==='rtk') updRtk(d);
  else if (t==='gps') updGps(d);
  else if (t==='odom') updOdom(d);
  else if (t==='mission') updMission(d);
};

function ts(){return new Date().toLocaleTimeString('en-GB',{hour12:false}).slice(0,8)}
function stageName(s){return{1:'Antenna Installation',2:'Shackleton Crater',3:'Lava Tube',4:'Return to Airlock'}[s]||'Stage '+s}
function rtkCol(q){return{'RTK_FIXED':'ok','RTK_FLOAT':'warn','DGPS':'warn'}[q]||'err'}
function cls(ok,inv){return inv?(ok?'err':'ok'):(ok?'ok':'err')}
function sT(id,v){document.getElementById(id).textContent=v}
function sC(id,ok,inv){document.getElementById(id).className='value '+cls(ok,inv)}

function updSafety(d){
  document.getElementById('sa-dot').style.background=d.estop_active?'#f85149':'#3fb950';
  sC('sa-estop',d.estop_active,true);sT('sa-estop',d.estop_active?'ACTIVE':'OFF');
  sC('sa-watchdog',d.watchdog_ok);sT('sa-watchdog',d.watchdog_ok?'OK ('+Math.round(d.watchdog_age_ms)+'ms)':'FAIL');
  sC('sa-deadman',d.deadman_held);sT('sa-deadman',d.deadman_held?'HELD':'RELEASED');
  sC('sa-gps',d.gps_ok);sT('sa-gps',d.gps_ok?'OK':'LOST');
  sC('sa-imu',d.imu_ok);sT('sa-imu',d.imu_ok?'OK':'LOST');
  sC('sa-lidar',d.lidar_ok);sT('sa-lidar',d.lidar_ok?'OK':'LOST');
  sC('sa-odom',d.odom_ok);sT('sa-odom',d.odom_ok?'OK':'LOST');
  sT('sa-speed',d.linear_limit.toFixed(1)+' m/s, '+d.angular_limit.toFixed(1)+' rad/s');
}
function updStm(d){
  document.getElementById('stm-dot').style.background=d.link_active?'#3fb950':'#f85149';
  sC('stm-mode',d.mode_id===2);sT('stm-mode',d.mode);
  sC('stm-link',d.link_active);sT('stm-link',d.link_active?'ACTIVE ('+d.link_age_sec+'s)':'LOST');
  sT('stm-stats',d.commands_sent+'/'+d.acks_received+'/'+d.timeouts);
}
function updRscp(d){
  document.getElementById('rscp-dot').style.background=d.connection_id===2?'#3fb950':'#f85149';
  sC('rscp-conn',d.connection_id===2);sT('rscp-conn',d.connection);
  sT('rscp-frames',d.frames_rx+' / '+d.frames_tx);
  sC('rscp-errors',d.parse_errors===0&&d.cobs_errors===0);sT('rscp-errors','P:'+d.parse_errors+' C:'+d.cobs_errors);
  sT('rscp-last-cmd',d.last_cmd_age>=0?d.last_cmd_age+'s ago':'--');
}
function updStage(d){var el=document.getElementById('rscp-stage');el.textContent='Stage '+d.stage+': '+d.stage_name;el.className='value ok'}
function updRtk(d){var el=document.getElementById('rscp-rtk');el.textContent=d.quality;el.className='value '+(d.color||'ok')}
function updGps(d){sT('gps-lat',d.lat.toFixed(7));sT('gps-lon',d.lon.toFixed(7));sT('gps-alt',d.alt.toFixed(2)+' m')}
function updOdom(d){sT('pos-x',d.x.toFixed(2)+' m');sT('pos-y',d.y.toFixed(2)+' m');sT('pos-vx',d.vx.toFixed(2)+' m/s');sT('pos-vz',d.vz.toFixed(2)+' rad/s')}
function updMission(d){
  document.getElementById('mi-dot').style.background=d.status==='RUNNING'?'#388bfd':d.status==='SUCCEEDED'?'#3fb950':d.status==='FAILED'?'#f85149':'#8b949e';
  sT('mi-status',d.status);sT('mi-name',d.name||'--');sT('mi-wp',d.current_wp+'/'+d.total_wp);
  sT('mi-time',d.elapsed+'s');sT('mi-results',d.succeeded+'/'+d.failed);sT('mi-error',d.last_error||'--');
}
function addCmdLog(cmd,detail){var el=document.getElementById('cmd-log');el.innerHTML='<div class="log-entry"><span class="log-time">'+ts()+'</span><span class="ok">'+cmd+'</span> '+detail+'</div>'+el.innerHTML;while(el.children.length>50)el.removeChild(el.lastChild)}
function addFaultLog(f){var el=document.getElementById('fault-log');el.innerHTML='<div class="log-entry"><span class="log-time">'+f.time+'</span><span class="err">'+f.motor+': '+f.fault+'</span></div>'+el.innerHTML;while(el.children.length>50)el.removeChild(el.lastChild)}

function estop(engage){ws.send(JSON.stringify({action:'estop',engage:engage}))}
function deadman(held){ws.send(JSON.stringify({action:'deadman',held:held}));var b=document.getElementById('deadman-btn');b.style.background=held?'#238636':'#da3633';b.textContent=held?'DEADMAN ACTIVE':'HOLD FOR DEADMAN'}
function teleop(l,a){ws.send(JSON.stringify({action:'teleop',linear:l,angular:a}))}

// Virtual Joystick
(function(){
  var c=document.getElementById('joystick'),x=c.getContext('2d'),R=c.width/2,SR=30,drag=false,cx=R,cy=R;
  function draw(){x.clearRect(0,0,c.width,c.height);x.beginPath();x.arc(R,R,R-4,0,Math.PI*2);x.strokeStyle='#30363d';x.lineWidth=2;x.stroke();x.beginPath();x.moveTo(R,10);x.lineTo(R,c.height-10);x.moveTo(10,R);x.lineTo(c.width-10,R);x.strokeStyle='#21262d';x.lineWidth=1;x.stroke();x.beginPath();x.arc(cx,cy,SR,0,Math.PI*2);x.fillStyle=drag?'#388bfd':'#30363d';x.fill();x.strokeStyle='#58a6ff';x.lineWidth=2;x.stroke()}
  function upd(px,py){var dx=px-R,dy=py-R,d=Math.sqrt(dx*dx+dy*dy),mx=R-SR,cd=Math.min(d,mx),a=Math.atan2(dy,dx);cx=R+cd*Math.cos(a);cy=R+cd*Math.sin(a);var nx=cd/mx*Math.cos(a),ny=cd/mx*Math.sin(a),lin=-ny*0.3,ang=nx*0.5;document.getElementById('joy-lin').textContent=lin.toFixed(2);document.getElementById('joy-ang').textContent=ang.toFixed(2);if(drag)teleop(lin,ang);draw()}
  function pos(e){var r=c.getBoundingClientRect(),t=e.touches?e.touches[0]:e;return{x:t.clientX-r.left,y:t.clientY-r.top}}
  c.addEventListener('mousedown',function(e){drag=true;var p=pos(e);upd(p.x,p.y)});
  c.addEventListener('mousemove',function(e){if(drag){var p=pos(e);upd(p.x,p.y)}});
  c.addEventListener('mouseup',function(){drag=false;cx=R;cy=R;teleop(0,0);draw()});
  c.addEventListener('mouseleave',function(){if(drag){drag=false;cx=R;cy=R;teleop(0,0);draw()}});
  c.addEventListener('touchstart',function(e){e.preventDefault();drag=true;var p=pos(e);upd(p.x,p.y)});
  c.addEventListener('touchmove',function(e){e.preventDefault();if(drag){var p=pos(e);upd(p.x,p.y)}});
  c.addEventListener('touchend',function(e){e.preventDefault();drag=false;cx=R;cy=R;teleop(0,0);draw()});
  draw();
})();
</script>
</body>
</html>'''

    # ── Server lifecycle ────────────────────────────────────────────

    def start_web_server(self):
        self._loop = asyncio.new_event_loop()

        async def run():
            app = web.Application()
            app.router.add_get('/', self._handle_index)
            app.router.add_get('/ws', self._handle_ws)
            runner = web.AppRunner(app)
            await runner.setup()
            site = web.TCPSite(runner, self._host, self._port)
            await site.start()
            self.get_logger().info(
                f'Web server running on http://{self._host}:{self._port}')
            while rclpy.ok():
                await asyncio.sleep(0.1)

        self._loop.run_until_complete(run())


def main(args=None):
    rclpy.init(args=args)
    node = WebDashboardNode()

    web_thread = threading.Thread(target=node.start_web_server, daemon=True)
    web_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
