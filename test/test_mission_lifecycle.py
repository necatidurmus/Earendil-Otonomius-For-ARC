#!/usr/bin/env python3
"""
Test mission lifecycle logic (extracted without ROS).

Kapsar (MissionManagerLogic — ROS'suz extract):
  - LifecycleState geçişleri: pause/resume/cancel/reset/skip
  - RSCP override: CM komutu pause/cancel'i kırar
  - Stuck algılama: retry → max dolunca skip
  - GPS↔SLAM mode trigger (RTK degrade/recover timeout)
  - Skip: SEARCHING'de idx++

mission_manager.py'nin lifecycle katmanı rclpy'ye bağlı (Node). Burada
aynı state makinesini ROS'suz olarak yeniden üretiyoruz — tıpkı
test_safety_mux.py SafetyMuxLogic gibi. Mantık değiştikçe iki yer senkron
tutulmalı (TODO: mission_manager'da logic'i ayrı modüle çıkar).

Usage:
  python3 test/test_mission_lifecycle.py
"""

import sys
import os
import math
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'earendil_navigation'))

from enum import IntEnum


passed = 0
failed = 0


def check(desc, condition):
    global passed, failed
    if condition:
        passed += 1
        print(f'  ✓ {desc}')
    else:
        failed += 1
        print(f'  ✗ {desc}')


def approx(a, b, tol=1e-6):
    return abs(a - b) <= tol


# ── Enums (mission_manager.py ile senkron) ────────────────────────────────────

class LifecycleState(IntEnum):
    RUNNING = 0
    PAUSED = 1
    CANCELLED = 2


class StageState(IntEnum):
    IDLE = 0
    SEARCHING = 1
    NAVIGATING = 2
    EXPLORING = 3
    DONE = 99
    ERROR = 100


# RSCP command types
CMD_SET_STAGE = 0
CMD_ARM = 1
CMD_DISARM = 2
CMD_NAVIGATE_TO_GPS = 3
CMD_SEARCH_AREA = 4
CMD_START_EXPLORATION = 5


# ── Testable logic extract ────────────────────────────────────────────────────

class MissionManagerLogic:
    """Lifecycle + stuck + mode logic — ROS'suz, test edilebilir.

    mission_manager.MissionManagerNode ile aynı state geçişlerini uygular.
    ROS çağrıları (publish, action client) stub'lanır; yalnızca state
    makinesi davranışı test edilir.
    """

    def __init__(self, stuck_window=10.0, stuck_min_progress=0.2,
                 stuck_source='odom', max_retry=3,
                 rtk_degrade_timeout=5.0, rtk_recover_timeout=5.0):
        # Lifecycle
        self.lifecycle = LifecycleState.RUNNING
        self.paused_target = None

        # Stage state
        self.stage_state = StageState.IDLE
        self.current_stage = 0
        self.current_wp_idx = 0
        self.retry_count = 0
        self.max_retry = max_retry
        self.search_waypoints = []  # list of targets
        self.explore_distance = 0.0

        # Navigation target (lat, lon, alt)
        self.target_lat = 0.0
        self.target_lon = 0.0
        self.target_alt = 0.0

        # Stuck detection
        self.stuck_window = stuck_window
        self.stuck_min_progress = stuck_min_progress
        self.stuck_source = stuck_source
        self.stuck_check_pos = None
        self.stuck_timer_start = 0.0
        self.gps_lat = 0.0
        self.gps_lon = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.last_odom_time = 0.0
        self.mission_start_time = 0.0

        # GPS↔SLAM mode
        self.loc_mode = 'gps'
        self.rtk_status = 'NO_FIX'
        self.rtk_degrade_timeout = rtk_degrade_timeout
        self.rtk_recover_timeout = rtk_recover_timeout
        self.rtk_bad_since = 0.0
        self.rtk_good_since = 0.0

        # Report (minimal)
        self.report_waypoints = []  # list of dicts {skipped, retries}
        self.last_result = None
        self.loc_mode_published = []  # publish log

        # Action stubs
        self.nav_sent = []  # _start_navigation calls
        self.search_sent = []  # _send_search_waypoint calls
        self.goal_cancellations = 0
        self.cmd_vel_zeroed = 0
        self.arrivals = 0

    # ── stubs for ROS calls ──────────────────────────────────────────────────

    def _cancel_current_goal(self):
        self.goal_cancellations += 1

    def _publish_cmd_vel_zero(self):
        self.cmd_vel_zeroed += 1

    def _send_result(self, msg):
        self.last_result = msg

    def _start_navigation(self):
        self.nav_sent.append((self.target_lat, self.target_lon, self.target_alt))

    def _send_search_waypoint(self):
        if self.current_wp_idx < len(self.search_waypoints):
            wp = self.search_waypoints[self.current_wp_idx]
            self.search_sent.append(wp)

    def _handle_arrival(self):
        self.arrivals += 1

    def _set_loc_mode(self, mode):
        if mode == self.loc_mode:
            return
        self.loc_mode = mode
        self.loc_mode_published.append(mode)

    def _get_progress_pos(self):
        if self.stuck_source == 'gps':
            return (self.gps_lat, self.gps_lon)
        if self.last_odom_time > 0:
            return (self.odom_x, self.odom_y)
        return (self.gps_lat, self.gps_lon)

    # ── Lifecycle handlers ───────────────────────────────────────────────────

    def handle_pause(self):
        if self.lifecycle != LifecycleState.RUNNING:
            return False  # ignored
        self.lifecycle = LifecycleState.PAUSED
        self._cancel_current_goal()
        self._publish_cmd_vel_zero()
        self.paused_target = (self.target_lat, self.target_lon, self.target_alt)
        return True

    def handle_resume(self):
        if self.lifecycle != LifecycleState.PAUSED:
            return False
        self.lifecycle = LifecycleState.RUNNING
        self.retry_count = 0
        if self.paused_target is not None:
            lat, lon, alt = self.paused_target
            self.target_lat, self.target_lon, self.target_alt = (lat, lon, alt)
            if self.stage_state == StageState.SEARCHING:
                self._send_search_waypoint()
            else:
                self._start_navigation()
        return True

    def handle_cancel(self):
        self.lifecycle = LifecycleState.CANCELLED
        self._cancel_current_goal()
        self._publish_cmd_vel_zero()
        self._send_result('cancelled')
        return True

    def handle_reset(self):
        self._cancel_current_goal()
        self._publish_cmd_vel_zero()
        self.current_stage = 0
        self.stage_state = StageState.IDLE
        self.lifecycle = LifecycleState.RUNNING
        self.current_wp_idx = 0
        self.retry_count = 0
        self.explore_distance = 0.0
        self.search_waypoints = []
        self.report_waypoints = []
        self.paused_target = None
        self._set_loc_mode('gps')
        return True

    def handle_skip(self):
        if self.lifecycle != LifecycleState.RUNNING:
            return False
        self._cancel_current_goal()
        if self.stage_state == StageState.SEARCHING:
            idx = self.current_wp_idx
            if idx < len(self.report_waypoints):
                self.report_waypoints[idx]['skipped'] = True
            self.retry_count = 0
            self.current_wp_idx += 1
            self._send_search_waypoint()
        else:
            self._handle_arrival()
        return True

    # ── RSCP dispatch (override logic) ───────────────────────────────────────

    def dispatch_rscp(self, command_type, stage_value=None, lat=0.0, lon=0.0):
        """RSCP komutunu uygula — pause/cancel override."""
        override_cmds = (CMD_SET_STAGE, CMD_NAVIGATE_TO_GPS,
                         CMD_SEARCH_AREA, CMD_START_EXPLORATION)
        if self.lifecycle in (LifecycleState.PAUSED, LifecycleState.CANCELLED):
            if command_type in override_cmds:
                self.lifecycle = LifecycleState.RUNNING
                self._cancel_current_goal()
                self.last_result = None  # clear cancelled result
        # Apply command
        if command_type == CMD_SET_STAGE:
            self.current_stage = stage_value
            self.mission_start_time = time.time()
            self.report_waypoints = []
        elif command_type == CMD_NAVIGATE_TO_GPS:
            self.target_lat, self.target_lon = lat, lon
            self.stage_state = StageState.NAVIGATING
            # mission_manager._handle_navigate sets mission_start_time
            self.mission_start_time = time.time()
            self._start_navigation()
        elif command_type == CMD_SEARCH_AREA:
            self.stage_state = StageState.SEARCHING
            # mission_manager._handle_search sets mission_start_time
            self.mission_start_time = time.time()
            self.search_waypoints = [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
            self.report_waypoints = [
                {'skipped': False, 'retries': 0} for _ in self.search_waypoints]
            self.current_wp_idx = 0
            self._send_search_waypoint()
        elif command_type == CMD_START_EXPLORATION:
            self.stage_state = StageState.EXPLORING
            self.mission_start_time = time.time()
        elif command_type == CMD_ARM:
            pass
        elif command_type == CMD_DISARM:
            self.stage_state = StageState.IDLE
            self.lifecycle = LifecycleState.RUNNING

    # ── Stuck detection ──────────────────────────────────────────────────────

    def check_stuck(self, now):
        if self.mission_start_time == 0:
            return None
        pos = self._get_progress_pos()
        if self.stuck_check_pos is None:
            self.stuck_check_pos = pos
            self.stuck_timer_start = now
            return None
        elapsed = now - self.stuck_timer_start
        if elapsed < self.stuck_window:
            return None
        dx = pos[0] - self.stuck_check_pos[0]
        dy = pos[1] - self.stuck_check_pos[1]
        progress = math.hypot(dx, dy)
        if progress < self.stuck_min_progress:
            self.stuck_check_pos = None
            self.retry_count += 1
            if self.retry_count <= self.max_retry:
                if self.stage_state == StageState.SEARCHING:
                    self._send_search_waypoint()
                else:
                    self._start_navigation()
                return f'retry:{self.retry_count}'
            else:
                idx = self.current_wp_idx
                if idx < len(self.report_waypoints):
                    self.report_waypoints[idx]['skipped'] = True
                    self.report_waypoints[idx]['retries'] = self.retry_count
                self._send_result(f'skipped_stuck:wp_{idx}')
                self.retry_count = 0
                if self.stage_state == StageState.SEARCHING:
                    self.current_wp_idx += 1
                    self._send_search_waypoint()
                else:
                    self._handle_arrival()
                return 'skipped_stuck'
        else:
            self.stuck_check_pos = pos
            self.stuck_timer_start = now
            return None

    # ── GPS↔SLAM mode ────────────────────────────────────────────────────────

    def update_loc_mode(self, now):
        bad = self.rtk_status in ('NO_FIX', 'UNKNOWN', 'SPS')
        good = self.rtk_status in ('RTK_FIXED', 'FIXED', 'RTK_FLOAT', 'FLOAT', 'DGPS')
        if bad:
            if self.rtk_bad_since == 0:
                self.rtk_bad_since = now
            self.rtk_good_since = 0
            if self.loc_mode == 'gps' and now - self.rtk_bad_since > self.rtk_degrade_timeout:
                self._set_loc_mode('slam')
        elif good:
            if self.rtk_good_since == 0:
                self.rtk_good_since = now
            self.rtk_bad_since = 0
            if self.loc_mode == 'slam' and now - self.rtk_good_since > self.rtk_recover_timeout:
                self._set_loc_mode('gps')

    def set_odom(self, x, y, t):
        self.odom_x = x
        self.odom_y = y
        self.last_odom_time = t


# ── Tests: lifecycle transitions ──────────────────────────────────────────────

def test_pause_running_to_paused():
    print('\n[lifecycle] pause RUNNING→PAUSED')
    m = MissionManagerLogic()
    m.dispatch_rscp(CMD_NAVIGATE_TO_GPS, lat=39.0, lon=32.0)
    ok = m.handle_pause()
    check('pause accepted', ok is True)
    check('lifecycle PAUSED', m.lifecycle == LifecycleState.PAUSED)
    check('goal cancelled', m.goal_cancellations >= 1)
    check('cmd_vel zeroed', m.cmd_vel_zeroed >= 1)
    check('paused_target saved', m.paused_target == (39.0, 32.0, 0.0))


def test_pause_from_non_running_ignored():
    print('\n[lifecycle] pause from PAUSED/CANCELLED ignored')
    m = MissionManagerLogic()
    m.lifecycle = LifecycleState.PAUSED
    ok = m.handle_pause()
    check('pause ignored', ok is False)
    check('still PAUSED', m.lifecycle == LifecycleState.PAUSED)


def test_resume_paused_to_running():
    print('\n[lifecycle] resume PAUSED→RUNNING')
    m = MissionManagerLogic()
    m.dispatch_rscp(CMD_NAVIGATE_TO_GPS, lat=39.0, lon=32.0)
    m.handle_pause()
    ok = m.handle_resume()
    check('resume accepted', ok is True)
    check('lifecycle RUNNING', m.lifecycle == LifecycleState.RUNNING)
    check('retry_count reset', m.retry_count == 0)
    check('nav re-sent', len(m.nav_sent) == 2)  # initial + resume


def test_resume_from_running_ignored():
    print('\n[lifecycle] resume from RUNNING ignored')
    m = MissionManagerLogic()
    ok = m.handle_resume()
    check('resume ignored', ok is False)
    check('still RUNNING', m.lifecycle == LifecycleState.RUNNING)


def test_resume_search_resends_same_idx():
    print('\n[lifecycle] resume in SEARCHING resends same waypoint idx')
    m = MissionManagerLogic()
    m.dispatch_rscp(CMD_SEARCH_AREA, lat=39.0, lon=32.0)
    m.current_wp_idx = 1  # 2. waypointte
    m.search_sent.clear()
    m.handle_pause()
    m.handle_resume()
    check('search resent at idx 1', len(m.search_sent) == 1)
    check('idx unchanged on resume', m.current_wp_idx == 1)


def test_cancel():
    print('\n[lifecycle] cancel → CANCELLED + result + motor stop')
    m = MissionManagerLogic()
    m.dispatch_rscp(CMD_NAVIGATE_TO_GPS, lat=39.0, lon=32.0)
    m.handle_cancel()
    check('lifecycle CANCELLED', m.lifecycle == LifecycleState.CANCELLED)
    check('result cancelled', m.last_result == 'cancelled')
    check('goal cancelled', m.goal_cancellations >= 1)
    check('cmd_vel zeroed', m.cmd_vel_zeroed >= 1)


def test_reset():
    print('\n[lifecycle] reset clears all state')
    m = MissionManagerLogic()
    m.dispatch_rscp(CMD_SEARCH_AREA, lat=39.0, lon=32.0)
    m.current_wp_idx = 2
    m.retry_count = 1
    m.loc_mode = 'slam'
    m.handle_reset()
    check('lifecycle RUNNING', m.lifecycle == LifecycleState.RUNNING)
    check('stage IDLE', m.stage_state == StageState.IDLE)
    check('stage 0', m.current_stage == 0)
    check('wp_idx 0', m.current_wp_idx == 0)
    check('retry 0', m.retry_count == 0)
    check('report cleared', m.report_waypoints == [])
    check('loc_mode → gps', m.loc_mode == 'gps')
    check('paused_target cleared', m.paused_target is None)


def test_skip_searching_advances_idx():
    print('\n[lifecycle] skip in SEARCHING → idx++ + next waypoint')
    m = MissionManagerLogic()
    m.dispatch_rscp(CMD_SEARCH_AREA, lat=39.0, lon=32.0)
    m.search_sent.clear()
    m.handle_skip()
    check('idx advanced', m.current_wp_idx == 1)
    check('next search sent', len(m.search_sent) == 1)
    check('wp0 marked skipped', m.report_waypoints[0]['skipped'] is True)
    check('retry reset', m.retry_count == 0)


def test_skip_navigating_arrival():
    print('\n[lifecycle] skip in NAVIGATING → arrival')
    m = MissionManagerLogic()
    m.dispatch_rscp(CMD_NAVIGATE_TO_GPS, lat=39.0, lon=32.0)
    m.handle_skip()
    check('arrival called', m.arrivals == 1)


def test_skip_from_non_running_ignored():
    print('\n[lifecycle] skip from PAUSED ignored')
    m = MissionManagerLogic()
    m.lifecycle = LifecycleState.PAUSED
    ok = m.handle_skip()
    check('skip ignored', ok is False)


# ── Tests: RSCP override ──────────────────────────────────────────────────────

def test_rscp_override_breaks_pause():
    print('\n[override] RSCP SetStage breaks PAUSED')
    m = MissionManagerLogic()
    m.dispatch_rscp(CMD_NAVIGATE_TO_GPS, lat=39.0, lon=32.0)
    m.handle_pause()
    check('paused', m.lifecycle == LifecycleState.PAUSED)
    m.dispatch_rscp(CMD_SET_STAGE, stage_value=2)
    check('lifecycle RUNNING', m.lifecycle == LifecycleState.RUNNING)
    check('stage 2', m.current_stage == 2)
    check('cancelled result cleared', m.last_result is None)


def test_rscp_override_breaks_cancelled():
    print('\n[override] RSCP NavigateToGPS breaks CANCELLED')
    m = MissionManagerLogic()
    m.dispatch_rscp(CMD_NAVIGATE_TO_GPS, lat=39.0, lon=32.0)
    m.handle_cancel()
    check('cancelled', m.lifecycle == LifecycleState.CANCELLED)
    m.dispatch_rscp(CMD_NAVIGATE_TO_GPS, lat=40.0, lon=33.0)
    check('lifecycle RUNNING', m.lifecycle == LifecycleState.RUNNING)
    check('nav to new target', m.nav_sent[-1] == (40.0, 33.0, 0.0))


def test_rscp_search_area_breaks_pause():
    print('\n[override] RSCP SearchArea breaks PAUSED')
    m = MissionManagerLogic()
    m.dispatch_rscp(CMD_NAVIGATE_TO_GPS, lat=39.0, lon=32.0)
    m.handle_pause()
    m.dispatch_rscp(CMD_SEARCH_AREA, lat=39.0, lon=32.0)
    check('lifecycle RUNNING', m.lifecycle == LifecycleState.RUNNING)
    check('stage SEARCHING', m.stage_state == StageState.SEARCHING)


def test_disarm_resets_idle():
    print('\n[override] ArmDisarm(false) → IDLE + RUNNING')
    m = MissionManagerLogic()
    m.dispatch_rscp(CMD_NAVIGATE_TO_GPS, lat=39.0, lon=32.0)
    m.dispatch_rscp(CMD_DISARM)
    check('stage IDLE', m.stage_state == StageState.IDLE)
    check('lifecycle RUNNING', m.lifecycle == LifecycleState.RUNNING)


# ── Tests: stuck detection ────────────────────────────────────────────────────

def test_stuck_no_movement_retry():
    print('\n[stuck] no movement → retry (not skip)')
    m = MissionManagerLogic(stuck_window=2.0, stuck_min_progress=0.2, max_retry=3)
    m.dispatch_rscp(CMD_SEARCH_AREA, lat=39.0, lon=32.0)
    m.set_odom(0.0, 0.0, 1.0)
    r0 = m.check_stuck(now=1.0)
    check('first tick init', r0 is None)
    m.set_odom(0.0, 0.0, 3.0)
    r1 = m.check_stuck(now=3.0)
    check('stuck → retry:1', r1 == 'retry:1')
    check('retry_count 1', m.retry_count == 1)


def test_stuck_max_retries_then_skip():
    print('\n[stuck] max retries exceeded → skip')
    m = MissionManagerLogic(stuck_window=1.0, stuck_min_progress=0.5, max_retry=2)
    m.dispatch_rscp(CMD_SEARCH_AREA, lat=39.0, lon=32.0)
    m.current_wp_idx = 0
    # retry 1
    m.set_odom(0.0, 0.0, 1.0); m.check_stuck(now=1.0)  # init
    m.set_odom(0.0, 0.0, 2.0); m.check_stuck(now=2.0)  # retry:1
    check('retry 1', m.retry_count == 1)
    # retry 2
    m.set_odom(0.0, 0.0, 2.0); m.check_stuck(now=2.0)  # init
    m.set_odom(0.0, 0.0, 3.0); m.check_stuck(now=3.0)  # retry:2
    check('retry 2', m.retry_count == 2)
    # 3. stuck → max (2) aşıldı → skip
    m.set_odom(0.0, 0.0, 3.0); m.check_stuck(now=3.0)  # init
    m.set_odom(0.0, 0.0, 4.0)
    r = m.check_stuck(now=4.0)
    check('stuck → skipped_stuck', r == 'skipped_stuck')
    check('idx advanced', m.current_wp_idx == 1)
    check('result skipped_stuck:wp_0', m.last_result == 'skipped_stuck:wp_0')
    check('wp0 skipped', m.report_waypoints[0]['skipped'] is True)
    check('retry reset after skip', m.retry_count == 0)


def test_stuck_progress_resets_window():
    print('\n[stuck] progress above threshold → no retry, window slides')
    m = MissionManagerLogic(stuck_window=2.0, stuck_min_progress=0.2, max_retry=2)
    m.dispatch_rscp(CMD_SEARCH_AREA, lat=39.0, lon=32.0)
    m.set_odom(0.0, 0.0, 0.0)
    m.check_stuck(now=0.0)  # init
    m.set_odom(1.0, 0.0, 2.0)
    r = m.check_stuck(now=2.0)
    check('progress OK no retry', r is None)
    check('retry 0', m.retry_count == 0)
    check('window slid (check_pos updated)', approx(m.stuck_check_pos[0], 1.0))


def test_stuck_uses_gps_when_source_gps():
    print('\n[stuck] source=gps uses gps_lat/lon')
    m = MissionManagerLogic(stuck_window=2.0, stuck_min_progress=0.2,
                            stuck_source='gps', max_retry=2)
    m.dispatch_rscp(CMD_SEARCH_AREA, lat=39.0, lon=32.0)
    m.gps_lat, m.gps_lon = 39.0, 32.0
    m.check_stuck(now=1.0)  # init
    m.gps_lat, m.gps_lon = 39.0, 32.0  # no movement
    r = m.check_stuck(now=3.0)
    check('gps stuck → retry', r == 'retry:1')


def test_stuck_no_mission_start_noop():
    print('\n[stuck] no mission_start → no-op')
    m = MissionManagerLogic()
    r = m.check_stuck(now=1.0)
    check('no-op when mission not started', r is None)


# ── Tests: GPS↔SLAM mode ──────────────────────────────────────────────────────

def test_mode_gps_to_slam_on_rtk_degrade():
    print('\n[mode] GPS→SLAM on RTK degrade (NO_FIX > timeout)')
    m = MissionManagerLogic(rtk_degrade_timeout=5.0)
    m.rtk_status = 'NO_FIX'
    m.update_loc_mode(now=1.0)   # bad_since=1
    m.update_loc_mode(now=3.0)   # still < 5s
    check('still gps', m.loc_mode == 'gps')
    m.update_loc_mode(now=7.0)   # 7-1=6 > 5
    check('mode → slam', m.loc_mode == 'slam')
    check('published slam', m.loc_mode_published == ['slam'])


def test_mode_slam_to_gps_on_rtk_recover():
    print('\n[mode] SLAM→GPS on RTK recover (FIXED > timeout)')
    m = MissionManagerLogic(rtk_recover_timeout=5.0)
    m.loc_mode = 'slam'
    m.rtk_status = 'FIXED'
    m.update_loc_mode(now=1.0)   # good_since=1
    m.update_loc_mode(now=4.0)   # < 5s
    check('still slam', m.loc_mode == 'slam')
    m.update_loc_mode(now=7.0)   # 7-1=6 > 5
    check('mode → gps', m.loc_mode == 'gps')
    check('published gps', m.loc_mode_published == ['gps'])


def test_mode_bad_resets_good_timer():
    print('\n[mode] bad status resets good_since')
    m = MissionManagerLogic(rtk_recover_timeout=5.0)
    m.loc_mode = 'slam'
    m.rtk_status = 'FIXED'
    m.update_loc_mode(now=1.0)
    m.update_loc_mode(now=4.0)
    m.rtk_status = 'NO_FIX'
    m.update_loc_mode(now=5.0)
    check('good_since reset', m.rtk_good_since == 0.0)
    check('still slam', m.loc_mode == 'slam')


def test_mode_sps_is_bad():
    print('\n[mode] SPS treated as bad')
    m = MissionManagerLogic(rtk_degrade_timeout=2.0)
    m.rtk_status = 'SPS'
    m.update_loc_mode(now=1.0)
    m.update_loc_mode(now=4.0)  # 3s > 2s
    check('SPS → slam', m.loc_mode == 'slam')


def test_mode_dgps_is_good():
    print('\n[mode] DGPS treated as good (recovery)')
    m = MissionManagerLogic(rtk_recover_timeout=2.0)
    m.loc_mode = 'slam'
    m.rtk_status = 'DGPS'
    m.update_loc_mode(now=1.0)
    m.update_loc_mode(now=4.0)  # 3s > 2s
    check('DGPS → gps', m.loc_mode == 'gps')


def test_mode_reset_to_gps():
    print('\n[mode] reset forces gps')
    m = MissionManagerLogic()
    m.loc_mode = 'slam'
    m.handle_reset()
    check('reset → gps', m.loc_mode == 'gps')
    check('published gps', m.loc_mode_published == ['gps'])


# ── run ───────────────────────────────────────────────────────────────────────

def main():
    print('═══ Mission Lifecycle Tests ═══')
    test_pause_running_to_paused()
    test_pause_from_non_running_ignored()
    test_resume_paused_to_running()
    test_resume_from_running_ignored()
    test_resume_search_resends_same_idx()
    test_cancel()
    test_reset()
    test_skip_searching_advances_idx()
    test_skip_navigating_arrival()
    test_skip_from_non_running_ignored()
    test_rscp_override_breaks_pause()
    test_rscp_override_breaks_cancelled()
    test_rscp_search_area_breaks_pause()
    test_disarm_resets_idle()
    test_stuck_no_movement_retry()
    test_stuck_max_retries_then_skip()
    test_stuck_progress_resets_window()
    test_stuck_uses_gps_when_source_gps()
    test_stuck_no_mission_start_noop()
    test_mode_gps_to_slam_on_rtk_degrade()
    test_mode_slam_to_gps_on_rtk_recover()
    test_mode_bad_resets_good_timer()
    test_mode_sps_is_bad()
    test_mode_dgps_is_good()
    test_mode_reset_to_gps()
    print(f'\n═══ {passed} passed, {failed} failed ═══')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
