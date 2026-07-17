#!/usr/bin/env python3
"""
Test ARC stage state machine (extracted without ROS).

Kapsar (StageStateMachineLogic — ROS'suz extract):
  - SetStage validasyonu (1-4) → WAIT_STAGE; invalid → error
  - Arm/Disarm → disarm IDLE
  - NavigateToGPS → WAIT_NAV → NAVIGATING
  - SearchArea → WAIT_SEARCH → SEARCHING
  - StartExploration → WAIT_EXPLORE → EXPLORING
  - Arrival: stage-aware (1/2→FOUND, 3 nav→REACHED_ENTRY+task_finished,
    3 explore→FOUND_EXIT, 4→REACHED+task_finished)
  - Nav failure retry → max dolunca ERROR

mission_manager.py stage state machine rclpy/Nav2'ye bağlı. Burada aynı
geçişleri ROS'suz üretiyoruz — tıpkı test_safety_mux.py gibi.

Usage:
  python3 test/test_stage_state_machine.py
"""

import sys
import os

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


# ── Enums (mission_manager.py ile senkron) ────────────────────────────────────

class StageState(IntEnum):
    IDLE = 0
    WAIT_STAGE = 10
    WAIT_SEARCH = 11
    SEARCHING = 12
    FOUND = 13
    WAIT_NAV = 20
    NAVIGATING = 21
    REACHED_ENTRY = 22
    WAIT_EXPLORE = 23
    EXPLORING = 24
    FOUND_EXIT = 25
    DOCKING = 30
    REACHED = 31
    WAIT_DISARM = 32
    DONE = 99
    ERROR = 100


# ── Stage state machine logic extract ─────────────────────────────────────────

class StageStateMachineLogic:
    """ARC stage state machine — ROS'suz, test edilebilir.

    mission_manager.MissionManagerNode ile aynı stage geçişlerini uygular.
    Nav2/ROS çağrıları stub'lanır; yalnızca state geçişleri test edilir.
    """

    def __init__(self, max_retry=3):
        self.current_stage = 0
        self.stage_state = StageState.IDLE
        self.retry_count = 0
        self.max_retry = max_retry
        self.armed = False

        self.target_lat = 0.0
        self.target_lon = 0.0
        self.target_alt = 0.0

        self.search_waypoints = []
        self.explore_distance = 0.0
        self.explore_start_pos = None

        # Stubs
        self.results = []
        self.nav_sent_count = 0
        self.gps_ok = True
        self.has_datum = True

    def _send_result(self, msg):
        self.results.append(msg)

    def _start_navigation(self):
        if not self.gps_ok:
            return
        if not self.has_datum:
            self.stage_state = StageState.ERROR
            return
        self.stage_state = StageState.NAVIGATING
        self.nav_sent_count += 1

    def _start_search(self):
        self.stage_state = StageState.SEARCHING

    def _report_gps_coordinate(self):
        self.stage_state = StageState.FOUND

    def _report_distance(self):
        self.stage_state = StageState.FOUND_EXIT

    def handle_set_stage(self, stage):
        if stage < 1 or stage > 4:
            self._send_result('message:invalid_stage')
            return False
        self.current_stage = stage
        self.stage_state = StageState.WAIT_STAGE
        self.retry_count = 0
        return True

    def handle_arm(self, arm):
        self.armed = arm
        if not arm:
            self.stage_state = StageState.IDLE
        return True

    def handle_navigate(self, lat, lon, alt):
        self.target_lat = lat
        self.target_lon = lon
        self.target_alt = alt
        self.stage_state = StageState.WAIT_NAV
        self.retry_count = 0
        self._start_navigation()

    def handle_search(self, center_lat, center_lon, radius):
        self.stage_state = StageState.WAIT_SEARCH
        self.retry_count = 0
        self._generate_search_waypoints()
        self._start_search()

    def _generate_search_waypoints(self):
        self.search_waypoints = [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]

    def handle_start_exploration(self):
        self.stage_state = StageState.WAIT_EXPLORE
        self.explore_distance = 0.0
        self.explore_start_pos = (0.0, 0.0)
        self.stage_state = StageState.EXPLORING

    def handle_arrival(self):
        if self.current_stage in (1, 2):
            self.stage_state = StageState.FOUND
            self._report_gps_coordinate()
        elif self.current_stage == 3:
            if self.stage_state == StageState.NAVIGATING:
                self.stage_state = StageState.REACHED_ENTRY
                self._send_result('task_finished')
            elif self.stage_state == StageState.EXPLORING:
                self.stage_state = StageState.FOUND_EXIT
                self._report_distance()
        elif self.current_stage == 4:
            self.stage_state = StageState.REACHED
            self._send_result('task_finished')

    def handle_nav_failure(self):
        self.retry_count += 1
        if self.retry_count <= self.max_retry:
            self._start_navigation()
        else:
            self._send_result(f'navigation_failed:stage_{self.current_stage}')
            self.stage_state = StageState.ERROR


# ── Tests: SetStage ───────────────────────────────────────────────────────────

def test_set_stage_valid():
    print('\n[stage] SetStage valid 1-4')
    for s in (1, 2, 3, 4):
        m = StageStateMachineLogic()
        ok = m.handle_set_stage(s)
        check(f'stage {s} accepted', ok is True)
        check(f'stage {s} → WAIT_STAGE', m.stage_state == StageState.WAIT_STAGE)
        check(f'stage {s} current_stage set', m.current_stage == s)


def test_set_stage_invalid():
    print('\n[stage] SetStage invalid 0,5,-1')
    for s in (0, 5, -1):
        m = StageStateMachineLogic()
        ok = m.handle_set_stage(s)
        check(f'stage {s} rejected', ok is False)
        check(f'stage {s} → invalid_stage result',
              m.results == ['message:invalid_stage'])
        check(f'stage {s} stays IDLE', m.stage_state == StageState.IDLE)


def test_set_stage_resets_retry():
    print('\n[stage] SetStage resets retry_count')
    m = StageStateMachineLogic()
    m.retry_count = 5
    m.handle_set_stage(2)
    check('retry reset', m.retry_count == 0)


# ── Tests: Arm/Disarm ─────────────────────────────────────────────────────────

def test_arm():
    print('\n[arm] arm true')
    m = StageStateMachineLogic()
    m.handle_arm(True)
    check('armed', m.armed is True)


def test_disarm_idle():
    print('\n[arm] disarm → IDLE')
    m = StageStateMachineLogic()
    m.handle_set_stage(2)
    m.handle_navigate(39.0, 32.0, 0.0)
    check('navigating before disarm', m.stage_state == StageState.NAVIGATING)
    m.handle_arm(False)
    check('disarmed', m.armed is False)
    check('stage IDLE after disarm', m.stage_state == StageState.IDLE)


# ── Tests: NavigateToGPS ──────────────────────────────────────────────────────

def test_navigate_to_gps():
    print('\n[nav] NavigateToGPS → WAIT_NAV → NAVIGATING')
    m = StageStateMachineLogic()
    m.handle_set_stage(3)
    m.handle_navigate(39.0, 32.0, 100.0)
    check('target lat', m.target_lat == 39.0)
    check('target lon', m.target_lon == 32.0)
    check('target alt', m.target_alt == 100.0)
    check('nav sent', m.nav_sent_count == 1)
    check('state NAVIGATING', m.stage_state == StageState.NAVIGATING)
    check('retry reset', m.retry_count == 0)


def test_navigate_no_datum_error():
    print('\n[nav] no datum → ERROR')
    m = StageStateMachineLogic()
    m.has_datum = False
    m.handle_set_stage(3)
    m.handle_navigate(39.0, 32.0, 0.0)
    check('state ERROR', m.stage_state == StageState.ERROR)


def test_navigate_gps_not_ok_no_nav():
    print('\n[nav] GPS not reliable → no nav sent')
    m = StageStateMachineLogic()
    m.gps_ok = False
    m.handle_set_stage(3)
    m.handle_navigate(39.0, 32.0, 0.0)
    check('no nav sent', m.nav_sent_count == 0)
    check('not NAVIGATING', m.stage_state != StageState.NAVIGATING)


# ── Tests: SearchArea ─────────────────────────────────────────────────────────

def test_search_area():
    print('\n[search] SearchArea → WAIT_SEARCH → SEARCHING')
    m = StageStateMachineLogic()
    m.handle_set_stage(1)
    m.handle_search(39.0, 32.0, 5.0)
    check('state SEARCHING', m.stage_state == StageState.SEARCHING)
    check('waypoints generated', len(m.search_waypoints) == 3)
    check('retry reset', m.retry_count == 0)


# ── Tests: StartExploration ───────────────────────────────────────────────────

def test_start_exploration():
    print('\n[explore] StartExploration → EXPLORING')
    m = StageStateMachineLogic()
    m.handle_set_stage(3)
    m.handle_start_exploration()
    check('state EXPLORING', m.stage_state == StageState.EXPLORING)
    check('explore_distance 0', m.explore_distance == 0.0)


# ── Tests: Arrival (stage-aware) ──────────────────────────────────────────────

def test_arrival_stage1_found():
    print('\n[arrival] stage 1 → FOUND')
    m = StageStateMachineLogic()
    m.handle_set_stage(1)
    m.handle_search(39.0, 32.0, 5.0)
    m.handle_arrival()
    check('state FOUND', m.stage_state == StageState.FOUND)


def test_arrival_stage2_found():
    print('\n[arrival] stage 2 → FOUND')
    m = StageStateMachineLogic()
    m.handle_set_stage(2)
    m.handle_search(39.0, 32.0, 5.0)
    m.handle_arrival()
    check('state FOUND', m.stage_state == StageState.FOUND)


def test_arrival_stage3_navigating_reached_entry():
    print('\n[arrival] stage 3 NAVIGATING → REACHED_ENTRY + task_finished')
    m = StageStateMachineLogic()
    m.handle_set_stage(3)
    m.handle_navigate(39.0, 32.0, 0.0)
    check('navigating', m.stage_state == StageState.NAVIGATING)
    m.handle_arrival()
    check('state REACHED_ENTRY', m.stage_state == StageState.REACHED_ENTRY)
    check('result task_finished', m.results == ['task_finished'])


def test_arrival_stage3_exploring_found_exit():
    print('\n[arrival] stage 3 EXPLORING → FOUND_EXIT')
    m = StageStateMachineLogic()
    m.handle_set_stage(3)
    m.handle_start_exploration()
    check('exploring', m.stage_state == StageState.EXPLORING)
    m.handle_arrival()
    check('state FOUND_EXIT', m.stage_state == StageState.FOUND_EXIT)


def test_arrival_stage4_reached():
    print('\n[arrival] stage 4 → REACHED + task_finished')
    m = StageStateMachineLogic()
    m.handle_set_stage(4)
    m.handle_navigate(39.0, 32.0, 0.0)
    m.handle_arrival()
    check('state REACHED', m.stage_state == StageState.REACHED)
    check('result task_finished', m.results == ['task_finished'])


# ── Tests: Nav failure retry ──────────────────────────────────────────────────

def test_nav_failure_retry_then_success():
    print('\n[navfail] retry ≤ max → re-navigate')
    m = StageStateMachineLogic(max_retry=3)
    m.handle_set_stage(3)
    m.handle_navigate(39.0, 32.0, 0.0)
    initial_nav = m.nav_sent_count
    m.handle_nav_failure()
    check('retry 1', m.retry_count == 1)
    check('nav resent', m.nav_sent_count == initial_nav + 1)
    check('still NAVIGATING', m.stage_state == StageState.NAVIGATING)


def test_nav_failure_max_then_error():
    print('\n[navfail] max retries → ERROR + navigation_failed')
    m = StageStateMachineLogic(max_retry=2)
    m.handle_set_stage(3)
    m.handle_navigate(39.0, 32.0, 0.0)
    m.handle_nav_failure()  # retry 1
    m.handle_nav_failure()  # retry 2
    m.handle_nav_failure()  # retry 3 > max(2) → ERROR
    check('state ERROR', m.stage_state == StageState.ERROR)
    check('result navigation_failed:stage_3',
          m.results == ['navigation_failed:stage_3'])


# ── run ───────────────────────────────────────────────────────────────────────

def main():
    print('═══ Stage State Machine Tests ═══')
    test_set_stage_valid()
    test_set_stage_invalid()
    test_set_stage_resets_retry()
    test_arm()
    test_disarm_idle()
    test_navigate_to_gps()
    test_navigate_no_datum_error()
    test_navigate_gps_not_ok_no_nav()
    test_search_area()
    test_start_exploration()
    test_arrival_stage1_found()
    test_arrival_stage2_found()
    test_arrival_stage3_navigating_reached_entry()
    test_arrival_stage3_exploring_found_exit()
    test_arrival_stage4_reached()
    test_nav_failure_retry_then_success()
    test_nav_failure_max_then_error()
    print(f'\n═══ {passed} passed, {failed} failed ═══')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
