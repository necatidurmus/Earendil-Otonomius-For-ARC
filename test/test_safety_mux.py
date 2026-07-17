#!/usr/bin/env python3
"""
Test safety_mux logic (extracted without ROS).

Tests priority ordering, gate logic, speed limiting, and arm/disarm gate.

Usage:
  python3 test/test_safety_mux.py
"""

import sys
import os
import time
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'earendil_safety'))

# ── Extracted safety mux logic for standalone testing ────────────────────────

class SafetyMuxLogic:
    """Testable safety mux logic — same as safety_mux.py but without ROS."""

    def __init__(self, max_linear=0.5, max_angular=1.0,
                 command_timeout=0.3, watchdog_timeout=0.4,
                 deadman_timeout=1.0, estop_latching=True,
                 arm_required=True):
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.command_timeout = command_timeout
        self.watchdog_timeout = watchdog_timeout
        self.deadman_timeout = deadman_timeout
        self.estop_latching = estop_latching
        self.arm_required = arm_required

        # State
        self.estop_active = False
        self.deadman_held = False
        self.last_deadman_time = 0.0
        self.armed = False
        self.last_stm_time = 0.0

        # Per-topic: (vx, wz, timestamp)
        self.cmd_nav = (0.0, 0.0, 0.0)
        self.cmd_manual = (0.0, 0.0, 0.0)

        self.last_output = (0.0, 0.0)
        self.last_reason = ''

    def set_estop(self, active):
        self.estop_active = active

    def set_deadman(self, held):
        self.deadman_held = held
        if held:
            self.last_deadman_time = time.time()

    def set_armed(self, armed):
        self.armed = armed

    def set_stm_heartbeat(self):
        self.last_stm_time = time.time()

    def set_nav_cmd(self, vx, wz):
        self.cmd_nav = (vx, wz, time.time())

    def set_manual_cmd(self, vx, wz):
        self.cmd_manual = (vx, wz, time.time())

    def _limit(self, vx, wz):
        vx = max(-self.max_linear, min(self.max_linear, vx))
        wz = max(-self.max_angular, min(self.max_angular, wz))
        return (vx, wz)

    def evaluate(self):
        """Evaluate safety gates and return (vx, wz, reason).
        Same logic as safety_mux._safety_loop."""
        now = time.time()

        # Gate 1: E-stop
        if self.estop_active:
            return (0.0, 0.0, 'e-stop')

        # Gate 2: Arm gate
        if self.arm_required and not self.armed:
            return (0.0, 0.0, 'disarmed')

        # Gate 3: Deadman
        if self.last_deadman_time > 0:
            if now - self.last_deadman_time > self.deadman_timeout:
                return (0.0, 0.0, 'deadman_timeout')

        # Gate 4: STM watchdog
        if self.last_stm_time > 0:
            if now - self.last_stm_time > self.watchdog_timeout:
                return (0.0, 0.0, 'stm_watchdog')

        # Gate 5: Command timeout — manual priority
        _, _, manual_t = self.cmd_manual
        _, _, nav_t = self.cmd_nav

        if manual_t > 0 and now - manual_t < self.command_timeout:
            vx, wz, _ = self.cmd_manual
            self.last_output = self._limit(vx, wz)
            self.last_reason = 'manual'
            return self.last_output + ('manual',)

        if nav_t > 0 and now - nav_t < self.command_timeout:
            vx, wz, _ = self.cmd_nav
            self.last_output = self._limit(vx, wz)
            self.last_reason = 'nav'
            return self.last_output + ('nav',)

        # No valid command
        return (0.0, 0.0, 'no_command')


# ── Tests ────────────────────────────────────────────────────────────────────

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


def test_estop_blocks_all():
    """E-stop active → zero output regardless of commands."""
    mux = SafetyMuxLogic(arm_required=False)
    mux.set_estop(True)
    mux.set_stm_heartbeat()
    mux.set_nav_cmd(1.0, 0.5)
    vx, wz, reason = mux.evaluate()
    print('Test: E-stop blocks all')
    check('vx=0', vx == 0.0)
    check('wz=0', wz == 0.0)
    check('reason=e-stop', reason == 'e-stop')


def test_estop_latching():
    """E-stop latching: stays active until explicitly cleared."""
    mux = SafetyMuxLogic(arm_required=False)
    mux.set_estop(True)
    mux.set_stm_heartbeat()
    mux.set_nav_cmd(1.0, 0.0)

    # Active
    vx, wz, reason = mux.evaluate()
    check('E-stop active', reason == 'e-stop')

    # Still active (latched)
    vx, wz, reason = mux.evaluate()
    check('Still latched', reason == 'e-stop')

    # Clear
    mux.set_estop(False)
    vx, wz, reason = mux.evaluate()
    check('Cleared → forward', reason == 'nav')
    check('vx>0', vx > 0)


def test_disarmed_blocks():
    """Disarmed (arm_required=True) → zero output."""
    mux = SafetyMuxLogic(arm_required=True)
    mux.set_stm_heartbeat()
    mux.set_nav_cmd(1.0, 0.0)
    vx, wz, reason = mux.evaluate()
    print('Test: Disarmed blocks')
    check('vx=0', vx == 0.0)
    check('reason=disarmed', reason == 'disarmed')


def test_armed_allows():
    """Armed → commands pass through."""
    mux = SafetyMuxLogic(arm_required=True)
    mux.set_armed(True)
    mux.set_stm_heartbeat()
    mux.set_nav_cmd(0.5, 0.3)
    vx, wz, reason = mux.evaluate()
    print('Test: Armed allows commands')
    check('vx≈0.5', abs(vx - 0.5) < 0.01)
    check('wz≈0.3', abs(wz - 0.3) < 0.01)
    check('reason=nav', reason == 'nav')


def test_manual_priority_over_nav():
    """Manual has priority over nav when both active."""
    mux = SafetyMuxLogic(arm_required=False, max_linear=2.0, max_angular=2.0)
    mux.set_stm_heartbeat()
    mux.set_nav_cmd(0.5, 0.0)
    mux.set_manual_cmd(1.0, 0.5)
    vx, wz, reason = mux.evaluate()
    print('Test: Manual priority over nav')
    check('vx=1.0 (manual)', vx == 1.0)
    check('wz=0.5 (manual)', wz == 0.5)
    check('reason=manual', reason == 'manual')


def test_nav_fallback():
    """Nav command when no manual → nav used."""
    mux = SafetyMuxLogic(arm_required=False)
    mux.set_stm_heartbeat()
    mux.set_nav_cmd(0.5, 0.2)
    vx, wz, reason = mux.evaluate()
    print('Test: Nav fallback (no manual)')
    check('vx≈0.5', abs(vx - 0.5) < 0.01)
    check('wz≈0.2', abs(wz - 0.2) < 0.01)
    check('reason=nav', reason == 'nav')


def test_command_timeout():
    """Command timeout (300ms) → zero."""
    mux = SafetyMuxLogic(arm_required=False, command_timeout=0.3)
    mux.set_stm_heartbeat()
    mux.set_nav_cmd(1.0, 0.0)

    # Immediate → ok
    vx, wz, reason = mux.evaluate()
    check('Fresh cmd → vx>0', vx > 0)

    # After timeout
    time.sleep(0.35)
    vx, wz, reason = mux.evaluate()
    print('Test: Command timeout (300ms)')
    check('Timed out → vx=0', vx == 0.0)
    check('reason=no_command', reason == 'no_command')


def test_stm_watchdog():
    """STM watchdog timeout → zero."""
    mux = SafetyMuxLogic(arm_required=False, watchdog_timeout=0.2)
    mux.set_stm_heartbeat()
    mux.set_nav_cmd(1.0, 0.0)

    # Fresh
    vx, wz, reason = mux.evaluate()
    check('Fresh STM → vx>0', vx > 0)

    # After watchdog
    time.sleep(0.25)
    vx, wz, reason = mux.evaluate()
    print('Test: STM watchdog timeout')
    check('Watchdog → vx=0', vx == 0.0)
    check('reason=stm_watchdog', reason == 'stm_watchdog')


def test_speed_limit_linear():
    """Linear speed limiting."""
    mux = SafetyMuxLogic(arm_required=False, max_linear=0.5)
    mux.set_stm_heartbeat()
    mux.set_nav_cmd(2.0, 0.0)
    vx, wz, reason = mux.evaluate()
    print('Test: Linear speed limit (max 0.5)')
    check(f'vx clamped to 0.5 ({vx})', abs(vx - 0.5) < 0.01)


def test_speed_limit_angular():
    """Angular speed limiting."""
    mux = SafetyMuxLogic(arm_required=False, max_angular=1.0)
    mux.set_stm_heartbeat()
    mux.set_nav_cmd(0.0, 5.0)
    vx, wz, reason = mux.evaluate()
    print('Test: Angular speed limit (max 1.0)')
    check(f'wz clamped to 1.0 ({wz})', abs(wz - 1.0) < 0.01)


def test_negative_speed_limit():
    """Negative speed clamped to -max."""
    mux = SafetyMuxLogic(arm_required=False, max_linear=0.5, max_angular=1.0)
    mux.set_stm_heartbeat()
    mux.set_nav_cmd(-2.0, -3.0)
    vx, wz, reason = mux.evaluate()
    print('Test: Negative speed clamped')
    check('vx clamped to -0.5', abs(vx - (-0.5)) < 0.01)
    check('wz clamped to -1.0', abs(wz - (-1.0)) < 0.01)


def test_no_command():
    """No commands → zero."""
    mux = SafetyMuxLogic(arm_required=False)
    mux.set_stm_heartbeat()
    vx, wz, reason = mux.evaluate()
    print('Test: No commands → zero')
    check('vx=0', vx == 0.0)
    check('wz=0', wz == 0.0)
    check('reason=no_command', reason == 'no_command')


def test_deadman_timeout():
    """Deadman timeout → zero."""
    mux = SafetyMuxLogic(arm_required=False, deadman_timeout=0.2)
    mux.set_stm_heartbeat()
    mux.set_nav_cmd(1.0, 0.0)

    # Deadman held
    mux.set_deadman(True)
    vx, wz, reason = mux.evaluate()
    check('Deadman held → vx>0', vx > 0)

    # Deadman expired
    time.sleep(0.25)
    vx, wz, reason = mux.evaluate()
    print('Test: Deadman timeout')
    check('Expired → vx=0', vx == 0.0)
    check('reason=deadman_timeout', reason == 'deadman_timeout')


def test_priority_order():
    """Full priority: e-stop > disarmed > deadman > watchdog > timeout."""
    mux = SafetyMuxLogic(arm_required=True, watchdog_timeout=0.2, deadman_timeout=0.2)
    mux.set_stm_heartbeat()
    mux.set_nav_cmd(1.0, 0.0)

    # All ok
    vx, wz, reason = mux.evaluate()
    check('All ok → disarmed (arm_required)', reason == 'disarmed')

    # Arm
    mux.set_armed(True)
    vx, wz, reason = mux.evaluate()
    check('Armed → vx>0', vx > 0)

    # Deadman timeout
    mux.set_deadman(True)
    time.sleep(0.25)
    vx, wz, reason = mux.evaluate()
    check('Deadman timeout → zero', reason == 'deadman_timeout')

    # Reset deadman, set stm watchdog
    mux.set_deadman(True)
    mux.last_stm_time = time.time() - 0.3
    vx, wz, reason = mux.evaluate()
    check('STM watchdog → zero', reason == 'stm_watchdog')

    # E-stop overrides all
    mux.set_stm_heartbeat()
    mux.set_estop(True)
    vx, wz, reason = mux.evaluate()
    check('E-stop overrides all', reason == 'e-stop')


if __name__ == '__main__':
    print('=' * 60)
    print('Safety Mux Tests — Gate Logic')
    print('=' * 60)
    print()

    test_estop_blocks_all()
    print()
    test_estop_latching()
    print()
    test_disarmed_blocks()
    print()
    test_armed_allows()
    print()
    test_manual_priority_over_nav()
    print()
    test_nav_fallback()
    print()
    test_command_timeout()
    print()
    test_stm_watchdog()
    print()
    test_speed_limit_linear()
    print()
    test_speed_limit_angular()
    print()
    test_negative_speed_limit()
    print()
    test_no_command()
    print()
    test_deadman_timeout()
    print()
    test_priority_order()

    print()
    print('=' * 60)
    print(f'Results: {passed} passed, {failed} failed')
    print('=' * 60)
    sys.exit(1 if failed > 0 else 0)
