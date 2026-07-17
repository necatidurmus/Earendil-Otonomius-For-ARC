#!/usr/bin/env python3
"""Unit tests for ExplorationExecutor — pure logic, no ROS required."""

import sys
import os
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'earendil_navigation'))

from earendil_navigation.exploration_executor import ExplorationExecutor


def test_start_stop():
    """Start and stop exploration."""
    ex = ExplorationExecutor(timeout_s=300)
    ex.start(39.925, 32.866, timestamp=0.0)
    assert ex.is_running
    dist = ex.stop()
    assert dist == 0.0
    assert not ex.is_running
    print('PASS: start/stop')


def test_gps_accumulation():
    """GPS updates accumulate distance."""
    ex = ExplorationExecutor(timeout_s=300, gps_jitter_threshold_m=0.05)
    ex.start(39.925000, 32.866000, timestamp=0.0)

    # Move ~11m north (0.0001 deg lat ~ 11.1m)
    ex.update_gps(39.925100, 32.866000)
    dist = ex.get_distance()
    assert 10.0 < dist < 12.0, f'Expected ~11m, got {dist:.1f}m'
    print(f'PASS: GPS accumulation — {dist:.1f}m')


def test_gps_jitter_filter():
    """GPS jitter below threshold is filtered."""
    ex = ExplorationExecutor(timeout_s=300, gps_jitter_threshold_m=0.05)
    ex.start(39.925000, 32.866000, timestamp=0.0)

    # Tiny movement (0.000001 deg ~ 0.11m) — should accumulate
    ex.update_gps(39.925001, 32.866000)
    d1 = ex.get_distance()
    assert d1 > 0, 'Should accumulate movement > threshold'

    # Very tiny movement (0.0000001 deg ~ 0.01m) — should NOT accumulate
    ex.update_gps(39.9250011, 32.866000)
    d2 = ex.get_distance()
    assert d2 == d1, f'Jitter should be filtered: d1={d1:.4f}, d2={d2:.4f}'
    print(f'PASS: GPS jitter filter — {d1:.4f}m accumulated, jitter blocked')


def test_odom_accumulation():
    """Odometry updates accumulate distance."""
    ex = ExplorationExecutor(timeout_s=300)
    ex.start_odom(0.0, 0.0, timestamp=0.0)

    ex.update_odom(1.0, 0.0)
    assert abs(ex.get_distance() - 1.0) < 0.01

    ex.update_odom(1.0, 1.0)
    assert abs(ex.get_distance() - 2.0) < 0.01
    print(f'PASS: odom accumulation — {ex.get_distance():.1f}m')


def test_timeout():
    """Timeout detection works."""
    ex = ExplorationExecutor(timeout_s=10.0)
    ex.start(39.925, 32.866, timestamp=0.0)

    assert not ex.is_timeout(5.0)
    assert ex.is_timeout(11.0)
    print('PASS: timeout detection')


def test_elapsed():
    """Elapsed time calculation."""
    ex = ExplorationExecutor(timeout_s=300)
    ex.start(39.925, 32.866, timestamp=100.0)

    assert abs(ex.get_elapsed(105.0) - 5.0) < 0.01
    assert abs(ex.get_elapsed(200.0) - 100.0) < 0.01
    print('PASS: elapsed time')


def test_reset():
    """Reset clears all state."""
    ex = ExplorationExecutor(timeout_s=300)
    ex.start(39.925, 32.866, timestamp=0.0)
    ex.update_gps(39.926, 32.866)

    ex.reset()
    assert not ex.is_running
    assert ex.get_distance() == 0.0
    print('PASS: reset')


def test_update_when_not_running():
    """Update before start returns 0."""
    ex = ExplorationExecutor()
    dist = ex.update_gps(39.925, 32.866)
    assert dist == 0.0
    print('PASS: update before start')


if __name__ == '__main__':
    test_start_stop()
    test_gps_accumulation()
    test_gps_jitter_filter()
    test_odom_accumulation()
    test_timeout()
    test_elapsed()
    test_reset()
    test_update_when_not_running()
    print('\nAll exploration_executor tests PASSED')
