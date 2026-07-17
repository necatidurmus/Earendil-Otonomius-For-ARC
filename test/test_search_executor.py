#!/usr/bin/env python3
"""Unit tests for SearchExecutor — pure logic, no ROS required."""

import sys
import os
import math

# Add src to path for direct import
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'earendil_navigation'))

from earendil_navigation.search_executor import SearchExecutor


def test_spiral_basic():
    """Spiral generates correct number of points."""
    ex = SearchExecutor(pattern='spiral')
    wps = ex.generate(39.925, 32.866, 10.0)
    assert len(wps) == max(8, int(10.0 / 2.0)), f'Expected 8, got {len(wps)}'
    print(f'PASS: spiral basic — {len(wps)} waypoints')


def test_spiral_radius_scaling():
    """Spiral points increase in distance from center."""
    ex = SearchExecutor(pattern='spiral')
    center = (39.925, 32.866)
    wps = ex.generate(center[0], center[1], 20.0)

    def haversine_m(p1, p2):
        R = 6371000.0
        lat1, lat2 = math.radians(p1[0]), math.radians(p2[0])
        dlat = math.radians(p2[0] - p1[0])
        dlon = math.radians(p2[1] - p1[1])
        a = (math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2)
        return 2 * R * math.asin(math.sqrt(a))

    d_first = haversine_m(center, wps[0])
    d_last = haversine_m(center, wps[-1])
    assert d_last > d_first, f'Last ({d_last:.1f}m) should be farther than first ({d_first:.1f}m)'
    print(f'PASS: spiral radius scaling — first={d_first:.1f}m, last={d_last:.1f}m')


def test_grid_generates():
    """Grid pattern generates waypoints."""
    ex = SearchExecutor(pattern='grid')
    wps = ex.generate(39.925, 32.866, 10.0)
    assert len(wps) > 0, 'Grid should generate at least 1 waypoint'
    print(f'PASS: grid basic — {len(wps)} waypoints')


def test_unknown_pattern_falls_back():
    """Unknown pattern falls back to spiral."""
    ex = SearchExecutor(pattern='unknown')
    wps = ex.generate(39.925, 32.866, 5.0)
    assert len(wps) > 0, 'Fallback should generate waypoints'
    print(f'PASS: unknown pattern fallback — {len(wps)} waypoints')


def test_zero_radius():
    """Zero radius generates minimal points."""
    ex = SearchExecutor(pattern='spiral')
    wps = ex.generate(39.925, 32.866, 0.0)
    assert len(wps) >= 8, f'Min 8 points expected, got {len(wps)}'
    print(f'PASS: zero radius — {len(wps)} waypoints')


if __name__ == '__main__':
    test_spiral_basic()
    test_spiral_radius_scaling()
    test_grid_generates()
    test_unknown_pattern_falls_back()
    test_zero_radius()
    print('\nAll search_executor tests PASSED')
