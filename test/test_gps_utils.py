#!/usr/bin/env python3
"""
Test GPS coordinate utilities — WGS84 to local ENU conversion.

Usage:
  python3 test/test_gps_utils.py
"""

import sys
import os
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'earendil_navigation'))

from earendil_navigation.gps_utils import GPSOrigin, gps_to_local, gps_distance


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


def test_origin_same_point():
    """Same point as origin → (0, 0)."""
    origin = GPSOrigin(lat0=39.925, lon0=32.836)
    x, y = gps_to_local(39.925, 32.836, origin)
    print('Test: Same point as origin → (0, 0)')
    check(f'x≈0 ({x:.6f})', abs(x) < 0.001)
    check(f'y≈0 ({y:.6f})', abs(y) < 0.001)


def test_north_displacement():
    """1 km north → y≈1000, x≈0."""
    origin = GPSOrigin(lat0=39.925, lon0=32.836)
    # ~0.009 degrees latitude ≈ 1 km
    x, y = gps_to_local(39.934, 32.836, origin)
    print('Test: ~1 km north displacement')
    check(f'y≈1000m ({y:.1f})', abs(y - 1000.0) < 50.0)
    check(f'x≈0 ({x:.1f})', abs(x) < 10.0)


def test_east_displacement():
    """1 km east → x≈1000, y≈0."""
    origin = GPSOrigin(lat0=39.925, lon0=32.836)
    # ~0.011 degrees longitude at lat 39.925 ≈ 1 km
    x, y = gps_to_local(39.925, 32.847, origin)
    print('Test: ~1 km east displacement')
    check(f'x≈1000m ({x:.1f})', abs(x - 1000.0) < 100.0)
    check(f'y≈0 ({y:.1f})', abs(y) < 10.0)


def test_gps_distance_same():
    """Same point → distance 0."""
    d = gps_distance(39.925, 32.836, 39.925, 32.836)
    print('Test: Same point distance = 0')
    check(f'd≈0 ({d:.6f})', d < 0.001)


def test_gps_distance_known():
    """Known distance between two points."""
    # Ankara to ~1 km away
    d = gps_distance(39.925, 32.836, 39.934, 32.836)
    print('Test: ~1 km north distance')
    check(f'd≈1000m ({d:.1f})', abs(d - 1000.0) < 50.0)


def test_gps_distance_symmetric():
    """Distance A→B == B→A."""
    d1 = gps_distance(39.925, 32.836, 39.934, 32.847)
    d2 = gps_distance(39.934, 32.847, 39.925, 32.836)
    print('Test: Distance symmetric')
    check(f'd1≈d2 ({d1:.3f} vs {d2:.3f})', abs(d1 - d2) < 0.001)


def test_enu_consistency():
    """gps_to_local distance should match gps_distance approximately."""
    origin = GPSOrigin(lat0=39.925, lon0=32.836)
    x, y = gps_to_local(39.934, 32.847, origin)
    enu_dist = math.sqrt(x**2 + y**2)
    haversine_dist = gps_distance(39.925, 32.836, 39.934, 32.847)
    print('Test: ENU distance ≈ Haversine distance')
    check(f'ENU={enu_dist:.1f}m, Haversine={haversine_dist:.1f}m',
          abs(enu_dist - haversine_dist) < 50.0)


if __name__ == '__main__':
    print('=' * 60)
    print('GPS Utils Tests — Coordinate Conversion')
    print('=' * 60)
    print()

    test_origin_same_point()
    print()
    test_north_displacement()
    print()
    test_east_displacement()
    print()
    test_gps_distance_same()
    print()
    test_gps_distance_known()
    print()
    test_gps_distance_symmetric()
    print()
    test_enu_consistency()

    print()
    print('=' * 60)
    print(f'Results: {passed} passed, {failed} failed')
    print('=' * 60)
    sys.exit(1 if failed > 0 else 0)
