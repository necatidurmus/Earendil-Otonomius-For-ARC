#!/usr/bin/env python3
"""
Search Executor — spiral/grid search pattern generation for ARC stages 1-2.

Extracted from mission_manager.py for separation of concerns.
Pure logic, no ROS dependency — testable standalone.

Usage:
    from earendil_navigation.search_executor import SearchExecutor
    executor = SearchExecutor(pattern='spiral', speed_mps=0.3)
    waypoints = executor.generate(center_lat, center_lon, radius_m)
"""

import math
from typing import List, Tuple


class SearchExecutor:
    """Generate search waypoints around a center point."""

    def __init__(self, pattern: str = 'spiral', speed_mps: float = 0.3):
        self.pattern = pattern
        self.speed_mps = speed_mps

    def generate(self, center_lat: float, center_lon: float,
                 radius_m: float) -> List[Tuple[float, float]]:
        """Generate search waypoints.

        Args:
            center_lat: Center latitude (degrees)
            center_lon: Center longitude (degrees)
            radius_m: Search radius (meters)

        Returns:
            List of (lat, lon) waypoints
        """
        if self.pattern == 'spiral':
            return self._spiral(center_lat, center_lon, radius_m)
        elif self.pattern == 'grid':
            return self._grid(center_lat, center_lon, radius_m)
        else:
            return self._spiral(center_lat, center_lon, radius_m)

    def _spiral(self, center_lat: float, center_lon: float,
                radius_m: float) -> List[Tuple[float, float]]:
        """Generate Archimedean spiral waypoints.

        Points increase in radius from center outward.
        Number of points scales with radius (1 point per 2m).
        """
        waypoints = []
        num_points = max(8, int(radius_m / 2.0))

        for i in range(num_points):
            angle = 2.0 * math.pi * i / num_points
            r = radius_m * (i + 1) / num_points
            dlat = r * math.cos(angle) / 111320.0
            dlon = r * math.sin(angle) / (
                111320.0 * math.cos(math.radians(center_lat)))
            wp = (center_lat + dlat, center_lon + dlon)
            waypoints.append(wp)

        return waypoints

    def _grid(self, center_lat: float, center_lon: float,
              radius_m: float) -> List[Tuple[float, float]]:
        """Generate lawnmower/grid waypoints.

        Rows spaced 2m apart, alternating direction.
        """
        waypoints = []
        spacing = 2.0  # meters between rows
        num_rows = max(2, int(radius_m * 2 / spacing))
        cos_lat = math.cos(math.radians(center_lat))

        for row in range(num_rows):
            y_offset = -radius_m + row * spacing
            if y_offset > radius_m:
                break

            # Alternating direction (lawnmower)
            if row % 2 == 0:
                x_range = range(-int(radius_m), int(radius_m) + 1, 2)
            else:
                x_range = range(int(radius_m), -int(radius_m) - 1, -2)

            for x_m in x_range:
                dlat = y_offset / 111320.0
                dlon = x_m / (111320.0 * cos_lat)
                wp = (center_lat + dlat, center_lon + dlon)
                waypoints.append(wp)

        return waypoints
