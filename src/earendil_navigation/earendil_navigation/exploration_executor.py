#!/usr/bin/env python3
"""
Exploration Executor — tunnel exploration distance measurement for ARC Stage 3.

Extracted from mission_manager.py for separation of concerns.
Pure logic, no ROS dependency — testable standalone.

Two measurement modes:
  1. GPS-based cumulative path length (outdoor)
  2. Odometry-based cumulative path length (indoor/tunnel)

Usage:
    from earendil_navigation.exploration_executor import ExplorationExecutor
    executor = ExplorationExecutor(timeout_s=300)
    executor.start(gps_lat, gps_lon)
    executor.update_gps(gps_lat, gps_lon)  # call periodically
    distance = executor.get_distance()
"""

import math
from typing import Optional, Tuple


class ExplorationExecutor:
    """Tunnel exploration distance measurement."""

    def __init__(self, timeout_s: float = 300.0, gps_jitter_threshold_m: float = 0.05):
        self._timeout_s = timeout_s
        self._gps_jitter_threshold = gps_jitter_threshold_m

        # State
        self._start_pos: Optional[Tuple[float, float]] = None
        self._prev_pos: Optional[Tuple[float, float]] = None
        self._distance: float = 0.0
        self._start_time: float = -1.0  # sentinel: -1 means not started
        self._running: bool = False

    def start(self, lat: float, lon: float, timestamp: float = 0.0):
        """Start exploration measurement.

        Args:
            lat: Starting latitude (degrees)
            lon: Starting longitude (degrees)
            timestamp: Current time (seconds, monotonic)
        """
        self._start_pos = (lat, lon)
        self._prev_pos = (lat, lon)
        self._distance = 0.0
        self._start_time = timestamp
        self._running = True

    def start_odom(self, x: float, y: float, timestamp: float = 0.0):
        """Start exploration measurement with odometry coordinates.

        Args:
            x: Starting x (meters, odom frame)
            y: Starting y (meters, odom frame)
            timestamp: Current time (seconds, monotonic)
        """
        self._start_pos = (x, y)
        self._prev_pos = (x, y)
        self._distance = 0.0
        self._start_time = timestamp
        self._running = True

    def update_gps(self, lat: float, lon: float) -> float:
        """Update with new GPS position. Returns accumulated distance.

        GPS jitter filter: only accumulate if step > threshold.
        """
        if not self._running or self._prev_pos is None:
            return self._distance

        dlat = lat - self._prev_pos[0]
        dlon = lon - self._prev_pos[1]
        dx = dlon * 111320.0 * math.cos(math.radians(lat))
        dy = dlat * 111320.0
        step = math.sqrt(dx * dx + dy * dy)

        if step > self._gps_jitter_threshold:
            self._distance += step

        self._prev_pos = (lat, lon)
        return self._distance

    def update_odom(self, x: float, y: float) -> float:
        """Update with new odometry position. Returns accumulated distance.

        No jitter filter for odom (already filtered by encoder).
        """
        if not self._running or self._prev_pos is None:
            return self._distance

        dx = x - self._prev_pos[0]
        dy = y - self._prev_pos[1]
        step = math.sqrt(dx * dx + dy * dy)
        self._distance += step

        self._prev_pos = (x, y)
        return self._distance

    def get_distance(self) -> float:
        """Get accumulated exploration distance in meters."""
        return self._distance

    def is_timeout(self, current_time: float) -> bool:
        """Check if exploration has exceeded timeout."""
        if not self._running or self._start_time < 0:
            return False
        return (current_time - self._start_time) > self._timeout_s

    def get_elapsed(self, current_time: float) -> float:
        """Get elapsed exploration time in seconds."""
        if self._start_time < 0:
            return 0.0
        return current_time - self._start_time

    def stop(self) -> float:
        """Stop exploration and return final distance."""
        self._running = False
        return self._distance

    def reset(self):
        """Reset all state."""
        self._start_pos = None
        self._prev_pos = None
        self._distance = 0.0
        self._start_time = -1.0
        self._running = False

    @property
    def is_running(self) -> bool:
        return self._running
