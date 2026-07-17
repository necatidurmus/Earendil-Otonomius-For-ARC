"""GPS coordinate utilities — WGS84 to local ENU conversion.

Simple Mercator-like projection for short-range (< 1 km) ARC use.
For larger ranges, consider pyproj or geographiclib.

Usage:
    from earendil_navigation.gps_utils import GPSOrigin, gps_to_local

    origin = GPSOrigin(lat0=39.925, lon0=32.866)  # datum from first fix
    x, y = gps_to_local(lat=39.926, lon=32.867, origin=origin)
"""

import math
from dataclasses import dataclass

# WGS84 ellipsoid
_EARTH_RADIUS_M = 6371000.0  # mean radius, adequate for < 1 km


@dataclass
class GPSOrigin:
    """Datum point for local ENU frame."""
    lat0: float  # degrees
    lon0: float  # degrees

    # Pre-computed cos(lat) for repeated use
    _cos_lat: float = 0.0

    def __post_init__(self):
        self._cos_lat = math.cos(math.radians(self.lat0))


def gps_to_local(lat: float, lon: float, origin: GPSOrigin) -> tuple:
    """Convert WGS84 lat/lon to local ENU x/y (meters) relative to origin.

    Args:
        lat: target latitude (degrees)
        lon: target longitude (degrees)
        origin: GPSOrigin datum

    Returns:
        (x, y) in meters: x=East, y=North
    """
    dlat = math.radians(lat - origin.lat0)
    dlon = math.radians(lon - origin.lon0)

    x = dlon * _EARTH_RADIUS_M * origin._cos_lat  # East
    y = dlat * _EARTH_RADIUS_M                     # North

    return x, y


def gps_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Haversine distance between two WGS84 points in meters."""
    lat1_r, lat2_r = math.radians(lat1), math.radians(lat2)
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2 +
         math.cos(lat1_r) * math.cos(lat2_r) * math.sin(dlon / 2) ** 2)
    return 2 * _EARTH_RADIUS_M * math.asin(math.sqrt(a))
