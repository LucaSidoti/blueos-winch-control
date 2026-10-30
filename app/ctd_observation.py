"""Read-only GPS/speed telemetry for CTD arrival diagnostics.

No actuator, MAVLink transmission, or autonomous execution capability.
"""
import math
import time
from threading import RLock


def distance_m(lat1, lon1, lat2, lon2):
    """Haversine distance, in meters."""
    vals = (lat1, lon1, lat2, lon2)
    if not all(isinstance(x, (float, int)) and math.isfinite(x) for x in vals):
        raise ValueError('Invalid coordinates')
    if not all((-90 <= x <= 90) for x in (lat1, lat2)) or not all((-180 <= x <= 180) for x in (lon1, lon2)):
        raise ValueError('Coordinates out of range')
    a, b = math.radians(lat1), math.radians(lat2)
    da, db = b-a, math.radians(lon2-lon1)
    h = math.sin(da/2)**2 + math.cos(a)*math.cos(b)*math.sin(db/2)**2
    return 6371000 * 2 * math.asin(min(1, math.sqrt(max(0, h))))


class CTDPositionMonitor:
    def __init__(self):
        self.lock = RLock()
        self.position = None
        self.last_received = None
        self.source = None

    def update_global_position(self, message):
        # Only valid GLOBAL_POSITION_INT with plausible lat/lon and speed.
        lat = int(message.lat) / 1e7
        lon = int(message.lon) / 1e7
        vx, vy = int(message.vx) / 100, int(message.vy) / 100
        if not all(math.isfinite(v) for v in (lat, lon, vx, vy)):
            return
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            return
        with self.lock:
            self.position = (lat, lon, math.hypot(vx, vy))
            self.last_received = time.monotonic()
            self.source = (message.get_srcSystem(), message.get_srcComponent())

    def snapshot(self, station=None, *, expected_source=None):
        with self.lock:
            age = None if self.last_received is None else time.monotonic() - self.last_received
            fresh = age is not None and age <= 3.0
            if expected_source is not None and self.source != expected_source:
                fresh = False
            lat, lon, speed = self.position if self.position else (None, None, None)
            result = {'positionFresh': fresh, 'positionAgeSeconds': round(age, 2) if age is not None else None,
                      'latitude': lat if fresh else None, 'longitude': lon if fresh else None,
                      'speedMps': speed if fresh else None, 'positionErrorM': None,
                      'withinStationTolerance': False, 'stableNow': False}
            if fresh and station:
                try:
                    error = distance_m(lat, lon, station['latitude'], station['longitude'])
                except (KeyError, TypeError, ValueError):
                    return result
                result['positionErrorM'] = round(error, 2)
                result['withinStationTolerance'] = error <= 3.0
                result['stableNow'] = error <= 3.0 and speed <= 0.3
            return result


ctd_position_monitor = CTDPositionMonitor()
