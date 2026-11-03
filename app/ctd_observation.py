"""Read-only GPS/speed telemetry for CTD arrival diagnostics.

No actuator, MAVLink transmission, or autonomous execution capability.
"""
import math
import time
from threading import RLock
from collections import deque


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
        self.samples = deque(maxlen=512)

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
            self.samples.append((self.last_received, lat, lon, self.position[2], self.source))

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


    def sustained_arrival(self, station, *, expected_source, at_loiter,
                          mission_validated, mission_fresh, sequence_entered=None, stable_seconds=10.0,
                          max_gap_seconds=2.0):
        """Read-only; count actual GPS samples, never HTTP polling as dwell time."""
        now = time.monotonic()
        result = {'arrivalConfirmed': False, 'stableDurationSeconds': 0.0,
                  'requiredStableSeconds': stable_seconds, 'deploymentAuthorized': False,
                  'arrivalBlockReason': 'missing_station'}
        if not station:
            return result
        if not mission_validated or not mission_fresh:
            result['arrivalBlockReason'] = 'mission_not_validated_or_stale'
            return result
        if sequence_entered is None:
            result['arrivalBlockReason'] = 'sequence_entry_unknown'
            return result
        if not at_loiter:
            result['arrivalBlockReason'] = 'not_at_loiter_sequence'
            return result
        with self.lock:
            samples = list(self.samples)
        if not samples or now - samples[-1][0] > 3.0:
            result['arrivalBlockReason'] = 'position_stale'
            return result
        if samples[-1][4] != expected_source:
            result['arrivalBlockReason'] = 'position_source_mismatch'
            return result
        # Scan backwards through a contiguous run of valid GPS observations.
        last = None
        earliest = None
        for timestamp, lat, lon, speed, source in reversed(samples):
            if (source != expected_source or
                (sequence_entered is not None and timestamp < sequence_entered) or
                (last is not None and last - timestamp > max_gap_seconds)):
                break
            try:
                stable = distance_m(lat, lon, station['latitude'], station['longitude']) <= 3.0 and speed <= 0.3
            except (KeyError, ValueError, TypeError):
                stable = False
            if not stable:
                break
            earliest = timestamp
            last = timestamp
        if earliest is None:
            result['arrivalBlockReason'] = 'outside_position_or_speed_limits'
            return result
        duration = max(0.0, samples[-1][0] - earliest)
        result['stableDurationSeconds'] = round(duration, 2)
        result['arrivalConfirmed'] = duration >= stable_seconds
        result['arrivalBlockReason'] = None if result['arrivalConfirmed'] else 'stabilizing'
        return result


ctd_position_monitor = CTDPositionMonitor()
