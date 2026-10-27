"""Read-only MAVLink mission download and CTD consistency audit.

Requests mission metadata/items only. Never changes mission or vehicle state.
"""
import math
import time
from threading import RLock
WAYPOINT = 16  # MAV_CMD_NAV_WAYPOINT
LOITER_UNLIM = 17  # MAV_CMD_NAV_LOITER_UNLIM
MISSION_TYPE = 0  # MAV_MISSION_TYPE_MISSION


def validate_items(stations, items, tolerance_deg=0.00001):
    """Validate a complete mission snapshot against configured CTD stations."""
    errors = []
    for index, station in enumerate(stations, 1):
        wp = items.get(station.waypoint_sequence)
        loiter = items.get(station.loiter_sequence)
        if wp is None:
            errors.append(f"Station {index}: waypoint {station.waypoint_sequence} missing")
        else:
            if wp['command'] != WAYPOINT:
                errors.append(f"Station {index}: expected NAV_WAYPOINT at {station.waypoint_sequence}")
            if abs(wp['latitude'] - station.latitude) > tolerance_deg or abs(wp['longitude'] - station.longitude) > tolerance_deg:
                errors.append(f"Station {index}: waypoint coordinates differ from CTD configuration")
        if loiter is None:
            errors.append(f"Station {index}: loiter {station.loiter_sequence} missing")
        else:
            if loiter['command'] != LOITER_UNLIM:
                errors.append(f"Station {index}: expected NAV_LOITER_UNLIM at {station.loiter_sequence}")
            if abs(loiter['latitude'] - station.latitude) > tolerance_deg or abs(loiter['longitude'] - station.longitude) > tolerance_deg:
                errors.append(f"Station {index}: loiter coordinates differ from CTD configuration")
    return errors


class MissionAudit:
    """Mission protocol state machine; all MAVLink I/O stays in listener thread."""
    def __init__(self):
        self.lock = RLock()
        self.generation = 0
        self.phase = 'idle'
        self.error = None
        self.items = {}
        self.count = None
        self.target = None
        self.deadline = 0.0
        self.last_request = 0.0
        self.stations = []
        self.result = None
        self.completed_at = None

    def invalidate(self):
        with self.lock:
            self.generation += 1
            self.phase = 'idle'
            self.error = None
            self.items = {}
            self.count = None
            self.target = None
            self.result = None
            self.completed_at = None

    def start(self, stations, system, component):
        with self.lock:
            if self.phase in ('waiting_count', 'downloading'):
                return False, 'Mission download already in progress'
            if not stations:
                return False, 'No CTD stations configured'
            if not system or not component:
                return False, 'No autopilot telemetry source available'
            self.generation += 1
            self.phase = 'request_list'
            self.error = None
            self.items = {}
            self.count = None
            self.target = (system, component)
            self.stations = list(stations)
            self.result = None
            self.completed_at = None
            self.deadline = time.monotonic() + 20
            self.last_request = 0.0
            return True, None

    def _fail(self, reason):
        self.phase = 'error'
        self.error = reason
        self.result = None

    def tick(self, conn):
        with self.lock:
            if self.phase not in ('request_list', 'waiting_count', 'downloading'):
                return
            now = time.monotonic()
            if now > self.deadline:
                self._fail('Mission download timed out; no validation performed')
                return
            if now - self.last_request < 1.0:
                return
            system, component = self.target
            try:
                if self.phase in ('request_list', 'waiting_count'):
                    conn.mav.mission_request_list_send(system, component, MISSION_TYPE)
                    self.phase = 'waiting_count'
                else:
                    missing = next((i for i in range(self.count) if i not in self.items), None)
                    if missing is not None:
                        conn.mav.mission_request_int_send(system, component, missing, MISSION_TYPE)
            except Exception as exc:
                self._fail(f'Mission request failed: {exc}')
            self.last_request = now

    def receive(self, message):
        kind = message.get_type()
        with self.lock:
            if self.phase not in ('waiting_count', 'downloading') or not self.target:
                return
            if (message.get_srcSystem(), message.get_srcComponent()) != self.target:
                return
            if kind == 'MISSION_COUNT' and self.phase == 'waiting_count':
                if getattr(message, 'mission_type', MISSION_TYPE) != MISSION_TYPE:
                    return
                count = int(message.count)
                if count < 1 or count > 1000:
                    self._fail('Invalid or excessive mission item count')
                    return
                self.count = count
                self.phase = 'downloading'
                self.last_request = 0.0
            elif kind in ('MISSION_ITEM_INT', 'MISSION_ITEM') and self.phase == 'downloading':
                if getattr(message, 'mission_type', MISSION_TYPE) != MISSION_TYPE:
                    return
                seq = int(message.seq)
                if not 0 <= seq < self.count:
                    return
                if kind == 'MISSION_ITEM_INT':
                    lat, lon = float(message.x) / 1e7, float(message.y) / 1e7
                else:
                    lat, lon = float(message.x), float(message.y)
                if not (math.isfinite(lat) and math.isfinite(lon) and -90 <= lat <= 90 and -180 <= lon <= 180):
                    self._fail('Invalid mission coordinates received')
                    return
                self.items[seq] = {'command': int(message.command), 'latitude': lat, 'longitude': lon}
                self.last_request = 0.0
                if len(self.items) == self.count:
                    errors = validate_items(self.stations, self.items)
                    self.result = {'valid': not errors, 'errors': errors, 'itemCount': self.count}
                    self.phase = 'complete'
                    self.completed_at = time.monotonic()
            elif kind == 'MISSION_ACK' and self.phase in ('waiting_count', 'downloading'):
                if int(message.type) != 0:  # MAV_MISSION_ACCEPTED
                    self._fail(f'Autopilot rejected mission request (code {message.type})')

    def snapshot(self):
        with self.lock:
            return {
                'status': self.phase,
                'missionValidated': (self.phase == 'complete' and self.result['valid']
                                     and self.completed_at is not None
                                     and time.monotonic() - self.completed_at <= 60),
                'snapshotOnly': True,
                'snapshotAgeSeconds': (round(time.monotonic() - self.completed_at, 2)
                                       if self.completed_at is not None else None),
                'missionItemCount': self.count,
                'receivedItemCount': len(self.items),
                'errors': self.result['errors'] if self.result else ([self.error] if self.error else []),
                'executionEnabled': False,
            }

mission_audit = MissionAudit()
