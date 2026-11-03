"""Read-only GPS fix monitor. No command interfaces."""
import time
from threading import RLock

class GPSFixMonitor:
    def __init__(self):
        self.lock = RLock()
        self.last_at = None
        self.fix_type = None
        self.source = None

    def update(self, message):
        with self.lock:
            self.fix_type = int(message.fix_type)
            self.source = (int(message.get_srcSystem()), int(message.get_srcComponent()))
            self.last_at = time.monotonic()

    def snapshot(self, expected_source=None, now=None):
        with self.lock:
            t = time.monotonic() if now is None else now
            age = None if self.last_at is None else t - self.last_at
            fresh = age is not None and 0 <= age <= 3.0
            matched = expected_source is None or self.source == expected_source
            return {"gpsFixType": self.fix_type, "gpsFixFresh": fresh and matched,
                    "gpsFixValid": bool(fresh and matched and self.fix_type is not None and self.fix_type >= 3),
                    "gpsFixAgeSeconds": round(age, 2) if age is not None else None}

ctd_gps_fix_monitor = GPSFixMonitor()
