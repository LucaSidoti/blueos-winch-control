import unittest
from unittest.mock import patch
from ctd_live_preview import LiveCTDPreview
from ctd_gps_fix import GPSFixMonitor

STATION = {"loiterSequence": 2, "latitude": 46.5, "longitude": 6.5,
           "targetDepth": 1.3, "samplingDuration": 15}

class FakePosition:
    def __init__(self):
        from threading import RLock
        self.lock = RLock()
        self.last_received = None
    def snapshot(self, station, expected_source):
        return {"positionFresh": True, "positionErrorM": 1.0, "speedMps": 0.1}

class LiveTests(unittest.TestCase):
    def setUp(self):
        self.preview = LiveCTDPreview()
        self.pos = FakePosition()
        self.audit = {"missionValidated": True, "auditGeneration": 7}
        self.mission = {"sourceSystem": 1, "sourceComponent": 1,
                        "currentSequence": 0, "telemetryFresh": True}
    def read(self, now):
        return self.preview.observe(stations=[STATION], audit=self.audit,
                                    mission=self.mission, position_monitor=self.pos, now=now)
    def test_indoor_no_fix_stays_waiting(self):
        with patch("ctd_live_preview.ctd_gps_fix_monitor", GPSFixMonitor()):
            for t in range(15):
                r = self.read(t)
            self.assertEqual(r["lifecycle"]["phase"], "waiting_for_station")
            self.assertFalse(r["executionEnabled"])
            self.assertFalse(r["hardwareCommandsSent"])
    def test_invalidated_audit_resets(self):
        with patch("ctd_live_preview.ctd_gps_fix_monitor", GPSFixMonitor()):
            self.read(0)
            self.audit["missionValidated"] = False
            r = self.read(1)
            self.assertEqual(r["status"], "idle")
            self.assertEqual(r["blockedReason"], "mission_not_validated_or_unconfigured")
    def test_no_audit_identity_fails_closed(self):
        with patch("ctd_live_preview.ctd_gps_fix_monitor", GPSFixMonitor()):
            self.audit.pop("auditGeneration")
            r = self.read(0)
            self.assertEqual(r["blockedReason"], "audit_identity_unavailable")
    def test_no_depth_or_home_fabrication(self):
        with patch("ctd_live_preview.ctd_gps_fix_monitor", GPSFixMonitor()):
            r = self.read(0)
            self.assertFalse(r["depthFeedbackConnected"])
            self.assertFalse(r["homeFeedbackConnected"])
            self.assertFalse(r["automaticAdvanceEnabled"])

if __name__ == "__main__":
    unittest.main()
