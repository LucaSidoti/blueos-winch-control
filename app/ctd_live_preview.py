"""Passive bridge from BlueOS observation state to offline CTD preview.

No depth or HOME data are fabricated. No hardware or MAVLink send capability.
"""
import time
from threading import RLock
from ctd_orchestrator import CTDOrchestratorPreview, PreviewInput
from ctd_gps_fix import ctd_gps_fix_monitor

class LiveCTDPreview:
    def __init__(self):
        self.lock = RLock()
        self.engine = CTDOrchestratorPreview()
        self.key = None

    def reset(self):
        with self.lock:
            self.engine.reset()
            self.key = None

    def observe(self, *, stations, audit, mission, position_monitor, now=None):
        with self.lock:
            now = time.monotonic() if now is None else now
            source = (mission.get("sourceSystem"), mission.get("sourceComponent"))
            gps = ctd_gps_fix_monitor.snapshot(expected_source=source, now=now)
            valid = bool(audit.get("missionValidated"))
            # A successful audit generation is the identity of this readback.
            # Invalidate/revalidate when the vehicle mission changes.
            generation = audit.get("auditGeneration")
            key = (generation, tuple((s.get("loiterSequence"), s.get("latitude"),
                                      s.get("longitude"), s.get("targetDepth"),
                                      s.get("samplingDuration")) for s in stations))
            if self.key != key:
                self.engine.reset()
                self.key = key
            if not valid or not stations:
                self.engine.reset()
                return self._result("mission_not_validated_or_unconfigured", gps)
            if generation is None:
                self.engine.reset()
                return self._result("audit_identity_unavailable", gps)
            if self.engine.status == "idle":
                self.engine.configure(stations, repr(key), now=now)
            idx = self.engine.station_index
            if idx >= len(stations) or self.engine.status != "running":
                return self._result(None, gps)
            station = stations[idx]
            pos = position_monitor.snapshot(station, expected_source=source)
            with position_monitor.lock:
                sample_at = position_monitor.last_received
            # Never fabricate depth feedback or HOME confirmation.
            item = PreviewInput(now=now, sequence=mission.get("currentSequence"),
                mission_validated=valid,
                mission_telemetry_fresh=bool(mission.get("telemetryFresh")),
                position_fresh=bool(pos.get("positionFresh")),
                gps_fix_valid=bool(gps["gpsFixValid"]),
                position_sample_id=int(sample_at * 1e6) if sample_at is not None else None,
                position_error_m=pos.get("positionErrorM"),
                speed_m_s=pos.get("speedMps"),
                depth_m=None, depth_feedback_ok=False, home_verified=False,
                mission_signature=repr(key))
            self.engine.step(item)
            return self._result(None, gps)

    def _result(self, blocked, gps):
        snap = self.engine.snapshot()
        snap.update({"mode":"live_telemetry_dry_run", "gps":gps,
                     "depthFeedbackConnected":False, "homeFeedbackConnected":False,
                     "blockedReason":blocked or ("depth_and_home_feedback_not_connected"
                       if snap["lifecycle"] and snap["lifecycle"]["phase"] in
                       ("descending","sampling","retracting") else None),
                     "note":"Observation only; no actuation or mission advancement."})
        return snap

live_ctd_preview = LiveCTDPreview()
