"""CTD multi-station mission preview: pure logic, no hardware or MAVLink I/O.

This is NOT a vehicle controller. It consumes synthetic or prevalidated observations,
returns proposals, and cannot send commands. Explicitly reset after any fault.
"""
from dataclasses import dataclass
from threading import RLock
import math
from ctd_lifecycle import CTDLifecycle, Observation, Phase


@dataclass(frozen=True)
class PreviewInput:
    now: float
    sequence: int | None
    mission_validated: bool
    mission_telemetry_fresh: bool
    position_fresh: bool
    position_error_m: float | None = None
    speed_m_s: float | None = None
    depth_m: float | None = None
    depth_feedback_ok: bool = False
    home_verified: bool = False
    actuator_fault: bool = False
    mission_signature: str | None = None
    gps_fix_valid: bool = False
    position_sample_id: int | None = None


class CTDOrchestratorPreview:
    """Stateful dry-run for all CTD stations. No command interfaces by design."""
    def __init__(self, *, stable_s=10.0, max_sample_gap_s=2.0):
        self.lock = RLock()
        self.stable_s = stable_s
        self.max_sample_gap_s = max_sample_gap_s
        self.reset()

    def reset(self):
        with self.lock:
            self.stations = []
            self.station_index = 0
            self.lifecycle = None
            self.signature = None
            self.status = 'idle'
            self.fault = None
            self.last_now = None
            self.last_position_sample_id = None
            self.last_position_sample_at = None
            self.last_sequence = None
            self.reviewed_stations = []

    def configure(self, stations, mission_signature, now=0.0):
        """Start a NEW dry run only. A restart never resumes an in-flight cycle."""
        if not isinstance(mission_signature, str) or not mission_signature:
            raise ValueError('A validated mission signature is required')
        if not math.isfinite(now):
            raise ValueError('Invalid clock')
        normalized = [s.to_dict() if hasattr(s, 'to_dict') else dict(s) for s in stations]
        if not normalized:
            raise ValueError('No CTD stations')
        for s in normalized:
            if not all(k in s for k in ('loiterSequence', 'targetDepth', 'samplingDuration')):
                raise ValueError('Incomplete station')
        with self.lock:
            self.reset()
            self.stations = normalized
            self.signature = mission_signature
            self.last_now = now
            self.lifecycle = CTDLifecycle(normalized[0], stable_s=self.stable_s)
            self.lifecycle.start(now)
            self.status = 'running'
            return self.snapshot()

    def _fault(self, reason, now):
        self.status = 'fault'
        self.fault = reason
        if self.lifecycle is not None:
            self.lifecycle._fail(reason, now)

    def step(self, item: PreviewInput):
        with self.lock:
            if not math.isfinite(item.now) or (self.last_now is not None and item.now < self.last_now):
                raise ValueError('Invalid or nonmonotonic clock')
            self.last_now = item.now
            if self.status != 'running':
                return self.snapshot()
            if not item.mission_validated or item.mission_signature != self.signature:
                self._fault('Mission invalidated or changed', item.now)
                return self.snapshot()
            if not item.mission_telemetry_fresh:
                self._fault('Mission telemetry stale', item.now)
                return self.snapshot()
            if item.actuator_fault:
                self._fault('Actuator fault observed', item.now)
                return self.snapshot()
            phase = self.lifecycle.phase
            if item.sequence != self.stations[self.station_index]['loiterSequence']:
                if phase in (Phase.WAITING, Phase.STABILIZING):
                    self.lifecycle.stable_since = None
                elif phase != Phase.READY:
                    self._fault('Mission sequence changed during CTD operation', item.now)
                    return self.snapshot()
            # A position reading is never considered usable without a fix and a NEW sample.
            new_sample = item.position_sample_id is not None and item.position_sample_id != self.last_position_sample_id
            position_ok = item.position_fresh and item.gps_fix_valid and new_sample
            if new_sample:
                if self.last_position_sample_at is not None and item.now - self.last_position_sample_at > self.max_sample_gap_s:
                    self.lifecycle.stable_since = None
                    if phase in (Phase.DESCENDING, Phase.SAMPLING, Phase.RETRACTING):
                        self._fault('GPS sample gap during CTD operation', item.now)
                        return self.snapshot()
                self.last_position_sample_id = item.position_sample_id
                self.last_position_sample_at = item.now
            elif self.last_position_sample_at is None or item.now - self.last_position_sample_at > self.max_sample_gap_s:
                self.lifecycle.stable_since = None
                if phase in (Phase.DESCENDING, Phase.SAMPLING, Phase.RETRACTING):
                    self._fault('GPS sample timeout during CTD operation', item.now)
                    return self.snapshot()
            if not item.position_fresh or not item.gps_fix_valid:
                self.lifecycle.stable_since = None
                if phase in (Phase.DESCENDING, Phase.SAMPLING, Phase.RETRACTING):
                    self._fault('GPS fix or position lost during CTD operation', item.now)
                    return self.snapshot()
            # Require fresh samples to establish dwell; repeated HTTP polling does not count.
            if not new_sample and phase in (Phase.WAITING, Phase.STABILIZING):
                return self.snapshot()
            # Position must be valid throughout operations; repeated reads can retain the
            # latest observation only within the configured gap, never indefinitely.
            allowed_position = item.position_fresh and item.gps_fix_valid and (
                new_sample or (self.last_position_sample_at is not None and
                               item.now - self.last_position_sample_at <= self.max_sample_gap_s))
            observation = Observation(
                now=item.now, sequence=item.sequence, telemetry_fresh=item.mission_telemetry_fresh,
                mission_validated=item.mission_validated,
                position_error_m=item.position_error_m if allowed_position else None,
                speed_m_s=item.speed_m_s if allowed_position else None,
                depth_m=item.depth_m, depth_feedback_ok=item.depth_feedback_ok,
                home_verified=item.home_verified, actuator_fault=item.actuator_fault,
            )
            self.lifecycle.step(observation)
            if self.lifecycle.phase == Phase.FAULT:
                self.status = 'fault'
                self.fault = self.lifecycle.fault
            elif self.lifecycle.phase == Phase.READY:
                # No automatic mission advancement. Preview is blocked at HOME until
                # a simulated mission-sequence transition is observed separately.
                self.status = 'awaiting_simulated_advance'
            return self.snapshot()

    def simulate_advance(self, now, next_sequence):
        """Offline-only simulated autopilot acknowledgement; NEVER sends MAVLink."""
        with self.lock:
            if self.status != 'awaiting_simulated_advance':
                raise ValueError('HOME not verified or preview not ready')
            if not math.isfinite(now) or now < self.last_now:
                raise ValueError('Invalid clock')
            expected = (self.stations[self.station_index + 1]['loiterSequence']
                        if self.station_index + 1 < len(self.stations) else None)
            if next_sequence != expected:
                raise ValueError('Unexpected next station sequence')
            self.last_now = now
            self.reviewed_stations.append(self.station_index)
            self.station_index += 1
            if self.station_index == len(self.stations):
                self.status = 'complete'
                self.lifecycle = None
            else:
                self.lifecycle = CTDLifecycle(self.stations[self.station_index], stable_s=self.stable_s)
                self.lifecycle.start(now)
                self.status = 'running'
                self.last_position_sample_id = None
                self.last_position_sample_at = None
            return self.snapshot()

    def snapshot(self):
        with self.lock:
            state = self.lifecycle.snapshot() if self.lifecycle else None
            return {
                'mode': 'offline_preview_only', 'status': self.status,
                'stationIndex': self.station_index if self.stations else None,
                'stationCount': len(self.stations), 'lifecycle': state,
                'fault': self.fault, 'completedStations': list(self.reviewed_stations),
                'hardwareCommandsSent': False, 'mavlinkCommandsSent': False,
                'executionEnabled': False, 'automaticWinchEnabled': False,
                'automaticAdvanceEnabled': False,
            }
