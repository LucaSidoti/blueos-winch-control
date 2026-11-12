"""Pure CTD lifecycle decision engine. No motor, network, or MAVLink imports.

All outputs are *proposals* for an external supervised test harness. They must
never be wired directly to physical actuators or mission-advance commands.
"""
from dataclasses import dataclass
from enum import Enum
import math

class Phase(str, Enum):
    IDLE='idle'
    WAITING='waiting_for_station'
    STABILIZING='stabilizing'
    DESCENDING='descending'
    SAMPLING='sampling'
    RETRACTING='retracting'
    READY='ready_to_continue'
    COMPLETE='complete'
    FAULT='fault'

@dataclass(frozen=True)
class Observation:
    now: float
    sequence: int | None
    telemetry_fresh: bool
    mission_validated: bool
    position_error_m: float | None = None
    speed_m_s: float | None = None
    depth_m: float | None = None
    depth_feedback_ok: bool = False
    home_verified: bool = False
    actuator_fault: bool = False

class CTDLifecycle:
    """Offline state machine. `step` changes internal state but never performs IO."""
    def __init__(self, station, *, stable_s=5, max_error_m=3, max_speed_m_s=0.3,
                 depth_tolerance_m=0.08, descent_timeout_s=120, retract_timeout_s=120):
        self.station = dict(station)
        self.stable_s = stable_s
        self.max_error_m = max_error_m
        self.max_speed_m_s = max_speed_m_s
        self.depth_tolerance_m = depth_tolerance_m
        self.descent_timeout_s = descent_timeout_s
        self.retract_timeout_s = retract_timeout_s
        self.phase = Phase.IDLE
        self.since = None
        self.stable_since = None
        self.fault = None
        self.last_time = None

    def start(self, now):
        if self.phase != Phase.IDLE:
            raise ValueError('Already started')
        self._set(Phase.WAITING, now)

    def _set(self, phase, now):
        self.phase, self.since = phase, now

    def _fail(self, reason, now):
        self.fault = reason
        self._set(Phase.FAULT, now)

    def step(self, obs: Observation):
        if not math.isfinite(obs.now) or (self.last_time is not None and obs.now < self.last_time):
            raise ValueError('Nonmonotonic or invalid clock')
        self.last_time = obs.now
        if self.phase in (Phase.IDLE, Phase.FAULT, Phase.COMPLETE):
            return self.snapshot()
        if obs.actuator_fault or not obs.telemetry_fresh or not obs.mission_validated:
            self._fail('Fault, stale telemetry, or invalid mission', obs.now)
            return self.snapshot()
        loiter = self.station['loiterSequence']
        if obs.sequence != loiter and self.phase not in (Phase.READY,):
            if self.phase in (Phase.WAITING, Phase.STABILIZING):
                self.stable_since = None
                self._set(Phase.WAITING, obs.now)
            else:
                self._fail('Mission sequence changed during CTD operation', obs.now)
            return self.snapshot()
        stable = (obs.position_error_m is not None and obs.speed_m_s is not None
                  and math.isfinite(obs.position_error_m) and math.isfinite(obs.speed_m_s)
                  and 0 <= obs.position_error_m <= self.max_error_m
                  and 0 <= obs.speed_m_s <= self.max_speed_m_s)
        if self.phase in (Phase.WAITING, Phase.STABILIZING):
            if not stable:
                self.stable_since = None
                self._set(Phase.STABILIZING, obs.now)
            else:
                if self.stable_since is None:
                    self.stable_since = obs.now
                self._set(Phase.STABILIZING, obs.now)
                if obs.now - self.stable_since >= self.stable_s:
                    self._set(Phase.DESCENDING, obs.now)
        elif self.phase == Phase.DESCENDING:
            if obs.now - self.since > self.descent_timeout_s:
                self._fail('Descent timeout', obs.now)
            elif not stable:
                self._fail('Station keeping lost during descent', obs.now)
            elif (obs.depth_feedback_ok and obs.depth_m is not None and
                  math.isfinite(obs.depth_m) and
                  abs(obs.depth_m - self.station['targetDepth']) <= self.depth_tolerance_m):
                self._set(Phase.SAMPLING, obs.now)
        elif self.phase == Phase.SAMPLING:
            if not stable:
                self._fail('Station keeping lost during sampling', obs.now)
            elif not obs.depth_feedback_ok or obs.depth_m is None or not math.isfinite(obs.depth_m):
                self._fail('Depth feedback lost', obs.now)
            elif obs.now - self.since >= self.station['samplingDuration']:
                self._set(Phase.RETRACTING, obs.now)
        elif self.phase == Phase.RETRACTING:
            if obs.now - self.since > self.retract_timeout_s:
                self._fail('Retraction timeout', obs.now)
            elif obs.home_verified:
                self._set(Phase.READY, obs.now)
        return self.snapshot()

    def snapshot(self):
        return {'phase': self.phase.value, 'fault': self.fault,
                'station': dict(self.station), 'phaseSince': self.since,
                'proposedAction': {
                    Phase.DESCENDING: 'SIMULATE_DESCENT',
                    Phase.RETRACTING: 'SIMULATE_RETRACTION',
                    Phase.READY: 'REQUEST_OPERATOR_REVIEW_FOR_NEXT_STATION',
                }.get(self.phase),
                'hardwareCommandsSent': False, 'mavlinkCommandsSent': False}
