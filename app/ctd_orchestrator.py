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


# HYBRID SITL: simulated vehicle telemetry, real winch; bench-only commissioning.
# One explicitly started session; no persistent authorization token.
import os
import time
from threading import Thread, Event, RLock
import depth_control
from state import state, mission_monitor
from mission_audit import mission_audit
from ctd_observation import ctd_position_monitor
from ctd_gps_fix import ctd_gps_fix_monitor
from ctd_command_ownership import ownership, command_as
from ctd_encoder_feedback import assess as assess_home
from ctd_measured_velocity import read as measured_velocity
from ctd_supervised_motion import ProgressWatchdog, MAX_RUN_S

class HybridSITL:
    def __init__(self):
        self.lock=RLock()
        self.stop_event=Event()
        self.worker=None
        self.phase='idle'
        self.error=None
        self.completed=[]
        self.index=0
        self.stations=[]
        self.source=None
        self.audit_generation=None
        self.advance_request=None
        self.advance_sent_at=None
        self.mode=None
        self.mode_at=None

    def update_mode(self, mode, source):
        with self.lock:
            if self.source is None or tuple(source)==tuple(self.source):
                self.mode=mode
                self.mode_at=time.monotonic()

    def snapshot(self):
        with self.lock:
            return {'phase':self.phase,'error':self.error,'stationIndex':self.index,
                    'stationCount':len(self.stations),'completedStations':list(self.completed),
                    'missionAdvancePending':self.advance_request,
                    'mode':self.mode,'tokenRequired':False,'maxDepthMeters':2.0,
                    'encoderHomeIsPhysicalSwitch':False}

    def start(self, stations, depth_limit=2.0, sample_limit_s=15):
        # Explicit opt-in prevents unintentional motor activation at service startup.
        if os.environ.get('CTD_HYBRID_SITL_ENABLE') != '1':
            raise RuntimeError('Set CTD_HYBRID_SITL_ENABLE=1 on isolated bench only')
        if type(depth_limit) not in (int,float) or not 0 < depth_limit <= 2.0:
            raise ValueError('Hybrid depth ceiling must be <= 2 m')
        if type(sample_limit_s) is not int or not 1 <= sample_limit_s <= 60:
            raise ValueError('Sampling ceiling must be 1..60 seconds')
        with self.lock:
            if self.worker and self.worker.is_alive():
                raise RuntimeError('Hybrid session already running')
            if self.phase not in ('idle','finished','fault','stopped'):
                raise RuntimeError('Hybrid session not reset')
            if not stations:
                raise RuntimeError('No CTD stations configured')
            mission=mission_monitor.snapshot(); audit=mission_audit.snapshot()
            if not mission['telemetryFresh'] or not audit['missionValidated']:
                raise RuntimeError('Fresh mission readback and telemetry required')
            if audit.get('snapshotAgeSeconds',1e9)>30:
                raise RuntimeError('Mission readback too old')
            if self.mode!='AUTO' or self.mode_at is None or time.monotonic()-self.mode_at>3:
                raise RuntimeError('Fresh AUTO heartbeat required')
            if not state.initialized or not state.torque_enabled or state.lock_state!='unlocked' or state.retract_limit.fault:
                raise RuntimeError('Winch must be initialized, unlocked, torque enabled and fault-free')
            if state.depth.active or state.motion.velocity!=0 or state.home_adjust_mode or state.home_recovery_mode:
                raise RuntimeError('Winch busy')
            status=depth_control.get_status(read_position=True)
            if not assess_home(status,sampled_at=time.monotonic()).get('encoderHomeCandidate'):
                raise RuntimeError('Calibrated encoder HOME required before starting')
            if ownership.snapshot()['owner']!='manual':
                raise RuntimeError('Winch owned by another controller')
            self.source=(mission['sourceSystem'],mission['sourceComponent'])
            self.audit_generation=audit['auditGeneration']
            self.stations=[dict(s) for s in stations]
            self.index=0;self.completed=[];self.error=None
            self.advance_request=None;self.advance_sent_at=None
            self.depth_limit=float(depth_limit);self.sample_limit=int(sample_limit_s)
            self.stop_event.clear();self.phase='navigating'
            self.worker=Thread(target=self._run,daemon=True,name='ctd-hybrid-sitl')
            self.worker.start()
            return self.snapshot()

    def stop(self):
        with self.lock:
            self.stop_event.set()
            if self.phase not in ('finished','fault'):
                self.phase='stopped'
            self.advance_request=None
        # Zero velocity always permitted, including when motor ownership is held.
        depth_control.cancel(stop=True)
        return self.snapshot()

    def _evidence(self, station, require_loiter=True):
        mission=mission_monitor.snapshot();audit=mission_audit.snapshot()
        if not mission['telemetryFresh'] or tuple((mission['sourceSystem'],mission['sourceComponent']))!=self.source:
            raise RuntimeError('MAVLink mission source lost or changed')
        if not audit['missionValidated'] or audit['auditGeneration']!=self.audit_generation:
            raise RuntimeError('Mission validation changed or expired')
        if self.mode!='AUTO' or self.mode_at is None or time.monotonic()-self.mode_at>3:
            raise RuntimeError('SITL no longer reports fresh AUTO mode')
        if require_loiter and mission['currentSequence']!=station['loiterSequence']:
            raise RuntimeError('LOITER_UNLIM mission item changed')
        position=ctd_position_monitor.snapshot(station,expected_source=self.source)
        gps=ctd_gps_fix_monitor.snapshot(expected_source=self.source)
        if not position['positionFresh'] or not position['stableNow']:
            raise RuntimeError('Station GPS/speed stale or outside 3m/0.3m/s limits')
        if not gps.get('fixValid',False):
            # GPS monitor key checked below against actual snapshot format.
            if not (gps.get('gpsFixValid') or gps.get('validFix') or gps.get('fixValid')):
                raise RuntimeError('GPS fix invalid')
        if self.stop_event.is_set():
            raise RuntimeError('Hybrid session stopped')

    def _wait_motion(self, desired, direction, station):
        first=depth_control.get_status(read_position=True)
        if not first.get('success'):
            raise RuntimeError('Encoder unavailable at movement start')
        progress=ProgressWatchdog(direction,time.monotonic(),first.get('cable_payout_m'))
        end=time.monotonic()+MAX_RUN_S
        while time.monotonic()<end:
            self._evidence(station)
            status=depth_control.get_status(read_position=True)
            if not status.get('success') or status.get('safety_fault'):
                raise RuntimeError('Encoder or motor safety fault')
            if not status['active']:
                if status['phase']!=desired:
                    raise RuntimeError('Motion ended unexpectedly: '+str(status['phase']))
                return
            progress.update(time.monotonic(),status.get('cable_payout_m'))
            if self.stop_event.wait(.15):
                raise RuntimeError('Stopped')
        raise RuntimeError('Motor motion timeout')

    def _cycle(self, station):
        ownership.acquire_bench_cycle()
        try:
            with command_as('bench_cycle'):
                self._evidence(station)
                with self.lock:self.phase='deploying'
                depth_control.set_target(min(self.depth_limit,station['targetDepth']))
                self._wait_motion('target_reached',1,station)
                with self.lock:self.phase='sampling'
                end=time.monotonic()+min(self.sample_limit,station['samplingDuration'])
                while time.monotonic()<end:
                    self._evidence(station)
                    if self.stop_event.wait(.1):raise RuntimeError('Stopped')
                with self.lock:self.phase='retracting'
                depth_control.go_home()
                self._wait_motion('home_reached',-1,station)
                with self.lock:self.phase='verifying_home'
                stable=None;deadline=time.monotonic()+8
                while time.monotonic()<deadline:
                    self._evidence(station)
                    status=depth_control.get_status(read_position=True)
                    home=assess_home(status,sampled_at=time.monotonic())
                    velocity=measured_velocity()
                    good=home.get('encoderHomeCandidate') and velocity.get('measuredStoppedCandidate') and velocity.get('measuredVelocityFresh')
                    if not good:stable=None
                    elif stable is None:stable=time.monotonic()
                    elif time.monotonic()-stable>=3:break
                    if self.stop_event.wait(.2):raise RuntimeError('Stopped')
                else:raise RuntimeError('Sustained HOME/stop feedback failed')
        finally:
            try:depth_control.cancel(stop=True)
            finally:ownership.release_bench_cycle()

    def _run(self):
        try:
            for i,station in enumerate(self.stations):
                with self.lock:self.index=i;self.phase='navigating'
                stable=None;wait_deadline=time.monotonic()+900
                while time.monotonic()<wait_deadline:
                    if self.stop_event.is_set():raise RuntimeError('Stopped')
                    mission=mission_monitor.snapshot()
                    if mission.get('currentSequence')==station['loiterSequence']:
                        try:self._evidence(station)
                        except RuntimeError:stable=None
                        else:
                            if stable is None:stable=time.monotonic()
                            if time.monotonic()-stable>=5:break
                    else:stable=None
                    if self.stop_event.wait(.2):raise RuntimeError('Stopped')
                else:raise RuntimeError('Station arrival timed out')
                with self.lock:
                    self.audit_generation=mission_audit.snapshot()['auditGeneration']
                self._cycle(station)
                # Check motor ownership and fresh HOME again before ANY mission advancement.
                if ownership.snapshot()['owner']!='manual':raise RuntimeError('Motor ownership not released')
                self._evidence(station)
                with self.lock:
                    self.completed.append(i)
                    if i==len(self.stations)-1:
                        self.phase='finished';return
                    next_seq=self.stations[i+1]['waypointSequence']
                    self.advance_request=next_seq
                    self.advance_sent_at=None
                    self.phase='advancing'
                deadline=time.monotonic()+12
                while time.monotonic()<deadline:
                    if self.stop_event.is_set():raise RuntimeError('Stopped')
                    mission=mission_monitor.snapshot()
                    if mission['telemetryFresh'] and mission['currentSequence']==next_seq:
                        with self.lock:self.advance_request=None
                        break
                    if self.stop_event.wait(.2):raise RuntimeError('Stopped')
                else:raise RuntimeError('SITL mission advancement unconfirmed')
            with self.lock:self.phase='finished'
        except Exception as exc:
            with self.lock:
                if self.phase!='stopped':self.phase='fault';self.error=str(exc)
                self.advance_request=None
            self.stop_event.set()
            try:depth_control.cancel(stop=True)
            except Exception as stop_exc:
                with self.lock:self.error=str(self.error)+'; STOP FAILED: '+str(stop_exc)

hybrid_sitl=HybridSITL()


