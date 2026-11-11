"""Explicit operator-requested bench movement; never triggered by CTD mission.

Not autonomous. Intended only for secured, attended, low-depth commissioning.
"""
import os
import math
import time
from threading import RLock, Thread, Event
import depth_control
from state import state

MAX_DEPTH_M = 0.5
MAX_RUN_S = 20.0
_lock = RLock()
_generation = 0
_status = {'phase': 'idle', 'note': 'Operator-supervised only'}


def snapshot():
    with _lock:
        return {**_status, 'automaticWinchEnabled': False,
                'automaticAdvanceEnabled': False, 'maxTargetDepthMeters': MAX_DEPTH_M,
                'maxMotionSeconds': MAX_RUN_S}


def _watch(generation, deadline):
    global _status
    while time.monotonic() < deadline:
        time.sleep(0.1)
        with _lock:
            if generation != _generation:
                return
            if not state.depth.active:
                _status = {'phase': 'finished', 'depthPhase': state.depth.phase,
                           'depthError': state.depth.last_error}
                return
    with _lock:
        if generation != _generation:
            return
        # Hold lock during cancel to prevent another supervised request starting.
        try:
            depth_control.cancel(stop=True)
            _status = {'phase': 'timed_out_stopped'}
        except Exception as exc:
            _status = {'phase': 'stop_failed', 'error': str(exc)}


def request(depth_m, *, token, confirmation):
    global _generation, _status
    secret = os.environ.get('CTD_BENCH_MOTION_TOKEN', '')
    if not secret or not isinstance(token, str):
        raise PermissionError('Bench movement disabled: configure CTD_BENCH_MOTION_TOKEN')
    import hmac
    if not hmac.compare_digest(token, secret):
        raise PermissionError('Invalid bench movement token')
    if confirmation != 'WINCH_CLEAR_AND_ATTENDED':
        raise ValueError('Explicit attended-motion confirmation required')
    if isinstance(depth_m, bool) or not isinstance(depth_m, (float, int)) or not math.isfinite(depth_m):
        raise ValueError('Invalid depth')
    if not 0 <= depth_m <= MAX_DEPTH_M:
        raise ValueError('Bench target depth outside permitted range')
    with _lock:
        if state.depth.active:
            raise RuntimeError('Depth controller already active')
        if state.home_adjust_mode or state.home_recovery_mode:
            raise RuntimeError('HOME adjustment/recovery active')
        if not state.initialized or state.retract_limit.reference is None:
            raise RuntimeError('Calibrated HOME required')
        if state.retract_limit.fault:
            raise RuntimeError('Motor safety fault')
        # The existing depth worker retains its normal safety checks.
        result = depth_control.set_target(float(depth_m))
        _generation += 1
        generation = _generation
        _status = {'phase': 'moving', 'requestedDepthMeters': depth_m,
                   'startedMonotonic': time.monotonic()}
        Thread(target=_watch, args=(generation, time.monotonic() + MAX_RUN_S),
               daemon=True, name='ctd-bench-motion-watchdog').start()
        return {'accepted': True, 'depthController': result, **snapshot()}


def stop():
    global _generation, _status
    with _lock:
        _generation += 1
        try:
            result = depth_control.cancel(stop=True)
            _status = {'phase': 'operator_stopped'}
            return {'success': True, 'depthController': result, **snapshot()}
        except Exception as exc:
            _status = {'phase': 'stop_failed', 'error': str(exc)}
            return {'success': False, **snapshot()}


# GPS-triggered, operator-armed real-motor commissioning cycle.
# This deliberately does not transmit MAVLink mission-advance commands.
from ctd_command_ownership import ownership, command_as
from ctd_measured_velocity import read as measured_velocity
from ctd_encoder_feedback import assess as assess_home

_cycle_lock = RLock()
_cycle_generation = 0
_cycle_stop = Event()
_cycle = {'phase': 'idle', 'motorCommandsIssued': False,
          'automaticAdvanceEnabled': False}


def cycle_snapshot():
    with _cycle_lock:
        return dict(_cycle)


def _cycle_set(phase, **values):
    with _cycle_lock:
        _cycle.update(phase=phase, **values)


def cycle_arm(*, token, confirmation, station, mission, audit,
              test_depth_m=0.2, sample_seconds=5, rehearsal=None):
    global _cycle_generation, _cycle
    secret = os.environ.get('CTD_BENCH_MOTION_TOKEN', '')
    import hmac
    if not secret or not isinstance(token, str) or not hmac.compare_digest(token, secret):
        raise PermissionError('Valid CTD_BENCH_MOTION_TOKEN required')
    if confirmation != 'WINCH_CLEAR_AND_ATTENDED':
        raise ValueError('Operator must confirm attended and clear winch')
    if not isinstance(test_depth_m, (int,float)) or isinstance(test_depth_m, bool) or not math.isfinite(test_depth_m) or not 0 <= test_depth_m <= MAX_DEPTH_M:
        raise ValueError('Commissioning depth must be 0 to 0.5 m')
    if type(sample_seconds) is not int or not 1 <= sample_seconds <= 15:
        raise ValueError('Commissioning sampling duration must be 1 to 15 s')
    if not audit.get('missionValidated') or not mission.get('telemetryFresh'):
        raise ValueError('Real ArduPilot mission validation and fresh telemetry required')
    if rehearsal is None or rehearsal.get('station') != station:
        raise ValueError('Matching indoor rehearsal required')
    if rehearsal.get('virtualSequence') not in (station['waypointSequence'], station['loiterSequence']):
        raise ValueError('Virtual navigation is not at selected station')
    if rehearsal.get('stationIndex') in rehearsal.get('completedStationIndices', []):
        raise ValueError('Station already completed')
    if not state.initialized or state.retract_limit.reference is None or state.retract_limit.fault:
        raise RuntimeError('Winch not initialized, calibrated or fault-free')
    if state.depth.active or state.motion.velocity != 0 or state.home_adjust_mode or state.home_recovery_mode:
        raise RuntimeError('Winch is busy')
    if not state.torque_enabled or state.lock_state != 'unlocked':
        raise RuntimeError('Winch torque must be enabled and lock released')
    # This is a physical motor test. An encoder-based HOME check is necessary,
    # but never represents independent confirmation that the CTD is stowed.
    home_status = depth_control.get_status(read_position=True)
    home_check = assess_home(home_status, sampled_at=time.monotonic())
    if not home_check.get('encoderHomeCandidate'):
        raise RuntimeError('Fresh encoder feedback within HOME tolerance required before arming')
    if rehearsal.get('rehearsal', {}).get('phase') == 'fault':
        raise RuntimeError('Rehearsal is faulted; reset before arming')
    with _cycle_lock:
        if _cycle['phase'] not in ('idle', 'complete', 'fault', 'stopped'):
            raise RuntimeError('Existing cycle must finish or be stopped')
        if ownership.snapshot()['owner'] != 'manual':
            raise RuntimeError('Motor is owned by another controller')
        _cycle_generation += 1
        _cycle_stop.clear()
        _cycle = {'phase':'armed_waiting_for_simulated_arrival',
                  'generation':_cycle_generation, 'stationSequence':station['loiterSequence'],
                  'testDepthMeters':float(test_depth_m), 'configuredStationDepthMeters':station['targetDepth'],
                  'samplingSeconds':sample_seconds,
                  'stationLatitude':station['latitude'],
                  'stationLongitude':station['longitude'],
                  'stationIndex':rehearsal['stationIndex'],
                  'motorReleased':False,
                  'motorCommandsIssued':False,
                  'navigationEvidence':'terminal_simulated_gps_only',
                  'realArduPilotSequenceRequired':False,
                  'homePhysicallyVerified':False,
                  'missionContinuationAuthorized':False,
                  'automaticAdvanceEnabled':False, 'error':None}
        return dict(_cycle)


def cycle_arrival(*, rehearsal, mission, audit):
    """Called only from terminal GPS injection route; triggers at sustained arrival."""
    with _cycle_lock:
        if _cycle['phase'] != 'armed_waiting_for_simulated_arrival':
            return dict(_cycle)
        if rehearsal.get('rehearsal',{}).get('phase') != 'descending':
            return dict(_cycle)
        gps = rehearsal.get('simulatedGPS')
        if (not gps or gps.get('ageSeconds', float('inf')) > 2.0
                or gps.get('ageSeconds', -1) < 0):
            return dict(_cycle)
        lat1, lon1 = map(math.radians, (gps['latitude'], gps['longitude']))
        lat2, lon2 = map(math.radians, (_cycle['stationLatitude'], _cycle['stationLongitude']))
        a = math.sin((lat2-lat1)/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin((lon2-lon1)/2)**2
        if 12742000*math.asin(min(1.0, math.sqrt(a))) > 3.0 or gps['speedMps'] > 0.3:
            return dict(_cycle)
        if (not mission.get('telemetryFresh') or not audit.get('missionValidated')
                or rehearsal.get('virtualSequence') != _cycle['stationSequence']
                or rehearsal.get('stationIndex') != _cycle['stationIndex']):
            _cycle.update(phase='fault',error='ArduPilot mission evidence lost before start')
            return dict(_cycle)
        if not state.torque_enabled or state.lock_state != 'unlocked' or state.retract_limit.fault:
            _cycle.update(phase='fault', error='Winch readiness lost before deployment')
            return dict(_cycle)
        ownership.acquire_bench_cycle()
        generation = _cycle['generation']
        _cycle['phase'] = 'deploying'
        _cycle['motorCommandsIssued'] = True
        Thread(target=_cycle_worker,args=(generation,_cycle['stationSequence']),
               name='ctd-attended-hybrid-cycle',daemon=True).start()
        return dict(_cycle)


def _cycle_worker(generation, sequence):
    def check():
        from state import mission_monitor
        from mission_audit import mission_audit
        from ctd_terminal_rehearsal import terminal_rehearsal
        mission = mission_monitor.snapshot()
        audit = mission_audit.snapshot()
        gps = terminal_rehearsal.snapshot().get('simulatedGPS')
        if (not mission.get('telemetryFresh') or not audit.get('missionValidated')
                or terminal_rehearsal.snapshot().get('virtualSequence') != sequence
                or terminal_rehearsal.snapshot().get('stationIndex') != cycle_snapshot().get('stationIndex')):
            raise RuntimeError('ArduPilot mission/telemetry evidence lost')
        if (not gps or not 0 <= gps.get('ageSeconds', 1e6) <= 2.0):
            raise RuntimeError('Terminal GPS injection lost')
        cfg = cycle_snapshot()
        lat1, lon1 = map(math.radians, (gps['latitude'], gps['longitude']))
        lat2, lon2 = map(math.radians, (cfg['stationLatitude'], cfg['stationLongitude']))
        a = math.sin((lat2-lat1)/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin((lon2-lon1)/2)**2
        if 12742000*math.asin(min(1.0, math.sqrt(a))) > 3.0 or gps['speedMps'] > 0.3:
            raise RuntimeError('Simulated station keeping lost')
        if _cycle_stop.is_set() or generation != _cycle_generation:
            raise RuntimeError('Operator stopped the cycle')
        if (not state.initialized or not state.torque_enabled
                or state.lock_state != 'unlocked' or state.retract_limit.fault):
            raise RuntimeError('Winch readiness lost during cycle')
        # Mission state is checked by the monitoring loop below, not trusted from GPS alone.
    try:
        with command_as('bench_cycle'):
            with _cycle_lock:
                check()
                target=cycle_snapshot()['testDepthMeters']
                depth_control.set_target(target)
            deadline=time.monotonic()+MAX_RUN_S
            while time.monotonic() < deadline:
                check()
                status=depth_control.get_status(read_position=True)
                if not status.get('success') or status.get('safety_fault'):
                    raise RuntimeError('Depth feedback or motor safety fault')
                if not status['active']:
                    if status.get('phase') != 'target_reached':
                        raise RuntimeError('Deployment ended without reaching target: '+str(status.get('phase')))
                    break
                time.sleep(0.15)
            else:
                raise RuntimeError('Deployment timed out')
            check()
            _cycle_set('sampling')
            end=time.monotonic()+cycle_snapshot()['samplingSeconds']
            while time.monotonic() < end:
                check()
                if state.retract_limit.fault:
                    raise RuntimeError('Motor safety fault during sampling')
                time.sleep(0.1)
            check()
            with _cycle_lock:
                check()
                _cycle_set('retracting')
                depth_control.go_home()
            deadline=time.monotonic()+MAX_RUN_S
            while time.monotonic() < deadline:
                check()
                status=depth_control.get_status(read_position=True)
                if not status.get('success') or status.get('safety_fault'):
                    raise RuntimeError('HOME feedback or motor safety fault')
                if not status['active']:
                    if status.get('phase') != 'home_reached':
                        raise RuntimeError('Retraction ended without HOME: '+str(status.get('phase')))
                    break
                time.sleep(0.15)
            else:
                raise RuntimeError('Retraction timed out')
            check()
            _cycle_set('verifying_home')
            first=None
            deadline=time.monotonic()+6.0
            while time.monotonic() < deadline:
                check()
                status=depth_control.get_status(read_position=True)
                home=assess_home(status,sampled_at=time.monotonic())
                velocity=measured_velocity()
                good=(home.get('encoderHomeCandidate') and
                      velocity.get('measuredStoppedCandidate') and
                      velocity.get('measuredVelocityFresh'))
                if not good:
                    first=None
                elif first is None:
                    first=time.monotonic()
                elif time.monotonic()-first>=3.0:
                    _cycle_set('home_confirmed_pending_stop',homeEncoderCandidate=True,
                               homePhysicallyVerified=False,
                               missionContinuationAuthorized=False)
                    return
                time.sleep(0.25)
            raise RuntimeError('Sustained encoder HOME / measured stop verification failed')
    except Exception as exc:
        if not _cycle_stop.is_set():
            _cycle_set('fault',error=str(exc))
    finally:
        try:
            depth_control.cancel(stop=True)
        except Exception as exc:
            _cycle_set('fault', error='Final motor stop failed: ' + str(exc))
        finally:
            ownership.release_bench_cycle()
            with _cycle_lock:
                _cycle['motorReleased'] = True
                if (_cycle['phase'] == 'home_confirmed_pending_stop'
                        and generation == _cycle_generation and not _cycle_stop.is_set()):
                    _cycle['phase'] = 'complete'
                _cycle_stop.set()


def cycle_stop():
    global _cycle_generation
    with _cycle_lock:
        _cycle_generation += 1
        _cycle_stop.set()
        if _cycle['phase'] not in ('complete', 'fault'):
            _cycle['phase']='stopped'
    # STOP is always authorized; do not release motor ownership until worker exits.
    depth_control.cancel(stop=True)
    return cycle_snapshot()
