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
