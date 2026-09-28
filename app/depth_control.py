"""Automatic CTD depth estimation and target-depth control.

Depth is estimated from the XW540 multi-turn encoder and a continuous winding
model referenced to the measured full-spool diameter at initialization.
This is an estimator, not an independent depth sensor; calibrate before deep use.
"""
import math
import time
from threading import Event, Thread

import config
import control
import motor_bus
from motor_bus import signed_32
from state import state

_worker = None
_cancel = Event()
_generation = 0


def _geometry():
    r_full = config.SPOOL_FULL_DIAMETER_M / 2.0
    r_core = config.SPOOL_CORE_DIAMETER_M / 2.0
    # Continuous close-packed approximation. Radius decreases by this amount/rev.
    radial_drop_per_rev = math.pi * config.CABLE_DIAMETER_M ** 2 / (4.0 * config.SPOOL_USABLE_WIDTH_M)
    max_revs_to_core = max(0.0, (r_full - r_core) / radial_drop_per_rev)
    max_payout_to_core = 2.0 * math.pi * (
        r_full * max_revs_to_core - 0.5 * radial_drop_per_rev * max_revs_to_core ** 2
    )
    return r_full, r_core, radial_drop_per_rev, max_revs_to_core, max_payout_to_core


def payout_from_position(position_counts: int) -> tuple[float, float, bool]:
    """Return (payout_m, spool_revolutions, model_extrapolated)."""
    if state.retract_limit.reference is None:
        raise RuntimeError('Initialize the system to establish the depth reference')
    deployed_counts = state.retract_limit.reference - signed_32(position_counts)
    revolutions = max(0.0, deployed_counts / config.WINCH_COUNTS_PER_REV)
    r_full, r_core, a, max_revs, max_payout = _geometry()
    if revolutions <= max_revs:
        payout = 2.0 * math.pi * (r_full * revolutions - 0.5 * a * revolutions ** 2)
        return max(0.0, payout), revolutions, False
    # Geometry is uncertain beyond the estimated core. Continue conservatively at
    # the core circumference so the UI remains usable, but flag extrapolation.
    payout = max_payout + 2.0 * math.pi * r_core * (revolutions - max_revs)
    return payout, revolutions, True


def _read_position() -> int:
    with motor_bus.session(state.bus_lock) as bus:
        bus.set_baudrate(config.WINCH_BAUDRATE)
        return signed_32(bus.read_position(config.WINCH_ID, action='Read winch depth position'))


def get_status(read_position=True) -> dict:
    position = None
    error = None
    payout = None
    depth = None
    revs = None
    extrapolated = False
    if state.initialized and state.retract_limit.reference is not None and read_position:
        try:
            position = _read_position()
            payout, revs, extrapolated = payout_from_position(position)
            depth = payout - config.SENSOR_HEIGHT_ABOVE_WATER_M
            if state.depth.target_m is not None:
                error = state.depth.target_m - depth
        except Exception as exc:
            return {'success': False, 'error': str(exc)}
    elif state.retract_limit.last_position is not None:
        position = state.retract_limit.last_position
        payout, revs, extrapolated = payout_from_position(position)
        depth = payout - config.SENSOR_HEIGHT_ABOVE_WATER_M
        if state.depth.target_m is not None:
            error = state.depth.target_m - depth

    _, _, _, _, geometric_capacity = _geometry()
    return {
        'success': True,
        'active': state.depth.active,
        'target_m': state.depth.target_m,
        'phase': state.depth.phase,
        'command_velocity': state.depth.command_velocity,
        'last_error': state.depth.last_error,
        'position_counts': position,
        'spool_revolutions': revs,
        'cable_payout_m': payout,
        'depth_m': depth,
        'error_m': error,
        'above_water': depth is not None and depth < 0,
        'model_extrapolated': extrapolated,
        'model_capacity_to_estimated_core_m': geometric_capacity,
        'sensor_offset_m': config.SENSOR_HEIGHT_ABOVE_WATER_M,
        'tolerance_m': config.DEPTH_TOLERANCE_M,
        'max_target_depth_m': config.MAX_TARGET_DEPTH_M,
    }


def _target_velocity(distance_m: float) -> int:
    for threshold, velocity in config.DEPTH_SPEED_PROFILE:
        if distance_m <= threshold:
            return velocity
    return config.DEPTH_SPEED_PROFILE[-1][1]


def _storage_limited_velocity(payout_m: float, velocity: int) -> tuple[int, str]:
    if payout_m <= config.STORAGE_CREEP_PAYOUT_M:
        return min(velocity, config.STORAGE_CREEP_VELOCITY), 'storage_creep'
    if payout_m <= config.STORAGE_SLOWDOWN_PAYOUT_M:
        return min(velocity, config.STORAGE_SLOW_VELOCITY), 'storage_slow'
    return velocity, 'retracting'


def _set_motion_state(direction: int, velocity: int):
    state.motion.direction = direction
    # Closest configured level for UI only.
    try:
        state.motion.speed_level = config.SPEED_LEVELS.index(abs(velocity))
    except ValueError:
        state.motion.speed_level = 0


def _run_target(generation):
    try:
        while not _cancel.wait(config.DEPTH_CONTROL_POLL_INTERVAL):
            if generation != _generation:
                break
            if not state.depth.active or state.depth.target_m is None:
                break
            control.require_winch_ready()
            position = _read_position()
            payout, _, extrapolated = payout_from_position(position)
            depth = payout - config.SENSOR_HEIGHT_ABOVE_WATER_M
            error = state.depth.target_m - depth
            state.depth.last_error = None

            if abs(error) <= config.DEPTH_TOLERANCE_M:
                control.execute_stop()
                state.depth.phase = 'target_reached'
                state.depth.command_velocity = 0
                state.depth.active = False
                break

            direction = 1 if error > 0 else -1  # +1 deploy, -1 retract
            velocity = _target_velocity(abs(error))
            phase = 'deploying' if direction == 1 else 'retracting'
            if direction == -1:
                velocity, phase = _storage_limited_velocity(payout, velocity)

            # control.write_velocity uses positive velocity for retract and negative for deploy.
            signed_velocity = -direction * velocity
            control.write_velocity(signed_velocity)
            _set_motion_state(direction, velocity)
            state.depth.phase = phase
            state.depth.command_velocity = velocity

            # Surface/retract target can be inside the braking envelope. The existing
            # retract safety remains authoritative and may stop before exact zero payout.
            if extrapolated:
                state.depth.phase = phase + '_model_extrapolated'
    except Exception as exc:
        state.depth.last_error = str(exc)
        state.depth.phase = 'fault'
        state.depth.active = False
        state.depth.command_velocity = 0
        try:
            control.execute_stop()
        except Exception:
            pass
    finally:
        if _cancel.is_set() and generation == _generation:
            state.depth.phase = 'cancelled'
            state.depth.active = False
            state.depth.command_velocity = 0


def set_target(depth_m: float) -> dict:
    global _worker, _generation
    depth_m = float(depth_m)
    if not 0.0 <= depth_m <= config.MAX_TARGET_DEPTH_M:
        raise ValueError(f'Target depth must be between 0 and {config.MAX_TARGET_DEPTH_M:.0f} m')
    control.require_winch_ready()
    cancel(stop=False)
    _generation += 1
    generation = _generation
    state.depth.target_m = depth_m
    state.depth.active = True
    state.depth.phase = 'starting'
    state.depth.command_velocity = 0
    state.depth.last_error = None
    _cancel.clear()
    _worker = Thread(target=_run_target, args=(generation,), name='winch-depth-control', daemon=True)
    _worker.start()
    return get_status(read_position=True)


def cancel(stop=True) -> dict:
    global _generation
    _generation += 1
    state.depth.active = False
    _cancel.set()
    if stop and state.initialized:
        try:
            control.execute_stop()
        except Exception:
            pass
    if state.depth.phase not in ('idle', 'target_reached', 'fault'):
        state.depth.phase = 'cancelled'
    state.depth.command_velocity = 0
    return get_status(read_position=False)
