"""Simple automatic CTD depth control.

The XW540 is in velocity mode, where Present Position wraps every revolution.
We therefore use the continuous payout counter maintained by retract_safety
instead of subtracting raw encoder positions.
"""
import math
from threading import Event, Thread

import config
import control
import motor_bus
import retract_safety
from state import state

_worker = None
_cancel = Event()
_generation = 0


def payout_from_counts(deployed_counts):
    """Simple first-test model: constant measured full-spool radius.

    This deliberately avoids the uncertain core/layer model. It is intended for
    short calibration deployments. After measured payout data is available, the
    conversion can be replaced by a calibrated variable-radius model.
    """
    revolutions = max(0.0, float(deployed_counts) / config.WINCH_COUNTS_PER_REV)
    payout = revolutions * math.pi * config.SPOOL_FULL_DIAMETER_M
    return payout, revolutions, False


def _sample():
    """Read one motor sample and update the shared continuous payout counter."""
    with motor_bus.session(state.bus_lock) as bus:
        bus.set_baudrate(config.WINCH_BAUDRATE)
        retract_safety.read_feedback(bus)
    payout, revs, extrapolated = payout_from_counts(state.retract_limit.deployed_counts)
    depth = payout - config.SENSOR_HEIGHT_ABOVE_WATER_M
    return payout, depth, revs, extrapolated


def get_status(read_position=True):
    payout = depth = revs = None
    extrapolated = False
    if state.initialized and state.retract_limit.reference is not None:
        try:
            if read_position:
                payout, depth, revs, extrapolated = _sample()
            else:
                payout, revs, extrapolated = payout_from_counts(state.retract_limit.deployed_counts)
                depth = payout - config.SENSOR_HEIGHT_ABOVE_WATER_M
        except Exception as exc:
            return {'success': False, 'error': str(exc)}
    error = None if depth is None or state.depth.target_m is None else state.depth.target_m - depth
    return {
        'success': True,
        'active': state.depth.active,
        'target_m': state.depth.target_m,
        'phase': state.depth.phase,
        'command_velocity': state.depth.command_velocity,
        'last_error': state.depth.last_error,
        'position_counts': state.retract_limit.last_position,
        'continuous_deployed_counts': state.retract_limit.deployed_counts,
        'spool_revolutions': revs,
        'cable_payout_m': payout,
        'depth_m': depth,
        'error_m': error,
        'above_water': depth is not None and depth < 0,
        'model_extrapolated': extrapolated,
        'sensor_offset_m': config.SENSOR_HEIGHT_ABOVE_WATER_M,
        'tolerance_m': config.DEPTH_TOLERANCE_M,
        'max_target_depth_m': config.MAX_TARGET_DEPTH_M,
    }


def _target_velocity(distance):
    for threshold, velocity in config.DEPTH_SPEED_PROFILE:
        if distance <= threshold:
            return velocity
    return config.DEPTH_SPEED_PROFILE[-1][1]


def _run_target(generation):
    previous_error = None
    try:
        while not _cancel.wait(config.DEPTH_CONTROL_POLL_INTERVAL):
            if generation != _generation or not state.depth.active:
                return
            control.require_winch_ready()
            payout, depth, _, _ = _sample()
            error = state.depth.target_m - depth

            if abs(error) <= config.DEPTH_TOLERANCE_M:
                control.execute_stop()
                state.depth.phase = 'target_reached'
                state.depth.command_velocity = 0
                state.depth.active = False
                return

            # Do not automatically reverse after crossing a target. Stop instead.
            # This prevents a noisy estimate from producing an immediate reversal.
            if previous_error is not None and error * previous_error < 0:
                control.execute_stop()
                state.depth.phase = 'target_reached'
                state.depth.command_velocity = 0
                state.depth.active = False
                return
            previous_error = error

            velocity = _target_velocity(abs(error))
            if error > 0:  # deploy
                command = -velocity
                phase = 'deploying'
                state.motion.direction = 1
            else:          # retract
                if payout <= config.STORAGE_CREEP_PAYOUT_M:
                    velocity = min(velocity, config.STORAGE_CREEP_VELOCITY)
                    phase = 'storage_creep'
                elif payout <= config.STORAGE_SLOWDOWN_PAYOUT_M:
                    velocity = min(velocity, config.STORAGE_SLOW_VELOCITY)
                    phase = 'storage_slow'
                else:
                    phase = 'retracting'
                command = velocity
                state.motion.direction = -1

            control.write_velocity(command)
            state.depth.phase = phase
            state.depth.command_velocity = velocity
            try:
                state.motion.speed_level = config.SPEED_LEVELS.index(abs(velocity))
            except ValueError:
                state.motion.speed_level = 0
    except Exception as exc:
        state.depth.last_error = str(exc)
        state.depth.phase = 'fault'
        state.depth.active = False
        state.depth.command_velocity = 0
        try:
            control.execute_stop()
        except Exception:
            pass


def set_target(depth_m):
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


def cancel(stop=True):
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
