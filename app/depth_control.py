"""Automatic CTD depth and home control.

One shared continuous payout coordinate is maintained by retract_safety:
    0 counts = HOME captured only during INITIALIZE
    positive counts = cable deployed from HOME

Depth targets never modify HOME.  Automatic control may move in either direction.
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
    """Convert motor encoder counts to cable payout.

    The XW540 encoder measures motor revolutions. The spool is driven through
    a 2:1 reduction, so two motor revolutions correspond to one spool revolution.

    A constant full-spool diameter is used for now. This can later be replaced
    by a variable-radius spool model after calibration.
    """
    motor_revolutions = (
        float(deployed_counts) / config.WINCH_COUNTS_PER_REV
    )

    spool_revolutions = (
        motor_revolutions / config.WINCH_GEAR_RATIO
    )

    payout = (
        spool_revolutions
        * math.pi
        * config.SPOOL_FULL_DIAMETER_M
    )

    return payout, spool_revolutions


def _sample():
    """Read motor feedback and return payout/depth from the immutable HOME."""
    with motor_bus.session(state.bus_lock) as bus:
        bus.set_baudrate(config.WINCH_BAUDRATE)
        retract_safety.read_feedback(bus)
    payout, revs = payout_from_counts(state.retract_limit.deployed_counts)
    depth = payout - config.SENSOR_HEIGHT_ABOVE_WATER_M
    return payout, depth, revs


def get_status(read_position=True):
    payout = depth = revs = None
    if state.initialized and state.retract_limit.reference is not None:
        try:
            if read_position:
                payout, depth, revs = _sample()
            else:
                payout, revs = payout_from_counts(state.retract_limit.deployed_counts)
                depth = payout - config.SENSOR_HEIGHT_ABOVE_WATER_M
        except Exception as exc:
            return {'success': False, 'error': str(exc)}

    error = None
    if depth is not None and state.depth.target_m is not None and state.depth.mode == 'depth':
        error = state.depth.target_m - depth

    home_remaining = None
    if payout is not None:
        home_remaining = max(0.0, payout)

    return {
        'success': True,
        'active': state.depth.active,
        'mode': state.depth.mode,
        'target_m': state.depth.target_m,
        'phase': state.depth.phase,
        'command_velocity': state.depth.command_velocity,
        'last_error': state.depth.last_error,
        'home_set': state.retract_limit.reference is not None,
        'home_position_counts': state.retract_limit.reference,
        'raw_position_counts': state.retract_limit.last_position,
        'continuous_deployed_counts': state.retract_limit.deployed_counts,
        'spool_revolutions': revs,
        'cable_payout_m': payout,
        'depth_m': depth,
        'error_m': error,
        'home_remaining_m': home_remaining,
        'above_water': depth is not None and depth < 0,
        'sensor_offset_m': config.SENSOR_HEIGHT_ABOVE_WATER_M,
        'tolerance_m': config.DEPTH_TOLERANCE_M,
        'max_target_depth_m': config.MAX_TARGET_DEPTH_M,
        'model_extrapolated': False,
    }


def _target_velocity(distance):
    for threshold, velocity in config.DEPTH_SPEED_PROFILE:
        if distance <= threshold:
            return velocity
    return config.DEPTH_SPEED_PROFILE[-1][1]


def _set_motion_state(command, velocity, phase):
    state.motion.direction = 1 if command < 0 else -1
    state.depth.phase = phase
    state.depth.command_velocity = velocity
    try:
        state.motion.speed_level = config.SPEED_LEVELS.index(abs(velocity))
    except ValueError:
        state.motion.speed_level = 0


def _run_depth(generation):
    """Bidirectional depth controller. HOME is never changed here."""
    try:
        while not _cancel.wait(config.DEPTH_CONTROL_POLL_INTERVAL):
            if generation != _generation or not state.depth.active:
                return
            control.require_winch_ready()
            payout, depth, _ = _sample()
            error = state.depth.target_m - depth

            if abs(error) <= config.DEPTH_TOLERANCE_M:
                control.execute_stop()
                state.depth.phase = 'target_reached'
                state.depth.command_velocity = 0
                state.depth.active = False
                return

            velocity = _target_velocity(abs(error))
            if error > 0:  # target is deeper -> deploy
                command = -velocity
                phase = 'deploying'
            else:          # target is shallower -> retract
                if payout <= config.STORAGE_CREEP_PAYOUT_M:
                    velocity = min(velocity, config.STORAGE_CREEP_VELOCITY)
                    phase = 'storage_creep'
                elif payout <= config.STORAGE_SLOWDOWN_PAYOUT_M:
                    velocity = min(velocity, config.STORAGE_SLOW_VELOCITY)
                    phase = 'storage_slow'
                else:
                    phase = 'retracting'
                command = velocity

            control.write_velocity(command)
            _set_motion_state(command, velocity, phase)
    except Exception as exc:
        _fault(exc)


def _run_home(generation):
    """Retract toward the HOME captured during initialization."""
    try:
        while not _cancel.wait(config.DEPTH_CONTROL_POLL_INTERVAL):
            if generation != _generation or not state.depth.active:
                return
            control.require_winch_ready()
            payout, _, _ = _sample()

            # The retract safety braking envelope may stop a few cm before the
            # exact encoder zero.  Consider that a safe stored position.
            if state.retract_limit.reached or payout <= config.HOME_TOLERANCE_M:
                control.execute_stop()
                state.depth.phase = 'home_reached'
                state.depth.command_velocity = 0
                state.depth.active = False
                return

            if payout <= config.STORAGE_CREEP_PAYOUT_M:
                velocity = config.STORAGE_CREEP_VELOCITY
                phase = 'homing_creep'
            elif payout <= config.STORAGE_SLOWDOWN_PAYOUT_M:
                velocity = config.STORAGE_SLOW_VELOCITY
                phase = 'homing_slow'
            else:
                velocity = config.HOME_NORMAL_VELOCITY
                phase = 'homing'

            try:
                control.write_velocity(velocity)
            except RuntimeError as exc:
                # Reaching the independent braking envelope is a successful,
                # safe end condition for HOME, not a depth-control fault.
                if state.retract_limit.reached:
                    state.depth.phase = 'home_reached'
                    state.depth.command_velocity = 0
                    state.depth.active = False
                    return
                raise exc
            _set_motion_state(velocity, velocity, phase)
    except Exception as exc:
        _fault(exc)


def _fault(exc):
    state.depth.last_error = str(exc)
    state.depth.phase = 'fault'
    state.depth.active = False
    state.depth.command_velocity = 0
    try:
        control.execute_stop()
    except Exception:
        pass


def _start_worker(mode, target_m=None):
    global _worker, _generation
    control.require_winch_ready()
    cancel(stop=True)
    _generation += 1
    generation = _generation
    _cancel.clear()
    state.depth.mode = mode
    state.depth.target_m = target_m
    state.depth.active = True
    state.depth.phase = 'starting_home' if mode == 'home' else 'starting'
    state.depth.command_velocity = 0
    state.depth.last_error = None
    target = _run_home if mode == 'home' else _run_depth
    _worker = Thread(target=target, args=(generation,), name=f'winch-{mode}-control', daemon=True)
    _worker.start()
    return get_status(read_position=True)


def set_target(depth_m):
    depth_m = float(depth_m)
    if not 0.0 <= depth_m <= config.MAX_TARGET_DEPTH_M:
        raise ValueError(f'Target depth must be between 0 and {config.MAX_TARGET_DEPTH_M:.0f} m')
    return _start_worker('depth', target_m=depth_m)


def go_home():
    """Return to the stored HOME without redefining it."""
    return _start_worker('home', target_m=None)


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
    if state.depth.phase not in ('idle', 'target_reached', 'home_reached', 'fault'):
        state.depth.phase = 'cancelled'
    state.depth.command_velocity = 0
    state.depth.mode = 'idle'
    return get_status(read_position=False)
