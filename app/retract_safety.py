"""
Retract reference, braking policy, and monitoring.
Normal commands use validate_retraction(). 
The unlock relief exception stays explicit in control.py. 
This module never imports control to avoid a cycle.
"""
from threading import Event, Thread
import logging
import math

import config
import motor_bus
from motor_bus import signed_32
from state import state, serialized_control

_monitor_thread = None
_shutdown = Event()
logger = logging.getLogger(__name__)


def encoder_delta(new_raw, old_raw):
    """Shortest signed delta for the XW540 single-turn 0..4095 position.

    In velocity mode Present Position wraps once per revolution.  At our maximum
    configured speed and polling rate the motor cannot move half a revolution
    between samples, so the shortest modular delta is unambiguous.
    """
    counts = config.WINCH_COUNTS_PER_REV
    delta = int(new_raw) - int(old_raw)
    half = counts // 2
    if delta > half:
        delta -= counts
    elif delta < -half:
        delta += counts
    return delta


def update_position(raw_position):
    """Update continuous cable payout from one raw encoder sample."""
    raw = int(raw_position) % config.WINCH_COUNTS_PER_REV
    previous = state.retract_limit.raw_position
    if previous is None:
        state.retract_limit.raw_position = raw
        state.retract_limit.last_position = raw
        return 0
    delta = encoder_delta(raw, previous)
    if abs(delta) > config.ENCODER_MAX_DELTA_COUNTS:
        raise RuntimeError(f'Implausible encoder jump: {delta} counts')
    # Positive motor rotation is RETRACT, negative rotation is DEPLOY.
    # Therefore payout grows when encoder motion is negative.
    state.retract_limit.deployed_counts -= delta
    state.retract_limit.raw_position = raw
    state.retract_limit.last_position = raw
    return delta


def read_feedback(bus):
    """Read feedback and maintain a continuous payout coordinate."""
    position = bus.read_position(config.WINCH_ID, action='Read winch limit position')
    velocity = bus.read_velocity(config.WINCH_ID, action='Read winch limit velocity')
    if state.torque_enabled:
        torque = bus.read_torque(config.WINCH_ID, action='Read winch torque status')
        if torque != config.TORQUE_ENABLE:
            raise RuntimeError('Winch torque was lost; initialize again before moving')
    update_position(position)
    return (state.retract_limit.last_position, signed_32(velocity))


def stopping_counts(velocity):
    """
    Conservative braking envelope for the configured velocity-based profile.
    ROBOTIS specifies 0.229 rpm per velocity unit and 214.577 rpm/min
    per acceleration unit. 
    This is not a hardware limit switch and relies on software only.
    """
    if (
        config.RETRACT_LIMIT_POLL_INTERVAL <= 0
        or config.RETRACT_LIMIT_REACTION_TIME < 0
        or config.RETRACT_LIMIT_BRAKING_FACTOR < 1
        or config.RETRACT_LIMIT_MARGIN_DEG < 0
    ):
        raise RuntimeError("Invalid retract limit braking configuration")
    acceleration = (
        config.WINCH_PROFILE_ACCELERATION
        * config.DYNAMIXEL_ACCEL_RPM_PER_MIN_PER_UNIT
        * config.WINCH_COUNTS_PER_REV / 3600.0
    )
    if acceleration <= 0:
        raise RuntimeError("Retract limit requires a positive profile acceleration")
    speed = (
        max(0, velocity) * config.DYNAMIXEL_RPM_PER_UNIT
        * config.WINCH_COUNTS_PER_REV / 60.0
    )
    reaction_time = max(
        config.RETRACT_LIMIT_REACTION_TIME, config.RETRACT_LIMIT_POLL_INTERVAL
    )
    return math.ceil(
        config.RETRACT_LIMIT_BRAKING_FACTOR * speed * speed / (2.0 * acceleration)
        + speed * reaction_time
        + config.RETRACT_LIMIT_MARGIN_DEG / config.POSITION_DEG_PER_COUNT
    )


def engage_mechanical_lock(bus):
    """Fail-safe action: release XW430 torque so the spring engages the pawl.

    This is intentionally independent of control.py so it can be used from the
    safety monitor without creating an import cycle.
    """
    bus.set_baudrate(config.LOCK_BAUDRATE)
    bus.set_torque(
        config.LOCK_ID,
        config.TORQUE_DISABLE,
        action='Engage mechanical lock after winch fault',
    )
    state.lock_state = 'locked'


def stop_and_lock(bus):
    """Best-effort emergency response: stop XW540 and engage the passive pawl.

    The lock attempt is made even when the winch stop write fails. The first
    exception is re-raised after both actions have been attempted.
    """
    first_error = None
    try:
        stop(bus)
    except Exception as exc:
        first_error = exc
    try:
        engage_mechanical_lock(bus)
    except Exception as exc:
        if first_error is None:
            first_error = exc
    state.motion.velocity = 0
    state.motion.direction = 0
    state.motion.speed_level = 0
    if first_error is not None:
        raise first_error


def stop(bus):
    # Keep retrying after a failed write; update motion state only after success.
    state.retract_limit.stopping = True
    bus.set_velocity(config.WINCH_ID, 0, action='Set goal velocity')
    state.motion.velocity = 0
    state.motion.direction = 0
    state.motion.speed_level = 0


def record_fault(exc):
    """Latch the first safety fault and make commanded motion state conservative.

    A secondary failure while trying to stop must not overwrite the original cause.
    """
    message = str(exc)
    if state.retract_limit.fault is None:
        logger.error('Retract safety fault: %s', message)
        state.retract_limit.fault = message
    elif state.retract_limit.fault != message:
        logger.error('Additional retract safety error: %s', message)
    state.retract_limit.stopping = True
    # Once feedback/control is faulted, the last command is not trustworthy.
    state.motion.velocity = 0
    state.motion.direction = 0
    state.motion.speed_level = 0


def fault_is_recoverable():
    """Return True for faults that indicate a responding Dynamixel shutdown.

    These cases can be rebooted and HOME preserved only after the encoder is checked
    for continuity. Communication loss and encoder-integrity faults remain latched
    and require normal reinitialization.
    """
    fault = state.retract_limit.fault or ''
    return (
        fault.startswith('Winch torque was lost')
        or ('[RxPacketError]' in fault and 'Hardware error occurred' in fault)
    )


def clear_recoverable_fault():
    """Clear only a recoverable torque-loss fault, preserving HOME tracking."""
    if state.retract_limit.fault is None:
        raise RuntimeError('No retract safety fault is active')
    if not fault_is_recoverable():
        raise RuntimeError(
            'This safety fault cannot be reset remotely; reinitialize the winch',
        )
    state.retract_limit.fault = None
    state.retract_limit.stopping = False
    state.retract_limit.reached = False


def require_reference():
    if not state.initialized or state.retract_limit.reference is None:
        raise RuntimeError('Initialize the system to establish the retract limit')
    if state.retract_limit.fault is not None:
        raise RuntimeError(
            f'Retract safety fault: {state.retract_limit.fault}; initialize again',
        )
    if _monitor_thread is None or not _monitor_thread.is_alive():
        raise RuntimeError('Retract safety monitor is not running; initialize again')


@serialized_control
def check():
    """One monitor cycle, also callable in hardware-free regression tests."""
    if not state.initialized or not state.torque_enabled:
        return
    # Keep sampling during BOTH deploy and retract so the continuous HOME-based
    # coordinate cannot lose turns. Only the retract direction is limit-guarded.
    if state.motion.velocity == 0 and (not state.retract_limit.stopping):
        return
    bus = None
    try:
        bus = motor_bus.open_bus()
        bus.set_baudrate(config.WINCH_BAUDRATE)
        try:
            position, measured_velocity = read_feedback(bus)
            if state.retract_limit.reference is None:
                raise RuntimeError('Retract reference is missing')
            if state.retract_limit.fault is not None:
                stop_and_lock(bus)
            elif state.motion.velocity > 0:
                braking_distance = stopping_counts(max(state.motion.velocity, measured_velocity))
                if state.retract_limit.deployed_counts <= braking_distance:
                    state.retract_limit.reached = True
                    stop(bus)
            # Goal velocity zero does not mean physical deceleration is finished.
            if state.motion.velocity == 0 and measured_velocity == 0:
                state.retract_limit.stopping = False
        except Exception as exc:
            record_fault(exc)
            try:
                stop_and_lock(bus)
            except Exception as stop_exc:
                record_fault(stop_exc)
    except Exception as exc:
        record_fault(exc)
    finally:
        if bus is not None:
            bus.close()


def _monitor_loop():
    while not _shutdown.wait(config.RETRACT_LIMIT_POLL_INTERVAL):
        try:
            check()
        except Exception as exc:
            with state.bus_lock:
                record_fault(exc)


def start_monitor():
    global _monitor_thread
    if _monitor_thread is not None and _monitor_thread.is_alive():
        return
    _shutdown.clear()
    _monitor_thread = Thread(target=_monitor_loop, name='winch-retract-limit', daemon=True)
    _monitor_thread.start()


def validate_retraction(bus, velocity):
    """Reject unsafe positive velocity commands and attempt a stop on failure."""
    try:
        position, measured_velocity = read_feedback(bus)
        braking_distance = stopping_counts(max(velocity, state.motion.velocity, measured_velocity))
    except Exception as exc:
        record_fault(exc)
        try:
            stop_and_lock(bus)
        except Exception as stop_exc:
            record_fault(stop_exc)
        raise
    if state.retract_limit.deployed_counts <= braking_distance:
        state.retract_limit.reached = True
        stop(bus)
        raise RuntimeError('Retract limit reached (including braking allowance)')


def capture_reference(bus):
    """Capture the stationary multi-turn reference after motor setup, then start monitoring."""
    bus.set_baudrate(config.WINCH_BAUDRATE)
    drive_mode = bus.read_drive_mode(config.WINCH_ID, action='Read winch drive mode')
    if drive_mode & 4:
        raise RuntimeError('Retract limit requires a velocity-based profile')
    stopping_counts(0)
    reference = bus.read_position(config.WINCH_ID, action='Capture winch retract limit')
    actual_velocity = bus.read_velocity(config.WINCH_ID, action='Check winch is stationary')
    if signed_32(actual_velocity) != 0:
        raise RuntimeError('Winch must be stationary when setting the retract limit')
    raw_reference = int(reference) % config.WINCH_COUNTS_PER_REV
    state.retract_limit.reference = raw_reference
    state.retract_limit.raw_position = raw_reference
    state.retract_limit.last_position = raw_reference
    state.retract_limit.deployed_counts = 0
    state.retract_limit.reached = True
    state.retract_limit.fault = None
    state.retract_limit.stopping = False
    start_monitor()
