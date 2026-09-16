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


def read_feedback(bus):
    """Read unrounded, signed multi-turn position and actual velocity."""
    position = bus.read_position(config.WINCH_ID, action='Read winch limit position')
    velocity = bus.read_velocity(config.WINCH_ID, action='Read winch limit velocity')
    if state.torque_enabled:
        torque = bus.read_torque(config.WINCH_ID, action='Read winch torque status')
        if torque != config.TORQUE_ENABLE:
            raise RuntimeError('Winch torque was lost; initialize again before moving')
    state.retract_limit.last_position = signed_32(position)
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


def stop(bus):
    # Keep retrying after a failed write; update motion state only after success.
    state.retract_limit.stopping = True
    bus.set_velocity(config.WINCH_ID, 0, action='Set goal velocity')
    state.motion.velocity = 0
    state.motion.direction = 0
    state.motion.speed_level = 0


def record_fault(exc):
    message = str(exc)
    if state.retract_limit.fault != message:
        logger.error('Retract safety fault: %s', message)
    state.retract_limit.fault = message
    state.retract_limit.stopping = True


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
    if state.motion.velocity <= 0 and (not state.retract_limit.stopping):
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
                stop(bus)
            elif (
                state.motion.velocity > 0
                and state.retract_limit.reference - position
                <= stopping_counts(max(state.motion.velocity, measured_velocity))
            ):
                state.retract_limit.reached = True
                stop(bus)
            # Goal velocity zero does not mean physical deceleration is finished.
            if state.motion.velocity <= 0 and measured_velocity <= 0:
                state.retract_limit.stopping = False
        except Exception as exc:
            record_fault(exc)
            stop(bus)
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
        stop(bus)
        raise
    if state.retract_limit.reference - position <= braking_distance:
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
    state.retract_limit.reference = signed_32(reference)
    state.retract_limit.last_position = state.retract_limit.reference
    state.retract_limit.reached = True
    state.retract_limit.fault = None
    state.retract_limit.stopping = False
    start_monitor()
