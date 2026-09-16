"""
Motor commands and sequences.
Underlying hardware operations are in motor_bus, shared state in state, and normal
retraction checks/background monitoring in retract_safety.
"""
import time

import config
import motor_bus
import retract_safety
from motor_bus import signed_16, signed_32
from state import state, serialized_control


def read_all_telemetry():
    """Read telemetry from both Dynamixels."""
    with motor_bus.session(state.bus_lock) as bus:
        winch = bus.read_telemetry(config.WINCH_ID, config.WINCH_BAUDRATE)
        lock = bus.read_telemetry(config.LOCK_ID, config.LOCK_BAUDRATE)
    return {'success': True, 'winch': winch, 'lock': lock}


def _send_winch_velocity(bus, velocity):
    """Write a goal on an already locked/open winch bus; update on success only."""
    bus.set_velocity(config.WINCH_ID, velocity, action='Set goal velocity')
    state.motion.velocity = velocity


@serialized_control
def write_velocity(velocity: int):
    """Command motor velocity, guarding every positive (retract) command.

    Unlock load relief has a separate, bounded loop; it cannot be requested
    through this function, HTTP movement commands, or MAVLink commands.
    """
    if velocity != 0:
        retract_safety.require_reference()
        if not state.torque_enabled:
            raise RuntimeError('Motor torque is not enabled')
        if state.lock_state != 'unlocked':
            raise RuntimeError('Mechanical lock is engaged')
    with motor_bus.session(state.bus_lock) as bus:
        bus.set_baudrate(config.WINCH_BAUDRATE)
        if velocity > 0:
            retract_safety.validate_retraction(bus, velocity)
        previous_velocity = state.motion.velocity
        try:
            _send_winch_velocity(bus, velocity)
        except Exception as exc:
            retract_safety.record_fault(exc)
            retract_safety.stop(bus)
            raise
        if velocity <= 0:
            state.retract_limit.stopping = state.retract_limit.stopping or previous_velocity > 0
        if velocity != 0:
            state.retract_limit.reached = False


def get_motor_state() -> dict:
    """Return the current application-side winch state."""
    rpm = abs(state.motion.velocity) * config.DYNAMIXEL_RPM_PER_UNIT
    if state.motion.direction == -1:
        status_text = 'retracting'
    elif state.motion.direction == 1:
        status_text = 'deploying'
    else:
        status_text = 'stopped'
    return {
        'success': True,
        'initialized': state.initialized,
        'torque_enabled': state.torque_enabled,
        'lock_state': state.lock_state,
        'status': status_text,
        'direction': state.motion.direction,
        'speed_level': state.motion.speed_level + 1 if state.motion.direction != 0 else 0,
        'max_speed_level': len(config.SPEED_LEVELS),
        'velocity': state.motion.velocity,
        'rpm': round(rpm, 1),
        'retract_limit': {
            'position_counts': state.retract_limit.reference,
            'last_position_counts': state.retract_limit.last_position,
            'reached': state.retract_limit.reached,
            'fault': state.retract_limit.fault,
            'stopping': state.retract_limit.stopping,
            'unlock_allowance_deg': config.UNLOCK_RELIEF_MOTOR_DEG,
        },
        'unlock_diagnostics': {
            'relief_motor_deg': config.UNLOCK_RELIEF_MOTOR_DEG,
            'relief_ratchet_deg': config.UNLOCK_RELIEF_MOTOR_DEG / 2.0,
            'lock_position_deg': state.unlock.position_deg,
            'lock_current_a': state.unlock.current_a,
            'position_error_deg': state.unlock.position_error_deg,
            'success': state.unlock.success,
        },
    }


def require_winch_ready():
    """Prevent movement unless the system is ready and unlocked."""
    if not state.initialized:
        raise RuntimeError('System is not initialized')
    if not state.torque_enabled:
        raise RuntimeError('Motor torque is not enabled')
    if state.lock_state != 'unlocked':
        raise RuntimeError('Mechanical lock is engaged')
    retract_safety.require_reference()


@serialized_control
def relieve_pawl_load():
    """
    Rotate the XW540 slightly in the RETRACT direction before
    lifting the pawl.

    The winch must already have torque enabled. The movement is
    measured using Present Position instead of a fixed sleep.
    """
    if not state.initialized:
        raise RuntimeError('Initialize the system first')
    if not state.torque_enabled:
        raise RuntimeError('Enable winch torque before unlocking the mechanism')
    retract_safety.require_reference()
    if state.lock_state != 'unlocking':
        raise RuntimeError('Load relief is only allowed during unlocking')
    execute_stop()
    with motor_bus.session(state.bus_lock) as bus:
        motion_started = False
        try:
            bus.set_baudrate(config.WINCH_BAUDRATE)
            start_position = bus.read_position(config.WINCH_ID, action='Read winch start position')
            relief_ceiling = state.retract_limit.reference + config.UNLOCK_RELIEF_COUNTS
            if signed_32(start_position) + config.UNLOCK_RELIEF_COUNTS > relief_ceiling:
                raise RuntimeError(
                    'Unlock relief would exceed its allowance beyond the retract limit',
                )
            command = config.UNLOCK_RELIEF_VELOCITY
            # An unacknowledged command may still have started the motor.
            motion_started = True
            bus.set_velocity(
                config.WINCH_ID,
                command,
                action='Start ratchet load-relief movement',
            )
            state.motion.velocity = config.UNLOCK_RELIEF_VELOCITY
            deadline = time.monotonic() + config.UNLOCK_RELIEF_TIMEOUT
            while True:
                position = bus.read_position(config.WINCH_ID, action='Read winch relief position')
                start_position_signed = signed_32(start_position)
                position_signed = signed_32(position)
                movement = position_signed - start_position_signed
                if movement >= config.UNLOCK_RELIEF_COUNTS:
                    break
                if time.monotonic() >= deadline:
                    movement_deg = movement * config.POSITION_DEG_PER_COUNT
                    raise RuntimeError(
                        f'Ratchet load-relief movement timed out: moved {movement_deg:.1f} deg, target {config.UNLOCK_RELIEF_MOTOR_DEG:.1f} deg',
                    )
                time.sleep(config.UNLOCK_RELIEF_POLL_INTERVAL)
        finally:
            if motion_started:
                try:
                    _send_winch_velocity(bus, 0)
                except Exception as exc:
                    retract_safety.record_fault(exc)
                    raise


@serialized_control
def initialize_lock_motor(bus):
    """
    Configure the XW430 in Position Control Mode.

    Initialization finishes with torque OFF so the spring keeps
    the ratchet mechanically locked.
    """
    bus.set_baudrate(config.LOCK_BAUDRATE)
    bus.set_torque(
        config.LOCK_ID,
        config.TORQUE_DISABLE,
        action='Disable lock motor torque',
    )
    bus.set_operating_mode(
        config.LOCK_ID,
        config.POSITION_MODE,
        action='Set lock motor position mode',
    )
    bus.set_profile_acceleration(
        config.LOCK_ID,
        config.LOCK_PROFILE_ACCELERATION,
        action='Set lock motor profile acceleration',
    )
    bus.set_profile_velocity(
        config.LOCK_ID,
        config.LOCK_PROFILE_VELOCITY,
        action='Set lock motor profile velocity',
    )
    state.lock_state = 'locked'


def verify_unlock():
    """
    Verify that the XW430 reached the unlock position.

    The latest measured position, current and position error are stored
    in the application diagnostics on every poll, including failed
    unlock attempts.

    Returns (position_deg, current_a, error_deg).
    Raises RuntimeError if the target is not reached in time.
    """
    deadline = time.monotonic() + config.UNLOCK_VERIFY_TIMEOUT
    last_position_deg = None
    last_current_a = None
    last_error_deg = None
    while time.monotonic() < deadline:
        with motor_bus.session(state.bus_lock) as bus:
            bus.set_baudrate(config.LOCK_BAUDRATE)
            raw_position = bus.read_position(config.LOCK_ID, action='Read lock motor position')
            raw_current = bus.read_current(config.LOCK_ID, action='Read lock motor current')
        last_position_deg = signed_32(raw_position) * config.POSITION_DEG_PER_COUNT
        last_current_a = signed_16(raw_current) * config.PRESENT_CURRENT_MA_PER_UNIT / 1000.0
        last_error_deg = abs(config.UNLOCK_POSITION_DEG - last_position_deg)
        state.unlock.position_deg = round(last_position_deg, 1)
        state.unlock.current_a = round(last_current_a, 3)
        state.unlock.position_error_deg = round(last_error_deg, 1)
        if last_error_deg <= config.UNLOCK_POSITION_TOLERANCE_DEG:
            return (state.unlock.position_deg, state.unlock.current_a, state.unlock.position_error_deg)
        time.sleep(config.UNLOCK_VERIFY_POLL_INTERVAL)
    if last_position_deg is None:
        raise RuntimeError('Unlock failed: no lock motor position measurement was obtained')
    raise RuntimeError(
        f'Unlock failed: lock motor did not reach {config.UNLOCK_POSITION_DEG:.1f} deg (last position {state.unlock.position_deg:.1f} deg, error {state.unlock.position_error_deg:.1f} deg, current {state.unlock.current_a:.3f} A)',
    )


@serialized_control
def unlock_mechanism():
    """
    Unlock sequence:

    1. Require XW540 torque to already be enabled.
    2. Rotate XW540 6 degrees in RETRACT to unload the pawl.
    3. Stop and wait briefly for the ratchet to settle.
    4. Enable XW430 torque.
    5. Command the pawl to 205 degrees.
    6. Verify the XW430 actually reaches the unlock region.
    """
    if not state.initialized:
        raise RuntimeError('Initialize the system first')
    if not state.torque_enabled:
        raise RuntimeError('Enable winch torque before unlocking the mechanism')
    retract_safety.require_reference()
    if state.lock_state != 'locked':
        raise RuntimeError('Mechanical lock must be engaged before unlocking')
    state.lock_state = 'unlocking'
    state.unlock.position_deg = None
    state.unlock.current_a = None
    state.unlock.position_error_deg = None
    state.unlock.success = None
    try:
        relieve_pawl_load()
        time.sleep(config.UNLOCK_RELIEF_SETTLE_DELAY)
        _lift_pawl()
        position_deg, current_a, error_deg = verify_unlock()
        state.unlock.position_deg = position_deg
        state.unlock.current_a = current_a
        state.unlock.position_error_deg = error_deg
        state.unlock.success = True
        state.lock_state = 'unlocked'
    except Exception:
        state.unlock.success = False
        state.lock_state = 'locked'
        _release_pawl_after_failure()
        raise


@serialized_control
def lock_mechanism(stop_winch=True):
    """
    Disable XW430 torque so the spring engages the mechanical lock.

    If requested, stop the winch before engaging the lock.
    """
    if not state.initialized:
        raise RuntimeError('Initialize the system first')
    if stop_winch and state.motion.direction != 0:
        execute_stop()
    with motor_bus.session(state.bus_lock) as bus:
        bus.set_baudrate(config.LOCK_BAUDRATE)
        bus.set_torque(
            config.LOCK_ID,
            config.TORQUE_DISABLE,
            action='Disable lock motor torque',
        )
    state.lock_state = 'locked'


@serialized_control
def execute_stop() -> dict:
    """Stop the winch and reset motion state."""
    write_velocity(0)
    state.motion.direction = 0
    state.motion.speed_level = 0
    return get_motor_state()


@serialized_control
def _execute_direction(requested_direction):
    require_winch_ready()
    if state.motion.direction == requested_direction:
        next_level = min(state.motion.speed_level + 1, len(config.SPEED_LEVELS) - 1)
        next_direction = state.motion.direction
    elif state.motion.direction != 0:
        if state.motion.speed_level == 0:
            return execute_stop()
        next_level = state.motion.speed_level - 1
        next_direction = state.motion.direction
    else:
        next_level = 0
        next_direction = requested_direction
    write_velocity(-next_direction * config.SPEED_LEVELS[next_level])
    state.motion.direction = next_direction
    state.motion.speed_level = next_level
    return get_motor_state()


def execute_retract() -> dict:
    """Retract or reduce deployment speed, respecting the initialization limit."""
    return _execute_direction(-1)


def execute_deploy() -> dict:
    """Deploy or reduce retraction speed; deployment can leave the retract limit."""
    return _execute_direction(1)


def ping_motor() -> dict:
    """Ping both Dynamixels on the shared U2D2 connection."""
    try:
        with motor_bus.session(state.bus_lock) as bus:
            bus.set_baudrate(config.WINCH_BAUDRATE)
            winch_model = bus.ping(config.WINCH_ID, action='Ping winch motor')
            bus.set_baudrate(config.LOCK_BAUDRATE)
            lock_model = bus.ping(config.LOCK_ID, action='Ping lock motor')
            return {
                'success': True,
                'connected': True,
                'device': config.DEVICE_NAME,
                'winch_connected': True,
                'lock_connected': True,
                'winch_model_number': winch_model,
                'lock_model_number': lock_model,
            }
    except Exception as exc:
        return {
            'success': False,
            'connected': False,
            'winch_connected': False,
            'lock_connected': False,
            'error': str(exc),
        }


@serialized_control
def initialize_motor() -> dict:
    """Initialize both Dynamixels and leave the system safely locked."""
    if state.motion.velocity != 0 or state.retract_limit.stopping:
        return {'success': False, 'error': 'Stop the winch before initializing'}
    state.retract_limit.reference = None
    state.retract_limit.last_position = None
    try:
        with motor_bus.session(state.bus_lock) as bus:
            _configure_winch(bus)
            initialize_lock_motor(bus)
            retract_safety.capture_reference(bus)
            state.motion.direction = 0
            state.motion.speed_level = 0
            state.motion.velocity = 0
            state.initialized = True
            state.torque_enabled = False
            state.lock_state = 'locked'
            return get_motor_state()
    except Exception as exc:
        state.retract_limit.reference = None
        state.initialized = False
        state.torque_enabled = False
        state.lock_state = 'locked'
        return {'success': False, 'error': str(exc)}


@serialized_control
def enable_torque() -> dict:
    try:
        if not state.initialized:
            raise RuntimeError('Initialize the system before enabling torque')
        retract_safety.require_reference()
        with motor_bus.session(state.bus_lock) as bus:
            bus.set_baudrate(config.WINCH_BAUDRATE)
            bus.set_torque(
                config.WINCH_ID,
                config.TORQUE_ENABLE,
                action='Enable winch torque',
            )
        state.torque_enabled = True
        return get_motor_state()
    except Exception as exc:
        return {'success': False, 'error': str(exc)}


@serialized_control
def disable_torque() -> dict:
    """
    Stop and disable winch torque.

    The lock motor is also released so the spring engages the
    mechanical lock.
    """
    try:
        if not state.initialized:
            raise RuntimeError('System is not initialized')
        with motor_bus.session(state.bus_lock) as bus:
            bus.set_baudrate(config.WINCH_BAUDRATE)
            bus.set_velocity(config.WINCH_ID, 0, action='Stop winch motor')
            bus.set_torque(
                config.WINCH_ID,
                config.TORQUE_DISABLE,
                action='Disable winch torque',
            )
            bus.set_baudrate(config.LOCK_BAUDRATE)
            bus.set_torque(
                config.LOCK_ID,
                config.TORQUE_DISABLE,
                action='Engage mechanical lock',
            )
        state.retract_limit.stopping = False
        state.motion.direction = 0
        state.motion.speed_level = 0
        state.motion.velocity = 0
        state.torque_enabled = False
        state.lock_state = 'locked'
        return get_motor_state()
    except Exception as exc:
        return {'success': False, 'error': str(exc)}


@serialized_control
def toggle_lock() -> dict:
    try:
        if not state.initialized:
            raise RuntimeError('Initialize the system first')
        if state.lock_state == 'locked':
            unlock_mechanism()
        elif state.lock_state == 'unlocked':
            lock_mechanism(stop_winch=True)
        else:
            raise RuntimeError('Lock mechanism is busy')
        return get_motor_state()
    except Exception as exc:
        return {
            'success': False,
            'error': str(exc),
            'lock_state': state.lock_state,
            'unlock_diagnostics': {
                'relief_motor_deg': config.UNLOCK_RELIEF_MOTOR_DEG,
                'relief_ratchet_deg': config.UNLOCK_RELIEF_MOTOR_DEG / 2.0,
                'lock_position_deg': state.unlock.position_deg,
                'lock_current_a': state.unlock.current_a,
                'position_error_deg': state.unlock.position_error_deg,
                'success': state.unlock.success,
            },
        }


def _configure_winch(bus):
    """Configure velocity mode with torque disabled and a zero velocity goal."""
    bus.set_baudrate(config.WINCH_BAUDRATE)
    bus.set_torque(config.WINCH_ID, config.TORQUE_DISABLE, action='Disable winch torque')
    bus.set_operating_mode(
        config.WINCH_ID,
        config.VELOCITY_MODE,
        action='Set winch velocity mode',
    )
    bus.set_profile_acceleration(
        config.WINCH_ID,
        config.WINCH_PROFILE_ACCELERATION,
        action='Set winch profile acceleration',
    )
    bus.set_velocity(config.WINCH_ID, 0, action='Set winch zero velocity')


def _lift_pawl():
    """Lift the pawl, then allow the original settling delay before verification."""
    with motor_bus.session(state.bus_lock) as bus:
        bus.set_baudrate(config.LOCK_BAUDRATE)
        bus.set_torque(
            config.LOCK_ID,
            config.TORQUE_ENABLE,
            action='Enable lock motor torque',
        )
        bus.set_position(
            config.LOCK_ID,
            config.UNLOCK_POSITION_RAW,
            action='Command unlock position',
        )
    time.sleep(config.LOCK_COMMAND_DELAY)


def _release_pawl_after_failure():
    """Best-effort spring-lock engagement, preserving the original unlock error."""
    try:
        with motor_bus.session(state.bus_lock) as bus:
            bus.set_baudrate(config.LOCK_BAUDRATE)
            bus.set_torque(
                config.LOCK_ID,
                config.TORQUE_DISABLE,
                action='Engage mechanical lock',
                checked=False,
            )
    except Exception:
        pass