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
            retract_safety.stop_and_lock(bus)
            raise
        if velocity == 0:
            # Keep feedback sampling until physical motion has actually stopped,
            # in either direction, so the continuous HOME coordinate stays valid.
            state.retract_limit.stopping = state.retract_limit.stopping or previous_velocity != 0
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
        'home_recovery_mode': state.home_recovery_mode,
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
            'fault_recoverable': retract_safety.fault_is_recoverable(),
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
            start_position = int(bus.read_position(config.WINCH_ID, action='Read winch start position')) % config.WINCH_COUNTS_PER_REV
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
                position_raw = int(position) % config.WINCH_COUNTS_PER_REV
                movement = retract_safety.encoder_delta(position_raw, start_position)
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
    """Ping both Dynamixels and keep device faults distinct from disconnection."""
    result = {
        'success': True,
        'connected': False,
        'device': config.DEVICE_NAME,
        'winch_connected': False,
        'lock_connected': False,
        'winch_hardware_error': False,
        'lock_hardware_error': False,
    }
    try:
        with motor_bus.session(state.bus_lock) as bus:
            bus.set_baudrate(config.WINCH_BAUDRATE)
            try:
                winch = bus.ping_status(config.WINCH_ID, action='Ping winch motor')
                result['winch_connected'] = True
                result['winch_model_number'] = winch['model']
                result['winch_hardware_error'] = bool(winch['device_error'])
                result['winch_error'] = winch['device_error_text']
            except Exception as exc:
                result['winch_error'] = str(exc)

            bus.set_baudrate(config.LOCK_BAUDRATE)
            try:
                lock = bus.ping_status(config.LOCK_ID, action='Ping lock motor')
                result['lock_connected'] = True
                result['lock_model_number'] = lock['model']
                result['lock_hardware_error'] = bool(lock['device_error'])
                result['lock_error'] = lock['device_error_text']
            except Exception as exc:
                result['lock_error'] = str(exc)
    except Exception as exc:
        result['success'] = False
        result['error'] = str(exc)
        return result

    result['connected'] = result['winch_connected'] and result['lock_connected']
    return result


@serialized_control
def recover_winch_hardware() -> dict:
    """Recover a responding XW540 hardware shutdown without dropping the load.

    If HOME is valid, use reset_safety_fault(). If HOME is absent, first engage
    the passive pawl, reboot the XW540, and leave the system uninitialized.
    """
    if state.initialized and state.retract_limit.reference is not None:
        if not retract_safety.fault_is_recoverable():
            raise RuntimeError('Active fault is not safely recoverable; reinitialize instead')
        return reset_safety_fault()

    with motor_bus.session(state.bus_lock) as bus:
        # Never reboot a load-bearing XW540 with the pawl intentionally lifted.
        retract_safety.engage_mechanical_lock(bus)
        time.sleep(config.LOCK_ENGAGE_SETTLE_DELAY)
        bus.set_baudrate(config.WINCH_BAUDRATE)
        bus.reboot(
            config.WINCH_ID,
            action='Reboot winch motor',
            allow_device_error=True,
        )
        time.sleep(0.5)
        bus.set_baudrate(config.WINCH_BAUDRATE)
        bus.set_velocity(config.WINCH_ID, 0, action='Set winch zero velocity after recovery')

    # No HOME exists in this process. Never invent one.
    state.initialized = False
    state.home_recovery_mode = False
    state.torque_enabled = False
    state.lock_state = 'locked'
    state.motion.direction = 0
    state.motion.speed_level = 0
    state.motion.velocity = 0
    state.retract_limit.reference = None
    state.retract_limit.raw_position = None
    state.retract_limit.last_position = None
    state.retract_limit.deployed_counts = 0
    state.retract_limit.fault = None
    state.retract_limit.stopping = False
    state.retract_limit.reached = False
    return get_motor_state()


@serialized_control
def initialize_motor() -> dict:
    """Initialize both Dynamixels and leave the system safely locked."""
    if state.home_recovery_mode:
        return {'success': False, 'error': 'Finish HOME recovery before initializing'}
    if state.motion.velocity != 0 or state.retract_limit.stopping:
        return {'success': False, 'error': 'Stop the winch before initializing'}
    state.retract_limit.reference = None
    state.retract_limit.last_position = None
    state.retract_limit.raw_position = None
    state.retract_limit.deployed_counts = 0
    try:
        with motor_bus.session(state.bus_lock) as bus:
            _configure_winch(bus)
            initialize_lock_motor(bus)
            retract_safety.capture_reference(bus)
            state.motion.direction = 0
            state.motion.speed_level = 0
            state.motion.velocity = 0
            state.initialized = True
            state.home_recovery_mode = False
            state.torque_enabled = False
            state.lock_state = 'locked'
            return get_motor_state()
    except Exception as exc:
        state.retract_limit.reference = None
        state.initialized = False
        state.home_recovery_mode = False
        state.torque_enabled = False
        state.lock_state = 'locked'
        return {'success': False, 'error': str(exc)}


@serialized_control
def reset_safety_fault() -> dict:
    """Recover a torque/hardware shutdown while preserving HOME when safe.

    Critical invariant: the spring-loaded pawl is engaged BEFORE XW540 reboot.
    Recovery always finishes LOCKED; only an explicit later UNLOCK may move.
    """
    try:
        if not state.initialized or state.retract_limit.reference is None:
            raise RuntimeError('System is not initialized')
        if not retract_safety.fault_is_recoverable():
            raise RuntimeError(
                'Safety fault is not remotely recoverable; HOME recovery is required',
            )

        state.depth.active = False
        state.depth.mode = 'idle'
        state.depth.command_velocity = 0
        state.motion.direction = 0
        state.motion.speed_level = 0
        state.motion.velocity = 0

        with motor_bus.session(state.bus_lock) as bus:
            # 1) Make the load passive-safe first. XW430 torque OFF lets the
            # spring drive the pawl into the ratchet.
            retract_safety.engage_mechanical_lock(bus)
            time.sleep(config.LOCK_ENGAGE_SETTLE_DELAY)

            # 2) Reboot the faulted XW540 only after the pawl has been commanded in.
            bus.set_baudrate(config.WINCH_BAUDRATE)
            # A hardware-shutdown motor can acknowledge REBOOT while the reply
            # still carries the *old* hardware-error bit. Accept that device bit
            # for this instruction only; a transport failure is still fatal.
            bus.reboot(
                config.WINCH_ID,
                action='Reboot winch motor',
                allow_device_error=True,
            )
            state.torque_enabled = False
            time.sleep(0.5)

            # 3) Zero goal BEFORE torque enable, then enable holding torque.
            bus.set_baudrate(config.WINCH_BAUDRATE)
            bus.set_velocity(config.WINCH_ID, 0, action='Set winch zero velocity after reboot')
            bus.set_torque(
                config.WINCH_ID, config.TORQUE_ENABLE,
                action='Re-enable winch torque after safety reset',
            )
            state.torque_enabled = True

            # 4) Only now check encoder continuity. If this fails, keep the pawl
            # locked and invalidate HOME rather than pretending recovery succeeded.
            position = bus.read_position(
                config.WINCH_ID, action='Read winch position after safety reset',
            )
            try:
                retract_safety.update_position(position)
            except Exception:
                # This is an expected branch after a hard obstruction: the pawl may
                # have caught on the next ratchet tooth, so the encoder can legitimately
                # move farther than the continuity threshold. Do not expose the low-level
                # encoder exception to the operator. Invalidate HOME and transition in
                # one press to the explicit supervised HOME-recovery workflow.
                state.retract_limit.reference = None
                state.retract_limit.raw_position = None
                state.retract_limit.last_position = None
                state.retract_limit.deployed_counts = 0
                state.retract_limit.fault = None
                state.retract_limit.stopping = False
                state.retract_limit.reached = False
                state.initialized = False
                state.lock_state = 'locked'
                state.home_recovery_mode = False
                state.depth.phase = 'idle'
                state.depth.last_error = None
                return {
                    **get_motor_state(),
                    'home_recovery_required': True,
                }

        retract_safety.clear_recoverable_fault()
        state.lock_state = 'locked'
        state.home_recovery_mode = False
        state.depth.phase = 'idle'
        state.depth.last_error = None
        return get_motor_state()
    except Exception as exc:
        # Do not claim torque state changed unless we know it did. The mechanical
        # lock was commanded before reboot and remains the primary load holder.
        state.lock_state = 'locked'
        return {**get_motor_state(), 'success': False, 'error': str(exc)}


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


@serialized_control
def start_home_recovery() -> dict:
    """Enter supervised HOME-loss recovery with no automatic retract limit.

    This mode exists only to retrieve a CTD after HOME was lost. It enables
    XW540 holding torque, performs the normal bounded pawl load relief, then
    lifts the pawl. Movement is restricted to bounded RETRACT jogs.
    """
    if state.initialized or state.retract_limit.reference is not None:
        raise RuntimeError('HOME recovery is only available when HOME is not valid')
    if state.home_recovery_mode:
        return get_motor_state()

    state.motion.direction = 0
    state.motion.speed_level = 0
    state.motion.velocity = 0
    state.lock_state = 'locked'
    with motor_bus.session(state.bus_lock) as bus:
        retract_safety.engage_mechanical_lock(bus)
        time.sleep(config.LOCK_ENGAGE_SETTLE_DELAY)
        initialize_lock_motor(bus)
        _configure_winch(bus)
        bus.set_baudrate(config.WINCH_BAUDRATE)
        bus.set_torque(config.WINCH_ID, config.TORQUE_ENABLE, action='Enable winch torque for HOME recovery')
        state.torque_enabled = True

        # Bounded retract relief; unlike normal unlock this intentionally does not
        # require HOME because HOME is exactly what is being recovered.
        start_raw = int(bus.read_position(config.WINCH_ID, action='Read recovery relief start')) % config.WINCH_COUNTS_PER_REV
        bus.set_velocity(config.WINCH_ID, config.UNLOCK_RELIEF_VELOCITY, action='Recovery pawl load relief')
        deadline = time.monotonic() + config.UNLOCK_RELIEF_TIMEOUT
        try:
            while True:
                raw = int(bus.read_position(config.WINCH_ID, action='Read recovery relief position')) % config.WINCH_COUNTS_PER_REV
                if retract_safety.encoder_delta(raw, start_raw) >= config.UNLOCK_RELIEF_COUNTS:
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError('HOME recovery pawl load relief timed out')
                time.sleep(config.UNLOCK_RELIEF_POLL_INTERVAL)
        finally:
            bus.set_velocity(config.WINCH_ID, 0, action='Stop recovery pawl relief')

    time.sleep(config.UNLOCK_RELIEF_SETTLE_DELAY)
    try:
        _lift_pawl()
        verify_unlock()
    except Exception:
        _release_pawl_after_failure()
        state.lock_state = 'locked'
        raise
    state.lock_state = 'unlocked'
    state.home_recovery_mode = True
    state.retract_limit.fault = None
    state.retract_limit.stopping = False
    return get_motor_state()


@serialized_control
def home_recovery_jog(direction: int) -> dict:
    """Perform one slow, encoder-bounded HOME-recovery jog.

    direction=+1 retracts toward storage; direction=-1 deploys away from storage.
    HOME is unknown in this mode, so BOTH directions are deliberately slow and
    bounded to one short encoder-measured jog per operator press.  Any error
    commands zero velocity and engages the spring-loaded pawl.
    """
    if direction not in (-1, 1):
        raise ValueError('HOME recovery direction must be +1 (retract) or -1 (deploy)')
    if not state.home_recovery_mode or state.initialized:
        raise RuntimeError('Start HOME recovery first')
    if not state.torque_enabled or state.lock_state != 'unlocked':
        raise RuntimeError('HOME recovery is not ready for motion')

    label = 'retract' if direction > 0 else 'deploy'
    command = direction * config.RECOVERY_JOG_VELOCITY
    target_counts = config.RECOVERY_JOG_COUNTS

    with motor_bus.session(state.bus_lock) as bus:
        bus.set_baudrate(config.WINCH_BAUDRATE)
        start_raw = int(bus.read_position(
            config.WINCH_ID, action=f'Read recovery {label} jog start',
        )) % config.WINCH_COUNTS_PER_REV
        started = False
        try:
            bus.set_velocity(
                config.WINCH_ID, command,
                action=f'Start HOME recovery {label} jog',
            )
            started = True
            state.motion.direction = -1 if direction > 0 else 1
            state.motion.velocity = command
            state.motion.speed_level = 0
            deadline = time.monotonic() + config.RECOVERY_JOG_TIMEOUT
            while True:
                raw = int(bus.read_position(
                    config.WINCH_ID, action=f'Read recovery {label} jog position',
                )) % config.WINCH_COUNTS_PER_REV
                moved = retract_safety.encoder_delta(raw, start_raw)
                if direction * moved >= target_counts:
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError(f'HOME recovery {label} jog timed out')
                time.sleep(config.RECOVERY_JOG_POLL_INTERVAL)
            bus.set_velocity(config.WINCH_ID, 0, action='Stop HOME recovery jog')
        except Exception:
            if started:
                try:
                    bus.set_baudrate(config.WINCH_BAUDRATE)
                    bus.set_velocity(config.WINCH_ID, 0, action='Emergency stop HOME recovery jog')
                except Exception:
                    pass
            try:
                retract_safety.engage_mechanical_lock(bus)
                state.lock_state = 'locked'
            except Exception:
                pass
            state.home_recovery_mode = False
            raise
        finally:
            state.motion.direction = 0
            state.motion.velocity = 0
            state.motion.speed_level = 0
    return get_motor_state()


def home_recovery_retract_jog() -> dict:
    return home_recovery_jog(+1)


def home_recovery_deploy_jog() -> dict:
    return home_recovery_jog(-1)


@serialized_control
def finish_home_recovery() -> dict:
    """Stop, engage pawl, disable XW540 torque, and exit recovery mode."""
    if not state.home_recovery_mode:
        raise RuntimeError('HOME recovery is not active')
    with motor_bus.session(state.bus_lock) as bus:
        bus.set_baudrate(config.WINCH_BAUDRATE)
        bus.set_velocity(config.WINCH_ID, 0, action='Stop HOME recovery')
        retract_safety.engage_mechanical_lock(bus)
        time.sleep(config.LOCK_ENGAGE_SETTLE_DELAY)
        bus.set_baudrate(config.WINCH_BAUDRATE)
        bus.set_torque(config.WINCH_ID, config.TORQUE_DISABLE, action='Disable winch torque after HOME recovery')
    state.home_recovery_mode = False
    state.torque_enabled = False
    state.lock_state = 'locked'
    state.motion.direction = 0
    state.motion.velocity = 0
    state.motion.speed_level = 0
    state.retract_limit.fault = None
    state.retract_limit.stopping = False
    return get_motor_state()


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