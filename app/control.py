"""Motor control, shared state, and hardware communication.

HTTP and MAVLink handlers call these functions through the control module.
All motor state and the shared serial-bus lock are owned here. Importing this
module does not start a listener or move either motor.
"""

from pathlib import Path
from functools import wraps
from threading import Event, RLock, Thread
import logging
import math
import time

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

import config


# ============================================================
# APPLICATION STATE
# ============================================================

# direction:
#   -1 = retract
#    0 = stopped
#   +1 = deploy

direction = 0
speed_level = 0
current_velocity = 0

initialized = False
torque_enabled = False
lock_state = "locked"

# Last unlock diagnostic values.
last_unlock_lock_position_deg = None
last_unlock_lock_current_a = None
last_unlock_position_error_deg = None
last_unlock_success = None

bus_lock = RLock()

# Positive encoder counts are retract. Never wrap this reference to one turn.
retract_limit_position = None
last_winch_position = None
retract_limit_reached = False
retract_limit_fault = None
_retract_stop_pending = False
_limit_monitor_thread = None
_limit_monitor_shutdown = Event()
logger = logging.getLogger(__name__)


def serialized_control(function):
    """Serialize state changes and bus access with the safety monitor.

    A reentrant lock allows command sequences to call other control helpers.
    The unlock relief loop monitors its own movement while holding this lock.
    """
    @wraps(function)
    def wrapped(*args, **kwargs):
        with bus_lock:
            return function(*args, **kwargs)
    return wrapped


# ============================================================
# DYNAMIXEL BUS HELPERS
# ============================================================

def open_bus():
    """Open the shared U2D2 serial port."""

    if not Path(config.DEVICE_NAME).exists():
        raise RuntimeError(f"{config.DEVICE_NAME} does not exist")

    port = PortHandler(config.DEVICE_NAME)

    if not port.openPort():
        raise RuntimeError(f"Could not open {config.DEVICE_NAME}")

    return port


def check_result(packet, comm_result, dxl_error, action):
    """Validate a Dynamixel communication result."""

    if comm_result != COMM_SUCCESS:
        raise RuntimeError(
            f"{action}: {packet.getTxRxResult(comm_result)}"
        )

    if dxl_error != 0:
        raise RuntimeError(
            f"{action}: {packet.getRxPacketError(dxl_error)}"
        )


def set_bus_baudrate(port, baudrate):
    """Switch the U2D2 baud rate for the motor being addressed."""

    if not port.setBaudRate(baudrate):
        raise RuntimeError(
            f"Could not set bus baud rate to {baudrate}"
        )


def signed_16(value):
    return value - 0x10000 if value & 0x8000 else value


def signed_32(value):
    return value - 0x100000000 if value & 0x80000000 else value


# ============================================================
# TELEMETRY HELPERS
# ============================================================

def read_motor_telemetry(port, packet, motor_id, baudrate):
    """Read current, speed, position, voltage and temperature."""

    set_bus_baudrate(port, baudrate)

    raw_current, comm_result, dxl_error = packet.read2ByteTxRx(
        port, motor_id, config.ADDR_PRESENT_CURRENT
    )
    check_result(
        packet,
        comm_result,
        dxl_error,
        f"Read motor {motor_id} current",
    )

    raw_velocity, comm_result, dxl_error = packet.read4ByteTxRx(
        port, motor_id, config.ADDR_PRESENT_VELOCITY
    )
    check_result(
        packet,
        comm_result,
        dxl_error,
        f"Read motor {motor_id} velocity",
    )

    raw_position, comm_result, dxl_error = packet.read4ByteTxRx(
        port, motor_id, config.ADDR_PRESENT_POSITION
    )
    check_result(
        packet,
        comm_result,
        dxl_error,
        f"Read motor {motor_id} position",
    )

    raw_voltage, comm_result, dxl_error = packet.read2ByteTxRx(
        port, motor_id, config.ADDR_PRESENT_INPUT_VOLTAGE
    )
    check_result(
        packet,
        comm_result,
        dxl_error,
        f"Read motor {motor_id} voltage",
    )

    raw_temperature, comm_result, dxl_error = packet.read1ByteTxRx(
        port, motor_id, config.ADDR_PRESENT_TEMPERATURE
    )
    check_result(
        packet,
        comm_result,
        dxl_error,
        f"Read motor {motor_id} temperature",
    )

    return {
        "current_a": round(
            signed_16(raw_current)
            * config.PRESENT_CURRENT_MA_PER_UNIT
            / 1000.0,
            3,
        ),
        "rpm": round(
            signed_32(raw_velocity)
            * config.PRESENT_VELOCITY_RPM_PER_UNIT,
            2,
        ),
        "position_deg": round(
            signed_32(raw_position)
            * config.POSITION_DEG_PER_COUNT,
            1,
        ),
        "voltage_v": round(
            raw_voltage
            * config.PRESENT_VOLTAGE_V_PER_UNIT,
            1,
        ),
        "temperature_c": int(raw_temperature),
    }


def read_all_telemetry():
    """Read telemetry from both Dynamixels."""

    with bus_lock:
        port = open_bus()
        packet = PacketHandler(config.PROTOCOL_VERSION)

        try:
            winch = read_motor_telemetry(
                port,
                packet,
                config.WINCH_ID,
                config.WINCH_BAUDRATE,
            )
            lock = read_motor_telemetry(
                port,
                packet,
                config.LOCK_ID,
                config.LOCK_BAUDRATE,
            )
        finally:
            port.closePort()

    return {
        "success": True,
        "winch": winch,
        "lock": lock,
    }


# ============================================================
# WINCH MOTOR HELPERS
# ============================================================

def _read_winch_feedback(port, packet):
    """Read unrounded, signed multi-turn position and actual velocity."""
    global last_winch_position

    position, result, error = packet.read4ByteTxRx(
        port, config.WINCH_ID, config.ADDR_PRESENT_POSITION
    )
    check_result(packet, result, error, "Read winch limit position")
    velocity, result, error = packet.read4ByteTxRx(
        port, config.WINCH_ID, config.ADDR_PRESENT_VELOCITY
    )
    check_result(packet, result, error, "Read winch limit velocity")
    if torque_enabled:
        torque, result, error = packet.read1ByteTxRx(
            port, config.WINCH_ID, config.ADDR_TORQUE_ENABLE
        )
        check_result(packet, result, error, "Read winch torque status")
        if torque != config.TORQUE_ENABLE:
            raise RuntimeError("Winch torque was lost; initialize again before moving")
    last_winch_position = signed_32(position)
    return last_winch_position, signed_32(velocity)


def _retract_stopping_counts(velocity):
    """Conservative braking envelope for the configured velocity-based profile.

    ROBOTIS specifies 0.229 rpm per velocity unit and 214.577 rpm/min
    per acceleration unit. Physical load, bus delays, and inertia still require
    on-hardware validation; this is not a hardware limit switch.
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


def _send_winch_velocity(port, packet, velocity):
    """Write a goal on an already locked/open winch bus; update on success only."""
    global current_velocity
    result, error = packet.write4ByteTxRx(
        port, config.WINCH_ID, config.ADDR_GOAL_VELOCITY, velocity & 0xFFFFFFFF
    )
    check_result(packet, result, error, "Set goal velocity")
    current_velocity = velocity


def _stop_for_retract_limit(port, packet):
    global direction, speed_level, _retract_stop_pending
    # Keep retrying from the monitor if the write fails. Never claim a stop
    # succeeded merely because we attempted it.
    _retract_stop_pending = True
    _send_winch_velocity(port, packet, 0)
    direction = 0
    speed_level = 0


def _record_limit_fault(exc):
    global retract_limit_fault, _retract_stop_pending
    message = str(exc)
    if retract_limit_fault != message:
        logger.error("Retract safety fault: %s", message)
    retract_limit_fault = message
    _retract_stop_pending = True


def _require_limit_reference():
    if not initialized or retract_limit_position is None:
        raise RuntimeError("Initialize the system to establish the retract limit")
    if retract_limit_fault is not None:
        raise RuntimeError(f"Retract safety fault: {retract_limit_fault}; initialize again")
    if _limit_monitor_thread is None or not _limit_monitor_thread.is_alive():
        raise RuntimeError("Retract safety monitor is not running; initialize again")


@serialized_control
def check_retract_limit():
    """One monitor cycle, also callable in hardware-free regression tests."""
    global retract_limit_reached, _retract_stop_pending
    if not initialized or not torque_enabled:
        return
    if current_velocity <= 0 and not _retract_stop_pending:
        return

    port = None
    try:
        port = open_bus()
        packet = PacketHandler(config.PROTOCOL_VERSION)
        set_bus_baudrate(port, config.WINCH_BAUDRATE)
        try:
            position, measured_velocity = _read_winch_feedback(port, packet)
            if retract_limit_position is None:
                raise RuntimeError("Retract reference is missing")
            if retract_limit_fault is not None:
                _stop_for_retract_limit(port, packet)
            elif current_velocity > 0 and (
                retract_limit_position - position
                <= _retract_stopping_counts(max(current_velocity, measured_velocity))
            ):
                retract_limit_reached = True
                _stop_for_retract_limit(port, packet)
            # Continue checking during deceleration even after goal velocity is 0.
            if current_velocity <= 0 and measured_velocity <= 0:
                _retract_stop_pending = False
        except Exception as exc:
            _record_limit_fault(exc)
            _stop_for_retract_limit(port, packet)
    except Exception as exc:
        _record_limit_fault(exc)
    finally:
        if port is not None:
            port.closePort()


def _retract_limit_monitor():
    while not _limit_monitor_shutdown.wait(config.RETRACT_LIMIT_POLL_INTERVAL):
        try:
            check_retract_limit()
        except Exception as exc:
            # Unexpected errors must not silently kill the monitoring thread.
            with bus_lock:
                _record_limit_fault(exc)


def _start_retract_limit_monitor():
    global _limit_monitor_thread
    if _limit_monitor_thread is not None and _limit_monitor_thread.is_alive():
        return
    _limit_monitor_shutdown.clear()
    _limit_monitor_thread = Thread(
        target=_retract_limit_monitor, name="winch-retract-limit", daemon=True
    )
    _limit_monitor_thread.start()


@serialized_control
def write_velocity(velocity: int):
    """Command motor velocity, guarding every positive (retract) command.

    Unlock load relief has a separate, bounded loop; it cannot be requested
    through this function, HTTP movement commands, or MAVLink commands.
    """
    global retract_limit_reached, _retract_stop_pending

    if velocity != 0:
        _require_limit_reference()
        if not torque_enabled:
            raise RuntimeError("Motor torque is not enabled")
        if lock_state != "unlocked":
            raise RuntimeError("Mechanical lock is engaged")

    with bus_lock:
        port = open_bus()
        packet = PacketHandler(config.PROTOCOL_VERSION)
        try:
            set_bus_baudrate(port, config.WINCH_BAUDRATE)
            if velocity > 0:
                try:
                    position, measured_velocity = _read_winch_feedback(port, packet)
                    stopping_counts = _retract_stopping_counts(
                        max(velocity, current_velocity, measured_velocity)
                    )
                except Exception as exc:
                    _record_limit_fault(exc)
                    _stop_for_retract_limit(port, packet)
                    raise
                if retract_limit_position - position <= stopping_counts:
                    retract_limit_reached = True
                    _stop_for_retract_limit(port, packet)
                    raise RuntimeError("Retract limit reached (including braking allowance)")
            previous_velocity = current_velocity
            try:
                _send_winch_velocity(port, packet, velocity)
            except Exception as exc:
                # A missing acknowledgement does not prove the motor ignored
                # the command. Latch a fault and attempt a stop immediately.
                _record_limit_fault(exc)
                _stop_for_retract_limit(port, packet)
                raise
            if velocity <= 0:
                # Reversal/stop commands do not instantly remove positive motion.
                _retract_stop_pending = _retract_stop_pending or previous_velocity > 0
            if velocity != 0:
                retract_limit_reached = False
        finally:
            port.closePort()



def get_motor_state() -> dict:
    """Return the current application-side winch state."""

    rpm = abs(current_velocity) * config.DYNAMIXEL_RPM_PER_UNIT

    if direction == -1:
        status_text = "retracting"
    elif direction == 1:
        status_text = "deploying"
    else:
        status_text = "stopped"

    return {
        "success": True,
        "initialized": initialized,
        "torque_enabled": torque_enabled,
        "lock_state": lock_state,
        "status": status_text,
        "direction": direction,
        "speed_level": (
            speed_level + 1 if direction != 0 else 0
        ),
        "max_speed_level": len(config.SPEED_LEVELS),
        "velocity": current_velocity,
        "rpm": round(rpm, 1),
        "retract_limit": {
            "position_counts": retract_limit_position,
            "last_position_counts": last_winch_position,
            "reached": retract_limit_reached,
            "fault": retract_limit_fault,
            "stopping": _retract_stop_pending,
            "unlock_allowance_deg": config.UNLOCK_RELIEF_MOTOR_DEG,
        },
        "unlock_diagnostics": {
            "relief_motor_deg": config.UNLOCK_RELIEF_MOTOR_DEG,
            "relief_ratchet_deg": config.UNLOCK_RELIEF_MOTOR_DEG / 2.0,
            "lock_position_deg": last_unlock_lock_position_deg,
            "lock_current_a": last_unlock_lock_current_a,
            "position_error_deg": last_unlock_position_error_deg,
            "success": last_unlock_success,
        },
    }


def require_winch_ready():
    """Prevent movement unless the system is ready and unlocked."""

    if not initialized:
        raise RuntimeError("System is not initialized")

    if not torque_enabled:
        raise RuntimeError("Motor torque is not enabled")

    if lock_state != "unlocked":
        raise RuntimeError("Mechanical lock is engaged")

    _require_limit_reference()


# ============================================================
# RATCHET LOAD RELIEF
# ============================================================

@serialized_control
def relieve_pawl_load():
    """
    Rotate the XW540 slightly in the RETRACT direction before
    lifting the pawl.

    The winch must already have torque enabled. The movement is
    measured using Present Position instead of a fixed sleep.
    """

    global current_velocity

    if not initialized:
        raise RuntimeError("Initialize the system first")

    if not torque_enabled:
        raise RuntimeError(
            "Enable winch torque before unlocking the mechanism"
        )

    _require_limit_reference()
    if lock_state != "unlocking":
        raise RuntimeError("Load relief is only allowed during unlocking")

    # Ensure the winch is stationary before the relief move.
    execute_stop()

    with bus_lock:
        port = open_bus()
        packet = PacketHandler(config.PROTOCOL_VERSION)
        motion_started = False

        try:
            set_bus_baudrate(port, config.WINCH_BAUDRATE)

            start_position, comm_result, dxl_error = (
                packet.read4ByteTxRx(
                    port,
                    config.WINCH_ID,
                    config.ADDR_PRESENT_POSITION,
                )
            )
            check_result(
                packet,
                comm_result,
                dxl_error,
                "Read winch start position",
            )

            relief_ceiling = retract_limit_position + config.UNLOCK_RELIEF_COUNTS
            if signed_32(start_position) + config.UNLOCK_RELIEF_COUNTS > relief_ceiling:
                raise RuntimeError(
                    "Unlock relief would exceed its allowance beyond the retract limit"
                )

            # RETRACT is positive motor velocity.
            command = config.UNLOCK_RELIEF_VELOCITY & 0xFFFFFFFF

            motion_started = True  # Also stop on an ambiguous start acknowledgement.
            comm_result, dxl_error = packet.write4ByteTxRx(
                port,
                config.WINCH_ID,
                config.ADDR_GOAL_VELOCITY,
                command,
            )
            check_result(
                packet,
                comm_result,
                dxl_error,
                "Start ratchet load-relief movement",
            )

            current_velocity = config.UNLOCK_RELIEF_VELOCITY

            deadline = (
                time.monotonic()
                + config.UNLOCK_RELIEF_TIMEOUT
            )

            while True:
                position, comm_result, dxl_error = (
                    packet.read4ByteTxRx(
                        port,
                        config.WINCH_ID,
                        config.ADDR_PRESENT_POSITION,
                    )
                )
                check_result(
                    packet,
                    comm_result,
                    dxl_error,
                    "Read winch relief position",
                )

                start_position_signed = signed_32(start_position)
                position_signed = signed_32(position)
                movement = position_signed - start_position_signed

                if movement >= config.UNLOCK_RELIEF_COUNTS:
                    break

                if time.monotonic() >= deadline:
                    movement_deg = movement * config.POSITION_DEG_PER_COUNT

                    raise RuntimeError(
                        "Ratchet load-relief movement timed out: "
                        f"moved {movement_deg:.1f} deg, "
                        f"target {config.UNLOCK_RELIEF_MOTOR_DEG:.1f} deg"
                    )

                time.sleep(
                    config.UNLOCK_RELIEF_POLL_INTERVAL
                )

        finally:
            try:
                if motion_started:
                    try:
                        _send_winch_velocity(port, packet, 0)
                    except Exception as exc:
                        _record_limit_fault(exc)
                        raise
            finally:
                port.closePort()



# ============================================================
# LOCK MOTOR HELPERS
# ============================================================

@serialized_control
def initialize_lock_motor(port, packet):
    """
    Configure the XW430 in Position Control Mode.

    Initialization finishes with torque OFF so the spring keeps
    the ratchet mechanically locked.
    """

    global lock_state

    set_bus_baudrate(port, config.LOCK_BAUDRATE)

    comm_result, dxl_error = packet.write1ByteTxRx(
        port,
        config.LOCK_ID,
        config.LOCK_ADDR_TORQUE_ENABLE,
        config.TORQUE_DISABLE,
    )
    check_result(
        packet,
        comm_result,
        dxl_error,
        "Disable lock motor torque",
    )

    comm_result, dxl_error = packet.write1ByteTxRx(
        port,
        config.LOCK_ID,
        config.LOCK_ADDR_OPERATING_MODE,
        config.POSITION_MODE,
    )
    check_result(
        packet,
        comm_result,
        dxl_error,
        "Set lock motor position mode",
    )

    comm_result, dxl_error = packet.write4ByteTxRx(
        port,
        config.LOCK_ID,
        config.LOCK_ADDR_PROFILE_ACCELERATION,
        config.LOCK_PROFILE_ACCELERATION,
    )
    check_result(
        packet,
        comm_result,
        dxl_error,
        "Set lock motor profile acceleration",
    )

    comm_result, dxl_error = packet.write4ByteTxRx(
        port,
        config.LOCK_ID,
        config.LOCK_ADDR_PROFILE_VELOCITY,
        config.LOCK_PROFILE_VELOCITY,
    )
    check_result(
        packet,
        comm_result,
        dxl_error,
        "Set lock motor profile velocity",
    )

    lock_state = "locked"


def verify_unlock():
    """
    Verify that the XW430 reached the unlock position.

    The latest measured position, current and position error are stored
    in the application diagnostics on every poll, including failed
    unlock attempts.

    Returns (position_deg, current_a, error_deg).
    Raises RuntimeError if the target is not reached in time.
    """

    global last_unlock_lock_position_deg
    global last_unlock_lock_current_a
    global last_unlock_position_error_deg

    deadline = time.monotonic() + config.UNLOCK_VERIFY_TIMEOUT

    last_position_deg = None
    last_current_a = None
    last_error_deg = None

    while time.monotonic() < deadline:
        with bus_lock:
            port = open_bus()
            packet = PacketHandler(config.PROTOCOL_VERSION)

            try:
                set_bus_baudrate(
                    port,
                    config.LOCK_BAUDRATE,
                )

                raw_position, comm_result, dxl_error = (
                    packet.read4ByteTxRx(
                        port,
                        config.LOCK_ID,
                        config.ADDR_PRESENT_POSITION,
                    )
                )
                check_result(
                    packet,
                    comm_result,
                    dxl_error,
                    "Read lock motor position",
                )

                raw_current, comm_result, dxl_error = (
                    packet.read2ByteTxRx(
                        port,
                        config.LOCK_ID,
                        config.ADDR_PRESENT_CURRENT,
                    )
                )
                check_result(
                    packet,
                    comm_result,
                    dxl_error,
                    "Read lock motor current",
                )

            finally:
                port.closePort()

        last_position_deg = (
            signed_32(raw_position)
            * config.POSITION_DEG_PER_COUNT
        )
        last_current_a = (
            signed_16(raw_current)
            * config.PRESENT_CURRENT_MA_PER_UNIT
            / 1000.0
        )
        last_error_deg = abs(
            config.UNLOCK_POSITION_DEG
            - last_position_deg
        )

        # Persist every measurement so a failed attempt still leaves
        # useful diagnostics for the UI and /motor/state endpoint.
        last_unlock_lock_position_deg = round(last_position_deg, 1)
        last_unlock_lock_current_a = round(last_current_a, 3)
        last_unlock_position_error_deg = round(last_error_deg, 1)

        if (
            last_error_deg
            <= config.UNLOCK_POSITION_TOLERANCE_DEG
        ):
            return (
                last_unlock_lock_position_deg,
                last_unlock_lock_current_a,
                last_unlock_position_error_deg,
            )

        time.sleep(
            config.UNLOCK_VERIFY_POLL_INTERVAL
        )

    if last_position_deg is None:
        raise RuntimeError(
            "Unlock failed: no lock motor position measurement was obtained"
        )

    raise RuntimeError(
        "Unlock failed: lock motor did not reach "
        f"{config.UNLOCK_POSITION_DEG:.1f} deg "
        f"(last position "
        f"{last_unlock_lock_position_deg:.1f} deg, "
        f"error {last_unlock_position_error_deg:.1f} deg, "
        f"current {last_unlock_lock_current_a:.3f} A)"
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

    global lock_state
    global last_unlock_lock_position_deg
    global last_unlock_lock_current_a
    global last_unlock_position_error_deg
    global last_unlock_success

    if not initialized:
        raise RuntimeError("Initialize the system first")

    if not torque_enabled:
        raise RuntimeError(
            "Enable winch torque before unlocking the mechanism"
        )

    _require_limit_reference()
    if lock_state != "locked":
        raise RuntimeError("Mechanical lock must be engaged before unlocking")

    lock_state = "unlocking"
    last_unlock_lock_position_deg = None
    last_unlock_lock_current_a = None
    last_unlock_position_error_deg = None
    last_unlock_success = None

    try:
        # First unload the ratchet tooth from the pawl.
        relieve_pawl_load()

        # Give the loaded mechanism a short time to settle.
        time.sleep(
            config.UNLOCK_RELIEF_SETTLE_DELAY
        )

        # Lift the pawl with the XW430.
        with bus_lock:
            port = open_bus()
            packet = PacketHandler(config.PROTOCOL_VERSION)

            try:
                set_bus_baudrate(
                    port,
                    config.LOCK_BAUDRATE,
                )

                comm_result, dxl_error = (
                    packet.write1ByteTxRx(
                        port,
                        config.LOCK_ID,
                        config.LOCK_ADDR_TORQUE_ENABLE,
                        config.TORQUE_ENABLE,
                    )
                )
                check_result(
                    packet,
                    comm_result,
                    dxl_error,
                    "Enable lock motor torque",
                )

                comm_result, dxl_error = (
                    packet.write4ByteTxRx(
                        port,
                        config.LOCK_ID,
                        config.LOCK_ADDR_GOAL_POSITION,
                        config.UNLOCK_POSITION_RAW,
                    )
                )
                check_result(
                    packet,
                    comm_result,
                    dxl_error,
                    "Command unlock position",
                )

            finally:
                port.closePort()

        # Give the pawl time to start moving.
        time.sleep(config.LOCK_COMMAND_DELAY)

        (
            position_deg,
            current_a,
            error_deg,
        ) = verify_unlock()

        last_unlock_lock_position_deg = position_deg
        last_unlock_lock_current_a = current_a
        last_unlock_position_error_deg = error_deg
        last_unlock_success = True

        lock_state = "unlocked"

    except Exception:
        last_unlock_success = False
        lock_state = "locked"

        # Fail safe: release XW430 torque so the spring can
        # return the pawl toward the locked position.
        try:
            with bus_lock:
                port = open_bus()
                packet = PacketHandler(config.PROTOCOL_VERSION)

                try:
                    set_bus_baudrate(
                        port,
                        config.LOCK_BAUDRATE,
                    )
                    packet.write1ByteTxRx(
                        port,
                        config.LOCK_ID,
                        config.LOCK_ADDR_TORQUE_ENABLE,
                        config.TORQUE_DISABLE,
                    )
                finally:
                    port.closePort()
        except Exception:
            pass

        raise


@serialized_control
def lock_mechanism(stop_winch=True):
    """
    Disable XW430 torque so the spring engages the mechanical lock.

    If requested, stop the winch before engaging the lock.
    """

    global lock_state

    if not initialized:
        raise RuntimeError("Initialize the system first")

    if stop_winch and direction != 0:
        execute_stop()

    with bus_lock:
        port = open_bus()
        packet = PacketHandler(config.PROTOCOL_VERSION)

        try:
            set_bus_baudrate(port, config.LOCK_BAUDRATE)

            comm_result, dxl_error = packet.write1ByteTxRx(
                port,
                config.LOCK_ID,
                config.LOCK_ADDR_TORQUE_ENABLE,
                config.TORQUE_DISABLE,
            )
            check_result(
                packet,
                comm_result,
                dxl_error,
                "Disable lock motor torque",
            )

        finally:
            port.closePort()

    lock_state = "locked"


# ============================================================
# INTERNAL WINCH COMMANDS
# ============================================================

@serialized_control
def execute_stop() -> dict:
    """Stop the winch and reset motion state."""

    global direction
    global speed_level

    write_velocity(0)
    direction = 0
    speed_level = 0

    return get_motor_state()


@serialized_control
def _execute_direction(requested_direction):
    global direction, speed_level
    require_winch_ready()
    if direction == requested_direction:
        next_level = min(speed_level + 1, len(config.SPEED_LEVELS) - 1)
        next_direction = direction
    elif direction != 0:
        if speed_level == 0:
            return execute_stop()
        next_level = speed_level - 1
        next_direction = direction
    else:
        next_level = 0
        next_direction = requested_direction
    write_velocity(-next_direction * config.SPEED_LEVELS[next_level])
    direction = next_direction
    speed_level = next_level
    return get_motor_state()


def execute_retract() -> dict:
    """Retract or reduce deployment speed, respecting the initialization limit."""
    return _execute_direction(-1)


def execute_deploy() -> dict:
    """Deploy or reduce retraction speed; deployment can leave the retract limit."""
    return _execute_direction(1)


# ============================================================
# PING MOTOR
# ============================================================

def ping_motor() -> dict:
    """Ping both Dynamixels on the shared U2D2 connection."""

    try:
        with bus_lock:
            port = open_bus()
            packet = PacketHandler(config.PROTOCOL_VERSION)

            try:
                set_bus_baudrate(
                    port,
                    config.WINCH_BAUDRATE,
                )

                winch_model, comm_result, dxl_error = (
                    packet.ping(
                        port,
                        config.WINCH_ID,
                    )
                )
                check_result(
                    packet,
                    comm_result,
                    dxl_error,
                    "Ping winch motor",
                )

                set_bus_baudrate(
                    port,
                    config.LOCK_BAUDRATE,
                )

                lock_model, comm_result, dxl_error = (
                    packet.ping(
                        port,
                        config.LOCK_ID,
                    )
                )
                check_result(
                    packet,
                    comm_result,
                    dxl_error,
                    "Ping lock motor",
                )

                return {
                    "success": True,
                    "connected": True,
                    "device": config.DEVICE_NAME,
                    "winch_connected": True,
                    "lock_connected": True,
                    "winch_model_number": winch_model,
                    "lock_model_number": lock_model,
                }

            finally:
                port.closePort()

    except Exception as exc:
        return {
            "success": False,
            "connected": False,
            "winch_connected": False,
            "lock_connected": False,
            "error": str(exc),
        }



# ============================================================
# INITIALIZE MOTOR
# ============================================================

@serialized_control
def initialize_motor() -> dict:
    """Initialize both Dynamixels and leave the system safely locked."""

    global direction
    global speed_level
    global current_velocity
    global initialized
    global torque_enabled
    global lock_state

    global retract_limit_position, last_winch_position
    global retract_limit_reached, retract_limit_fault, _retract_stop_pending

    if current_velocity != 0 or _retract_stop_pending:
        return {"success": False, "error": "Stop the winch before initializing"}
    retract_limit_position = None
    last_winch_position = None

    try:
        with bus_lock:
            port = open_bus()
            packet = PacketHandler(config.PROTOCOL_VERSION)

            try:
                # ---------------- WINCH MOTOR ----------------
                set_bus_baudrate(
                    port,
                    config.WINCH_BAUDRATE,
                )

                comm_result, dxl_error = (
                    packet.write1ByteTxRx(
                        port,
                        config.WINCH_ID,
                        config.ADDR_TORQUE_ENABLE,
                        config.TORQUE_DISABLE,
                    )
                )
                check_result(
                    packet,
                    comm_result,
                    dxl_error,
                    "Disable winch torque",
                )

                comm_result, dxl_error = (
                    packet.write1ByteTxRx(
                        port,
                        config.WINCH_ID,
                        config.ADDR_OPERATING_MODE,
                        config.VELOCITY_MODE,
                    )
                )
                check_result(
                    packet,
                    comm_result,
                    dxl_error,
                    "Set winch velocity mode",
                )

                comm_result, dxl_error = (
                    packet.write4ByteTxRx(
                        port,
                        config.WINCH_ID,
                        config.ADDR_PROFILE_ACCELERATION,
                        config.WINCH_PROFILE_ACCELERATION,
                    )
                )
                check_result(
                    packet,
                    comm_result,
                    dxl_error,
                    "Set winch profile acceleration",
                )

                comm_result, dxl_error = (
                    packet.write4ByteTxRx(
                        port,
                        config.WINCH_ID,
                        config.ADDR_GOAL_VELOCITY,
                        0,
                    )
                )
                check_result(
                    packet,
                    comm_result,
                    dxl_error,
                    "Set winch zero velocity",
                )

                # ---------------- LOCK MOTOR -----------------
                initialize_lock_motor(
                    port,
                    packet,
                )

                # Capture the signed multi-turn encoder reference after setup.
                set_bus_baudrate(port, config.WINCH_BAUDRATE)
                drive_mode, result, error = packet.read1ByteTxRx(
                    port, config.WINCH_ID, config.ADDR_DRIVE_MODE
                )
                check_result(packet, result, error, "Read winch drive mode")
                if drive_mode & 0x04:
                    raise RuntimeError("Retract limit requires a velocity-based profile")
                _retract_stopping_counts(0)  # Validate braking configuration.
                reference, result, error = packet.read4ByteTxRx(
                    port, config.WINCH_ID, config.ADDR_PRESENT_POSITION
                )
                check_result(packet, result, error, "Capture winch retract limit")
                actual_velocity, result, error = packet.read4ByteTxRx(
                    port, config.WINCH_ID, config.ADDR_PRESENT_VELOCITY
                )
                check_result(packet, result, error, "Check winch is stationary")
                if signed_32(actual_velocity) != 0:
                    raise RuntimeError("Winch must be stationary when setting the retract limit")
                retract_limit_position = signed_32(reference)
                last_winch_position = retract_limit_position
                retract_limit_reached = True
                retract_limit_fault = None
                _retract_stop_pending = False
                _start_retract_limit_monitor()

                direction = 0
                speed_level = 0
                current_velocity = 0
                initialized = True
                torque_enabled = False
                lock_state = "locked"

                return get_motor_state()

            finally:
                port.closePort()

    except Exception as exc:
        retract_limit_position = None
        initialized = False
        torque_enabled = False
        lock_state = "locked"

        return {
            "success": False,
            "error": str(exc),
        }



# ============================================================
# ENABLE TORQUE
# ============================================================

@serialized_control
def enable_torque() -> dict:
    global torque_enabled

    try:
        if not initialized:
            raise RuntimeError(
                "Initialize the system before enabling torque"
            )

        _require_limit_reference()

        with bus_lock:
            port = open_bus()
            packet = PacketHandler(config.PROTOCOL_VERSION)

            try:
                set_bus_baudrate(
                    port,
                    config.WINCH_BAUDRATE,
                )

                comm_result, dxl_error = (
                    packet.write1ByteTxRx(
                        port,
                        config.WINCH_ID,
                        config.ADDR_TORQUE_ENABLE,
                        config.TORQUE_ENABLE,
                    )
                )
                check_result(
                    packet,
                    comm_result,
                    dxl_error,
                    "Enable winch torque",
                )

            finally:
                port.closePort()

        torque_enabled = True
        return get_motor_state()

    except Exception as exc:
        return {
            "success": False,
            "error": str(exc),
        }



# ============================================================
# DISABLE TORQUE
# ============================================================

@serialized_control
def disable_torque() -> dict:
    """
    Stop and disable winch torque.

    The lock motor is also released so the spring engages the
    mechanical lock.
    """

    global direction
    global speed_level
    global current_velocity
    global torque_enabled
    global lock_state

    global _retract_stop_pending

    try:
        if not initialized:
            raise RuntimeError(
                "System is not initialized"
            )

        with bus_lock:
            port = open_bus()
            packet = PacketHandler(config.PROTOCOL_VERSION)

            try:
                # Stop winch.
                set_bus_baudrate(
                    port,
                    config.WINCH_BAUDRATE,
                )

                comm_result, dxl_error = (
                    packet.write4ByteTxRx(
                        port,
                        config.WINCH_ID,
                        config.ADDR_GOAL_VELOCITY,
                        0,
                    )
                )
                check_result(
                    packet,
                    comm_result,
                    dxl_error,
                    "Stop winch motor",
                )

                # Disable winch torque.
                comm_result, dxl_error = (
                    packet.write1ByteTxRx(
                        port,
                        config.WINCH_ID,
                        config.ADDR_TORQUE_ENABLE,
                        config.TORQUE_DISABLE,
                    )
                )
                check_result(
                    packet,
                    comm_result,
                    dxl_error,
                    "Disable winch torque",
                )

                # Fail-safe: engage mechanical lock.
                set_bus_baudrate(
                    port,
                    config.LOCK_BAUDRATE,
                )

                comm_result, dxl_error = (
                    packet.write1ByteTxRx(
                        port,
                        config.LOCK_ID,
                        config.LOCK_ADDR_TORQUE_ENABLE,
                        config.TORQUE_DISABLE,
                    )
                )
                check_result(
                    packet,
                    comm_result,
                    dxl_error,
                    "Engage mechanical lock",
                )

            finally:
                port.closePort()

        _retract_stop_pending = False
        direction = 0
        speed_level = 0
        current_velocity = 0
        torque_enabled = False
        lock_state = "locked"

        return get_motor_state()

    except Exception as exc:
        return {
            "success": False,
            "error": str(exc),
        }



# ============================================================
# TOGGLE LOCK
# ============================================================

@serialized_control
def toggle_lock() -> dict:
    try:
        if not initialized:
            raise RuntimeError(
                "Initialize the system first"
            )

        if lock_state == "locked":
            unlock_mechanism()
        elif lock_state == "unlocked":
            lock_mechanism(
                stop_winch=True
            )
        else:
            raise RuntimeError(
                "Lock mechanism is busy"
            )

        return get_motor_state()

    except Exception as exc:
        return {
            "success": False,
            "error": str(exc),
            "lock_state": lock_state,
            "unlock_diagnostics": {
                "relief_motor_deg": config.UNLOCK_RELIEF_MOTOR_DEG,
                "relief_ratchet_deg": config.UNLOCK_RELIEF_MOTOR_DEG / 2.0,
                "lock_position_deg": last_unlock_lock_position_deg,
                "lock_current_a": last_unlock_lock_current_a,
                "position_error_deg": last_unlock_position_error_deg,
                "success": last_unlock_success,
            },
        }
