#!/usr/bin/env python3

from pathlib import Path
from threading import Lock, Thread
import time

from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    COMM_SUCCESS,
)

from litestar import Litestar, get, post
from litestar.static_files.config import StaticFilesConfig
from pymavlink import mavutil


# ============================================================
# USB / PROTOCOL SETTINGS
# ============================================================

DEVICE_NAME = "/dev/ttyUSB0"
PROTOCOL_VERSION = 2.0


# ============================================================
# WINCH MOTOR - XW540-T140-R
# ============================================================

WINCH_ID = 7
WINCH_BAUDRATE = 1_000_000

ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104
ADDR_PROFILE_ACCELERATION = 108
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_INPUT_VOLTAGE = 144
ADDR_PRESENT_TEMPERATURE = 146
ADDR_PRESENT_POSITION = 132

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
VELOCITY_MODE = 1

WINCH_PROFILE_ACCELERATION = 1
DYNAMIXEL_RPM_PER_UNIT = 0.229
SPEED_LEVELS = [20, 40, 60, 80, 100]

PRESENT_CURRENT_MA_PER_UNIT = 2.69
PRESENT_VELOCITY_RPM_PER_UNIT = 0.229
PRESENT_VOLTAGE_V_PER_UNIT = 0.1
POSITION_DEG_PER_COUNT = 360.0 / 4096.0


# ============================================================
# LOCK MOTOR - XW430-T200-R
# ============================================================

LOCK_ID = 0
LOCK_BAUDRATE = 115_200

LOCK_ADDR_OPERATING_MODE = 11
LOCK_ADDR_TORQUE_ENABLE = 64
LOCK_ADDR_PROFILE_ACCELERATION = 108
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_INPUT_VOLTAGE = 144
ADDR_PRESENT_TEMPERATURE = 146
LOCK_ADDR_PROFILE_VELOCITY = 112
LOCK_ADDR_GOAL_POSITION = 116

POSITION_MODE = 3

UNLOCK_POSITION_DEG = 205.0
UNLOCK_POSITION_RAW = round(
    UNLOCK_POSITION_DEG / 360.0 * 4096
)

LOCK_PROFILE_ACCELERATION = 5
LOCK_PROFILE_VELOCITY = 20
LOCK_COMMAND_DELAY = 0.6

# Ratchet/pawl load-relief movement before unlocking.
# The ratchet has 24 teeth -> 15 degrees per tooth.
# With a 2:1 reduction, 8 motor degrees = 4 ratchet degrees.
WINCH_COUNTS_PER_REV = 4096
UNLOCK_RELIEF_MOTOR_DEG = 12.0
UNLOCK_RELIEF_SETTLE_DELAY = 0.2
UNLOCK_RELIEF_COUNTS = round(
    WINCH_COUNTS_PER_REV * UNLOCK_RELIEF_MOTOR_DEG / 360.0
)
UNLOCK_RELIEF_VELOCITY = 20
UNLOCK_RELIEF_TIMEOUT = 1.0
UNLOCK_RELIEF_POLL_INTERVAL = 0.02


# ============================================================
# MAVLINK SETTINGS
# ============================================================

MAVLINK_PORT = 14560
SERVO_NUMBER = 10

PWM_RETRACT = 1100
PWM_STOP = 1300
PWM_IDLE = 1500
PWM_DEPLOY = 1900


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

bus_lock = Lock()


# ============================================================
# DYNAMIXEL BUS HELPERS
# ============================================================

def open_bus():
    """Open the shared U2D2 serial port."""

    if not Path(DEVICE_NAME).exists():
        raise RuntimeError(f"{DEVICE_NAME} does not exist")

    port = PortHandler(DEVICE_NAME)

    if not port.openPort():
        raise RuntimeError(f"Could not open {DEVICE_NAME}")

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

    # Each backend operation opens a fresh PortHandler, so always
    # configure the requested baud rate explicitly.
    if not port.setBaudRate(baudrate):
        raise RuntimeError(
            f"Could not set bus baud rate to {baudrate}"
        )


# ============================================================
# TELEMETRY HELPERS
# ============================================================

def signed_16(value):
    return value - 0x10000 if value & 0x8000 else value

def signed_32(value):
    return value - 0x100000000 if value & 0x80000000 else value

def read_motor_telemetry(port, packet, motor_id, baudrate):
    set_bus_baudrate(port, baudrate)

    raw_current, comm_result, dxl_error = packet.read2ByteTxRx(port, motor_id, ADDR_PRESENT_CURRENT)
    check_result(packet, comm_result, dxl_error, f"Read motor {motor_id} current")

    raw_velocity, comm_result, dxl_error = packet.read4ByteTxRx(port, motor_id, ADDR_PRESENT_VELOCITY)
    check_result(packet, comm_result, dxl_error, f"Read motor {motor_id} velocity")

    raw_position, comm_result, dxl_error = packet.read4ByteTxRx(port, motor_id, ADDR_PRESENT_POSITION)
    check_result(packet, comm_result, dxl_error, f"Read motor {motor_id} position")

    raw_voltage, comm_result, dxl_error = packet.read2ByteTxRx(port, motor_id, ADDR_PRESENT_INPUT_VOLTAGE)
    check_result(packet, comm_result, dxl_error, f"Read motor {motor_id} voltage")

    raw_temperature, comm_result, dxl_error = packet.read1ByteTxRx(port, motor_id, ADDR_PRESENT_TEMPERATURE)
    check_result(packet, comm_result, dxl_error, f"Read motor {motor_id} temperature")

    return {
        "current_a": round(signed_16(raw_current) * PRESENT_CURRENT_MA_PER_UNIT / 1000.0, 3),
        "rpm": round(signed_32(raw_velocity) * PRESENT_VELOCITY_RPM_PER_UNIT, 2),
        "position_deg": round(signed_32(raw_position) * POSITION_DEG_PER_COUNT, 1),
        "voltage_v": round(raw_voltage * PRESENT_VOLTAGE_V_PER_UNIT, 1),
        "temperature_c": int(raw_temperature),
    }

def read_all_telemetry():
    with bus_lock:
        port = open_bus()
        packet = PacketHandler(PROTOCOL_VERSION)
        try:
            winch = read_motor_telemetry(port, packet, WINCH_ID, WINCH_BAUDRATE)
            lock = read_motor_telemetry(port, packet, LOCK_ID, LOCK_BAUDRATE)
        finally:
            port.closePort()
    return {"success": True, "winch": winch, "lock": lock}


# ============================================================
# WINCH MOTOR HELPERS
# ============================================================

def write_velocity(velocity: int):
    """
    Send a signed goal velocity to the XW540.

    The reduction gear reverses the physical direction:
      RETRACT -> positive motor velocity
      DEPLOY  -> negative motor velocity
    """

    global current_velocity

    with bus_lock:
        port = open_bus()
        packet = PacketHandler(PROTOCOL_VERSION)

        try:
            set_bus_baudrate(port, WINCH_BAUDRATE)

            command = velocity & 0xFFFFFFFF

            comm_result, dxl_error = packet.write4ByteTxRx(
                port,
                WINCH_ID,
                ADDR_GOAL_VELOCITY,
                command,
            )

            check_result(
                packet,
                comm_result,
                dxl_error,
                "Set goal velocity",
            )

            current_velocity = velocity

        finally:
            port.closePort()


def get_motor_state() -> dict:
    """Return the current application-side winch state."""

    rpm = abs(current_velocity) * DYNAMIXEL_RPM_PER_UNIT

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
        "max_speed_level": len(SPEED_LEVELS),
        "velocity": current_velocity,
        "rpm": round(rpm, 1),
    }


def require_winch_ready():
    """Prevent movement unless the system is ready and unlocked."""

    if not initialized:
        raise RuntimeError("System is not initialized")

    if not torque_enabled:
        raise RuntimeError("Motor torque is not enabled")

    if lock_state != "unlocked":
        raise RuntimeError("Mechanical lock is engaged")


# ============================================================
# RATCHET LOAD RELIEF
# ============================================================

def relieve_pawl_load():
    """
    Rotate the XW540 slightly in the RETRACT direction before
    lifting the pawl.

    The winch must already have torque enabled. The movement is
    measured from the XW540 Present Position register so it does
    not depend on a fixed sleep duration.
    """

    global current_velocity

    if not initialized:
        raise RuntimeError("Initialize the system first")

    if not torque_enabled:
        raise RuntimeError(
            "Enable winch torque before unlocking the mechanism"
        )

    # Extra safety: ensure the winch is stationary before the relief move.
    write_velocity(0)

    with bus_lock:
        port = open_bus()
        packet = PacketHandler(PROTOCOL_VERSION)
        motion_started = False

        try:
            set_bus_baudrate(port, WINCH_BAUDRATE)

            start_position, comm_result, dxl_error = packet.read4ByteTxRx(
                port,
                WINCH_ID,
                ADDR_PRESENT_POSITION,
            )
            check_result(
                packet,
                comm_result,
                dxl_error,
                "Read winch start position",
            )

            # RETRACT is positive motor velocity for this gearbox.
            command = UNLOCK_RELIEF_VELOCITY & 0xFFFFFFFF
            comm_result, dxl_error = packet.write4ByteTxRx(
                port,
                WINCH_ID,
                ADDR_GOAL_VELOCITY,
                command,
            )
            check_result(
                packet,
                comm_result,
                dxl_error,
                "Start ratchet load-relief movement",
            )

            motion_started = True
            current_velocity = UNLOCK_RELIEF_VELOCITY
            deadline = time.monotonic() + UNLOCK_RELIEF_TIMEOUT

            while True:
                position, comm_result, dxl_error = packet.read4ByteTxRx(
                    port,
                    WINCH_ID,
                    ADDR_PRESENT_POSITION,
                )
                check_result(
                    packet,
                    comm_result,
                    dxl_error,
                    "Read winch relief position",
                )

                movement = (position - start_position) & 0xFFFFFFFF

                if movement >= UNLOCK_RELIEF_COUNTS:
                    break

                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        "Ratchet load-relief movement timed out"
                    )

                time.sleep(UNLOCK_RELIEF_POLL_INTERVAL)

        finally:
            if motion_started:
                try:
                    comm_result, dxl_error = packet.write4ByteTxRx(
                        port,
                        WINCH_ID,
                        ADDR_GOAL_VELOCITY,
                        0,
                    )
                    check_result(
                        packet,
                        comm_result,
                        dxl_error,
                        "Stop ratchet load-relief movement",
                    )
                finally:
                    current_velocity = 0

            port.closePort()


# ============================================================
# LOCK MOTOR HELPERS
# ============================================================

def initialize_lock_motor(port, packet):
    """
    Configure the XW430 in Position Control Mode.

    Initialization finishes with torque OFF so the spring can
    keep the reduction gear mechanically locked.
    """

    global lock_state

    set_bus_baudrate(port, LOCK_BAUDRATE)

    comm_result, dxl_error = packet.write1ByteTxRx(
        port,
        LOCK_ID,
        LOCK_ADDR_TORQUE_ENABLE,
        TORQUE_DISABLE,
    )
    check_result(
        packet,
        comm_result,
        dxl_error,
        "Disable lock motor torque",
    )

    comm_result, dxl_error = packet.write1ByteTxRx(
        port,
        LOCK_ID,
        LOCK_ADDR_OPERATING_MODE,
        POSITION_MODE,
    )
    check_result(
        packet,
        comm_result,
        dxl_error,
        "Set lock motor position mode",
    )

    comm_result, dxl_error = packet.write4ByteTxRx(
        port,
        LOCK_ID,
        LOCK_ADDR_PROFILE_ACCELERATION,
        LOCK_PROFILE_ACCELERATION,
    )
    check_result(
        packet,
        comm_result,
        dxl_error,
        "Set lock motor profile acceleration",
    )

    comm_result, dxl_error = packet.write4ByteTxRx(
        port,
        LOCK_ID,
        LOCK_ADDR_PROFILE_VELOCITY,
        LOCK_PROFILE_VELOCITY,
    )
    check_result(
        packet,
        comm_result,
        dxl_error,
        "Set lock motor profile velocity",
    )

    lock_state = "locked"


def unlock_mechanism():
    """
    Unload the pawl, then command the XW430 to the unlock position.

    The XW540 torque must be enabled first. The ratchet is moved
    approximately 4 degrees at the ratchet (8 degrees at the motor)
    in the RETRACT direction before the pawl is lifted.
    """

    global lock_state

    if not initialized:
        raise RuntimeError("Initialize the system first")

    if not torque_enabled:
        raise RuntimeError(
            "Enable winch torque before unlocking the mechanism"
        )

    relieve_pawl_load()
    time.sleep(UNLOCK_RELIEF_SETTLE_DELAY)
    
    lock_state = "unlocking"

    try:
        with bus_lock:
            port = open_bus()
            packet = PacketHandler(PROTOCOL_VERSION)

            try:
                set_bus_baudrate(port, LOCK_BAUDRATE)

                comm_result, dxl_error = packet.write1ByteTxRx(
                    port,
                    LOCK_ID,
                    LOCK_ADDR_TORQUE_ENABLE,
                    TORQUE_ENABLE,
                )
                check_result(
                    packet,
                    comm_result,
                    dxl_error,
                    "Enable lock motor torque",
                )

                comm_result, dxl_error = packet.write4ByteTxRx(
                    port,
                    LOCK_ID,
                    LOCK_ADDR_GOAL_POSITION,
                    UNLOCK_POSITION_RAW,
                )
                check_result(
                    packet,
                    comm_result,
                    dxl_error,
                    "Command unlock position",
                )

            finally:
                port.closePort()

        time.sleep(LOCK_COMMAND_DELAY)
        lock_state = "unlocked"

    except Exception:
        lock_state = "locked"
        raise


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
        packet = PacketHandler(PROTOCOL_VERSION)

        try:
            set_bus_baudrate(port, LOCK_BAUDRATE)

            comm_result, dxl_error = packet.write1ByteTxRx(
                port,
                LOCK_ID,
                LOCK_ADDR_TORQUE_ENABLE,
                TORQUE_DISABLE,
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

def execute_stop() -> dict:
    """Stop the winch and reset motion state."""

    global direction
    global speed_level

    write_velocity(0)
    direction = 0
    speed_level = 0

    return get_motor_state()


def execute_retract() -> dict:
    """Retract the cable; repeated commands increase retract speed."""

    global direction
    global speed_level

    require_winch_ready()

    if direction == -1:
        if speed_level < len(SPEED_LEVELS) - 1:
            speed_level += 1

        # Reversed by the reduction gear.
        velocity = SPEED_LEVELS[speed_level]
        write_velocity(velocity)

    elif direction == 1:
        if speed_level > 0:
            speed_level -= 1

            # Still physically deploying until speed reaches zero.
            velocity = -SPEED_LEVELS[speed_level]
            write_velocity(velocity)
        else:
            return execute_stop()

    else:
        direction = -1
        speed_level = 0

        # Reversed by the reduction gear.
        velocity = SPEED_LEVELS[speed_level]
        write_velocity(velocity)

    return get_motor_state()


def execute_deploy() -> dict:
    """Deploy the cable; repeated commands increase deploy speed."""

    global direction
    global speed_level

    require_winch_ready()

    if direction == 1:
        if speed_level < len(SPEED_LEVELS) - 1:
            speed_level += 1

        # Reversed by the reduction gear.
        velocity = -SPEED_LEVELS[speed_level]
        write_velocity(velocity)

    elif direction == -1:
        if speed_level > 0:
            speed_level -= 1

            # Still physically retracting until speed reaches zero.
            velocity = SPEED_LEVELS[speed_level]
            write_velocity(velocity)
        else:
            return execute_stop()

    else:
        direction = 1
        speed_level = 0

        # Reversed by the reduction gear.
        velocity = -SPEED_LEVELS[speed_level]
        write_velocity(velocity)

    return get_motor_state()


# ============================================================
# GENERAL STATUS
# ============================================================

@get("/status", sync_to_thread=False)
def status() -> dict:
    return {
        "extension": "winch-control",
        "device": DEVICE_NAME,
        "device_exists": Path(DEVICE_NAME).exists(),
        "protocol_version": PROTOCOL_VERSION,
        "winch_motor_id": WINCH_ID,
        "winch_baudrate": WINCH_BAUDRATE,
        "lock_motor_id": LOCK_ID,
        "lock_baudrate": LOCK_BAUDRATE,
        "mavlink_port": MAVLINK_PORT,
    }


# ============================================================
# SYSTEM PING
# ============================================================

@get("/motor/ping", sync_to_thread=True)
def ping_motor() -> dict:
    """Ping both Dynamixels on the shared U2D2 connection."""

    try:
        with bus_lock:
            port = open_bus()
            packet = PacketHandler(PROTOCOL_VERSION)

            try:
                set_bus_baudrate(port, WINCH_BAUDRATE)

                winch_model, comm_result, dxl_error = packet.ping(
                    port,
                    WINCH_ID,
                )
                check_result(
                    packet,
                    comm_result,
                    dxl_error,
                    "Ping winch motor",
                )

                set_bus_baudrate(port, LOCK_BAUDRATE)

                lock_model, comm_result, dxl_error = packet.ping(
                    port,
                    LOCK_ID,
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
                    "device": DEVICE_NAME,
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
# SYSTEM INITIALIZATION
# ============================================================

@post("/motor/initialize", sync_to_thread=True)
def initialize_motor() -> dict:
    """Initialize both Dynamixels and leave the system safely locked."""

    global direction
    global speed_level
    global current_velocity
    global initialized
    global torque_enabled
    global lock_state

    try:
        with bus_lock:
            port = open_bus()
            packet = PacketHandler(PROTOCOL_VERSION)

            try:
                # ---------------- WINCH MOTOR ----------------
                set_bus_baudrate(port, WINCH_BAUDRATE)

                comm_result, dxl_error = packet.write1ByteTxRx(
                    port,
                    WINCH_ID,
                    ADDR_TORQUE_ENABLE,
                    TORQUE_DISABLE,
                )
                check_result(
                    packet,
                    comm_result,
                    dxl_error,
                    "Disable winch torque",
                )

                comm_result, dxl_error = packet.write1ByteTxRx(
                    port,
                    WINCH_ID,
                    ADDR_OPERATING_MODE,
                    VELOCITY_MODE,
                )
                check_result(
                    packet,
                    comm_result,
                    dxl_error,
                    "Set winch velocity mode",
                )

                comm_result, dxl_error = packet.write4ByteTxRx(
                    port,
                    WINCH_ID,
                    ADDR_PROFILE_ACCELERATION,
                    WINCH_PROFILE_ACCELERATION,
                )
                check_result(
                    packet,
                    comm_result,
                    dxl_error,
                    "Set winch profile acceleration",
                )

                comm_result, dxl_error = packet.write4ByteTxRx(
                    port,
                    WINCH_ID,
                    ADDR_GOAL_VELOCITY,
                    0,
                )
                check_result(
                    packet,
                    comm_result,
                    dxl_error,
                    "Set winch zero velocity",
                )

                # ---------------- LOCK MOTOR -----------------
                initialize_lock_motor(port, packet)

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
        initialized = False
        torque_enabled = False
        lock_state = "locked"

        return {
            "success": False,
            "error": str(exc),
        }


# ============================================================
# TORQUE ENABLE
# ============================================================

@post("/motor/torque/enable", sync_to_thread=True)
def enable_torque() -> dict:
    global torque_enabled

    try:
        if not initialized:
            raise RuntimeError(
                "Initialize the system before enabling torque"
            )

        with bus_lock:
            port = open_bus()
            packet = PacketHandler(PROTOCOL_VERSION)

            try:
                set_bus_baudrate(port, WINCH_BAUDRATE)

                comm_result, dxl_error = packet.write1ByteTxRx(
                    port,
                    WINCH_ID,
                    ADDR_TORQUE_ENABLE,
                    TORQUE_ENABLE,
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
# TORQUE DISABLE
# ============================================================

@post("/motor/torque/disable", sync_to_thread=True)
def disable_torque() -> dict:
    """
    Stop and disable winch torque.

    The lock motor is also released so the spring engages the
    mechanical lock. This prevents an unlocked gearbox with the
    winch motor unpowered.
    """

    global direction
    global speed_level
    global current_velocity
    global torque_enabled
    global lock_state

    try:
        if not initialized:
            raise RuntimeError("System is not initialized")

        with bus_lock:
            port = open_bus()
            packet = PacketHandler(PROTOCOL_VERSION)

            try:
                # Stop winch.
                set_bus_baudrate(port, WINCH_BAUDRATE)

                comm_result, dxl_error = packet.write4ByteTxRx(
                    port,
                    WINCH_ID,
                    ADDR_GOAL_VELOCITY,
                    0,
                )
                check_result(
                    packet,
                    comm_result,
                    dxl_error,
                    "Stop winch motor",
                )

                # Disable winch torque.
                comm_result, dxl_error = packet.write1ByteTxRx(
                    port,
                    WINCH_ID,
                    ADDR_TORQUE_ENABLE,
                    TORQUE_DISABLE,
                )
                check_result(
                    packet,
                    comm_result,
                    dxl_error,
                    "Disable winch torque",
                )

                # Fail-safe: engage mechanical lock.
                set_bus_baudrate(port, LOCK_BAUDRATE)

                comm_result, dxl_error = packet.write1ByteTxRx(
                    port,
                    LOCK_ID,
                    LOCK_ADDR_TORQUE_ENABLE,
                    TORQUE_DISABLE,
                )
                check_result(
                    packet,
                    comm_result,
                    dxl_error,
                    "Engage mechanical lock",
                )

            finally:
                port.closePort()

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
# LOCK TOGGLE
# ============================================================

@post("/lock/toggle", sync_to_thread=True)
def toggle_lock() -> dict:
    try:
        if not initialized:
            raise RuntimeError("Initialize the system first")

        if lock_state == "locked":
            unlock_mechanism()
        elif lock_state == "unlocked":
            lock_mechanism(stop_winch=True)
        else:
            raise RuntimeError("Lock mechanism is busy")

        return get_motor_state()

    except Exception as exc:
        return {
            "success": False,
            "error": str(exc),
        }


# ============================================================
# MOTOR STATE / MOTION HTTP ENDPOINTS
# ============================================================

@get("/motor/telemetry", sync_to_thread=True)
def motor_telemetry() -> dict:
    try:
        return read_all_telemetry()
    except Exception as exc:
        return {"success": False, "error": str(exc)}


@get("/motor/state", sync_to_thread=False)
def motor_state() -> dict:
    return get_motor_state()


@post("/motor/stop", sync_to_thread=True)
def stop_motor() -> dict:
    try:
        return execute_stop()
    except Exception as exc:
        return {
            "success": False,
            "error": str(exc),
        }


@post("/motor/retract", sync_to_thread=True)
def command_retract() -> dict:
    try:
        return execute_retract()
    except Exception as exc:
        return {
            "success": False,
            "error": str(exc),
        }


@post("/motor/deploy", sync_to_thread=True)
def command_deploy() -> dict:
    try:
        return execute_deploy()
    except Exception as exc:
        return {
            "success": False,
            "error": str(exc),
        }


# ============================================================
# MAVLINK HELPERS
# ============================================================

def reset_servo_signal(
    connection,
    target_system,
    target_component,
):
    """Reset SERVO10 to 1500 so the next D-pad press is detectable."""

    connection.mav.command_long_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        SERVO_NUMBER,
        PWM_IDLE,
        0,
        0,
        0,
        0,
        0,
    )


# ============================================================
# MAVLINK LISTENER
# ============================================================

def mavlink_listener():
    """
    Xbox/QGroundControl command bridge via SERVO10 telemetry.

      1100 -> Retract / faster
      1300 -> Stop
      1500 -> Idle
      1900 -> Deploy / faster
    """

    print(
        f"MAVLink winch listener starting on UDP port {MAVLINK_PORT}",
        flush=True,
    )

    try:
        connection = mavutil.mavlink_connection(
            f"udpin:0.0.0.0:{MAVLINK_PORT}"
        )
    except Exception as exc:
        print(
            f"Could not start MAVLink listener: {exc}",
            flush=True,
        )
        return

    command_ready = False

    while True:
        try:
            message = connection.recv_match(
                type="SERVO_OUTPUT_RAW",
                blocking=True,
                timeout=1,
            )

            if message is None:
                continue

            target_system = message.get_srcSystem()
            target_component = message.get_srcComponent()
            pwm = message.servo10_raw

            if not command_ready:
                if pwm == PWM_IDLE:
                    command_ready = True
                    print(
                        "MAVLink winch control ready",
                        flush=True,
                    )
                else:
                    reset_servo_signal(
                        connection,
                        target_system,
                        target_component,
                    )
                continue

            if pwm == PWM_IDLE:
                continue

            command_ready = False

            if pwm == PWM_RETRACT:
                print("Xbox command: RETRACT", flush=True)
                try:
                    result = execute_retract()
                    print(
                        f"Winch state: {result['status']}, "
                        f"speed level {result['speed_level']}",
                        flush=True,
                    )
                except Exception as exc:
                    print(
                        f"Retract command rejected: {exc}",
                        flush=True,
                    )

            elif pwm == PWM_STOP:
                print("Xbox command: STOP", flush=True)
                try:
                    execute_stop()
                    print("Winch stopped", flush=True)
                except Exception as exc:
                    print(
                        f"Stop command failed: {exc}",
                        flush=True,
                    )

            elif pwm == PWM_DEPLOY:
                print("Xbox command: DEPLOY", flush=True)
                try:
                    result = execute_deploy()
                    print(
                        f"Winch state: {result['status']}, "
                        f"speed level {result['speed_level']}",
                        flush=True,
                    )
                except Exception as exc:
                    print(
                        f"Deploy command rejected: {exc}",
                        flush=True,
                    )

            else:
                print(
                    f"SERVO10 value ignored: {pwm}",
                    flush=True,
                )

            reset_servo_signal(
                connection,
                target_system,
                target_component,
            )

        except Exception as exc:
            print(
                f"MAVLink listener error: {exc}",
                flush=True,
            )


# ============================================================
# START MAVLINK LISTENER
# ============================================================

mavlink_thread = Thread(
    target=mavlink_listener,
    daemon=True,
)
mavlink_thread.start()


# ============================================================
# APPLICATION
# ============================================================

app = Litestar(
    route_handlers=[
        status,
        ping_motor,
        initialize_motor,
        enable_torque,
        disable_torque,
        toggle_lock,
        motor_state,
        motor_telemetry,
        stop_motor,
        command_retract,
        command_deploy,
    ],
    static_files_config=[
        StaticFilesConfig(
            directories=["/app/static"],
            path="/",
            html_mode=True,
        )
    ],
)
