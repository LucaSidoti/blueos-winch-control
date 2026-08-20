#!/usr/bin/env python3

from pathlib import Path
from threading import Lock

from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    COMM_SUCCESS,
)
from litestar import Litestar, get, post
from litestar.static_files.config import StaticFilesConfig


# ============================================================
# DYNAMIXEL SETTINGS
# ============================================================

DEVICE_NAME = "/dev/ttyUSB0"
BAUDRATE = 1_000_000
PROTOCOL_VERSION = 2.0
DXL_ID = 7


# ============================================================
# XW540-T140-R CONTROL TABLE
# ============================================================

ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104
ADDR_PROFILE_ACCELERATION = 108

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
VELOCITY_MODE = 1

PROFILE_ACCELERATION = 1


# ============================================================
# WINCH SETTINGS
# ============================================================

# Dynamixel velocity unit = 0.229 RPM
DYNAMIXEL_RPM_PER_UNIT = 0.229

SPEED_LEVELS = [20, 40, 60, 80, 100]


# ============================================================
# WINCH STATE
# ============================================================

# direction:
#   -1 = retract
#    0 = stopped
#   +1 = deploy

direction = 0
speed_level = 0
current_velocity = 0

# Prevent simultaneous requests from trying to access
# the serial port at the same time.
motor_lock = Lock()


# ============================================================
# DYNAMIXEL HELPERS
# ============================================================

def open_motor_port():
    """
    Open the Dynamixel serial port and configure the baud rate.

    Returns:
        (port, packet)
    """
    if not Path(DEVICE_NAME).exists():
        raise RuntimeError(f"{DEVICE_NAME} does not exist")

    port = PortHandler(DEVICE_NAME)
    packet = PacketHandler(PROTOCOL_VERSION)

    if not port.openPort():
        raise RuntimeError(f"Could not open {DEVICE_NAME}")

    if not port.setBaudRate(BAUDRATE):
        port.closePort()
        raise RuntimeError(
            f"Could not set baud rate to {BAUDRATE}"
        )

    return port, packet


def check_result(packet, comm_result, dxl_error, action):
    """
    Check the result of a Dynamixel communication operation.
    """
    if comm_result != COMM_SUCCESS:
        raise RuntimeError(
            f"{action}: {packet.getTxRxResult(comm_result)}"
        )

    if dxl_error != 0:
        raise RuntimeError(
            f"{action}: {packet.getRxPacketError(dxl_error)}"
        )


def write_velocity(velocity: int):
    """
    Send a signed goal velocity to the Dynamixel.
    """
    global current_velocity

    with motor_lock:
        port, packet = open_motor_port()

        try:
            # Dynamixel expects the signed velocity value
            # as a 32-bit register value.
            command = velocity & 0xFFFFFFFF

            comm_result, dxl_error = packet.write4ByteTxRx(
                port,
                DXL_ID,
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
    """
    Return the current software-side winch state.
    """
    rpm = abs(current_velocity) * DYNAMIXEL_RPM_PER_UNIT

    if direction == -1:
        status_text = "retracting"
    elif direction == 1:
        status_text = "deploying"
    else:
        status_text = "stopped"

    return {
        "success": True,
        "status": status_text,
        "direction": direction,
        "speed_level": (
            speed_level + 1 if direction != 0 else 0
        ),
        "max_speed_level": len(SPEED_LEVELS),
        "velocity": current_velocity,
        "rpm": round(rpm, 1),
    }


# ============================================================
# GENERAL STATUS
# ============================================================

@get("/status", sync_to_thread=False)
def status() -> dict:
    return {
        "extension": "winch-control",
        "device": DEVICE_NAME,
        "device_exists": Path(DEVICE_NAME).exists(),
        "baudrate": BAUDRATE,
        "protocol_version": PROTOCOL_VERSION,
        "motor_id": DXL_ID,
    }


# ============================================================
# MOTOR PING
# ============================================================

@get("/motor/ping", sync_to_thread=True)
def ping_motor() -> dict:
    try:
        with motor_lock:
            port, packet = open_motor_port()

            try:
                model_number, comm_result, dxl_error = packet.ping(
                    port,
                    DXL_ID,
                )

                check_result(
                    packet,
                    comm_result,
                    dxl_error,
                    "Ping motor",
                )

                return {
                    "success": True,
                    "connected": True,
                    "device": DEVICE_NAME,
                    "baudrate": BAUDRATE,
                    "protocol_version": PROTOCOL_VERSION,
                    "motor_id": DXL_ID,
                    "model_number": model_number,
                }

            finally:
                port.closePort()

    except Exception as exc:
        return {
            "success": False,
            "connected": False,
            "error": str(exc),
        }


# ============================================================
# MOTOR INITIALIZATION
# ============================================================

@post("/motor/initialize", sync_to_thread=True)
def initialize_motor() -> dict:
    """
    Configure the Dynamixel for velocity control.

    Torque remains disabled when initialization is complete.
    """
    global direction
    global speed_level
    global current_velocity

    try:
        with motor_lock:
            port, packet = open_motor_port()

            try:
                # ------------------------------------------------
                # Disable torque before changing operating mode
                # ------------------------------------------------

                comm_result, dxl_error = packet.write1ByteTxRx(
                    port,
                    DXL_ID,
                    ADDR_TORQUE_ENABLE,
                    TORQUE_DISABLE,
                )

                check_result(
                    packet,
                    comm_result,
                    dxl_error,
                    "Disable torque",
                )

                # ------------------------------------------------
                # Set Velocity Control Mode
                # ------------------------------------------------

                comm_result, dxl_error = packet.write1ByteTxRx(
                    port,
                    DXL_ID,
                    ADDR_OPERATING_MODE,
                    VELOCITY_MODE,
                )

                check_result(
                    packet,
                    comm_result,
                    dxl_error,
                    "Set velocity mode",
                )

                # ------------------------------------------------
                # Set acceleration profile
                # ------------------------------------------------

                comm_result, dxl_error = packet.write4ByteTxRx(
                    port,
                    DXL_ID,
                    ADDR_PROFILE_ACCELERATION,
                    PROFILE_ACCELERATION,
                )

                check_result(
                    packet,
                    comm_result,
                    dxl_error,
                    "Set profile acceleration",
                )

                direction = 0
                speed_level = 0
                current_velocity = 0

                return {
                    "success": True,
                    "motor_id": DXL_ID,
                    "operating_mode": "velocity",
                    "profile_acceleration": PROFILE_ACCELERATION,
                    "torque_enabled": False,
                }

            finally:
                port.closePort()

    except Exception as exc:
        return {
            "success": False,
            "error": str(exc),
        }


# ============================================================
# TORQUE CONTROL
# ============================================================

@post("/motor/torque/enable", sync_to_thread=True)
def enable_torque() -> dict:
    try:
        with motor_lock:
            port, packet = open_motor_port()

            try:
                comm_result, dxl_error = packet.write1ByteTxRx(
                    port,
                    DXL_ID,
                    ADDR_TORQUE_ENABLE,
                    TORQUE_ENABLE,
                )

                check_result(
                    packet,
                    comm_result,
                    dxl_error,
                    "Enable torque",
                )

                return {
                    "success": True,
                    "motor_id": DXL_ID,
                    "torque_enabled": True,
                }

            finally:
                port.closePort()

    except Exception as exc:
        return {
            "success": False,
            "error": str(exc),
        }


@post("/motor/torque/disable", sync_to_thread=True)
def disable_torque() -> dict:
    global direction
    global speed_level
    global current_velocity

    try:
        with motor_lock:
            port, packet = open_motor_port()

            try:
                # Stop before disabling torque.
                comm_result, dxl_error = packet.write4ByteTxRx(
                    port,
                    DXL_ID,
                    ADDR_GOAL_VELOCITY,
                    0,
                )

                check_result(
                    packet,
                    comm_result,
                    dxl_error,
                    "Stop motor",
                )

                comm_result, dxl_error = packet.write1ByteTxRx(
                    port,
                    DXL_ID,
                    ADDR_TORQUE_ENABLE,
                    TORQUE_DISABLE,
                )

                check_result(
                    packet,
                    comm_result,
                    dxl_error,
                    "Disable torque",
                )

                direction = 0
                speed_level = 0
                current_velocity = 0

                return {
                    "success": True,
                    "motor_id": DXL_ID,
                    "torque_enabled": False,
                }

            finally:
                port.closePort()

    except Exception as exc:
        return {
            "success": False,
            "error": str(exc),
        }


# ============================================================
# MOTOR STATE
# ============================================================

@get("/motor/state", sync_to_thread=False)
def motor_state() -> dict:
    return get_motor_state()


# ============================================================
# STOP
# ============================================================

@post("/motor/stop", sync_to_thread=True)
def stop_motor() -> dict:
    global direction
    global speed_level

    try:
        write_velocity(0)

        direction = 0
        speed_level = 0

        return get_motor_state()

    except Exception as exc:
        return {
            "success": False,
            "error": str(exc),
        }


# ============================================================
# RETRACT
# ============================================================

@post("/motor/retract", sync_to_thread=True)
def command_retract() -> dict:
    global direction
    global speed_level

    try:

        # --------------------------------------------------------
        # Already retracting
        # Increase retract speed
        # --------------------------------------------------------

        if direction == -1:
            if speed_level < len(SPEED_LEVELS) - 1:
                speed_level += 1

            velocity = -SPEED_LEVELS[speed_level]
            write_velocity(velocity)

        # --------------------------------------------------------
        # Currently deploying
        # Reduce deployment speed first
        # --------------------------------------------------------

        elif direction == 1:
            if speed_level > 0:
                speed_level -= 1

                velocity = SPEED_LEVELS[speed_level]
                write_velocity(velocity)

            else:
                # At minimum deployment speed:
                # opposite command stops the winch.
                return stop_motor()

        # --------------------------------------------------------
        # Currently stopped
        # Start retracting at speed level 1
        # --------------------------------------------------------

        else:
            direction = -1
            speed_level = 0

            velocity = -SPEED_LEVELS[speed_level]
            write_velocity(velocity)

        return get_motor_state()

    except Exception as exc:
        return {
            "success": False,
            "error": str(exc),
        }


# ============================================================
# DEPLOY
# ============================================================

@post("/motor/deploy", sync_to_thread=True)
def command_deploy() -> dict:
    global direction
    global speed_level

    try:

        # --------------------------------------------------------
        # Already deploying
        # Increase deployment speed
        # --------------------------------------------------------

        if direction == 1:
            if speed_level < len(SPEED_LEVELS) - 1:
                speed_level += 1

            velocity = SPEED_LEVELS[speed_level]
            write_velocity(velocity)

        # --------------------------------------------------------
        # Currently retracting
        # Reduce retract speed first
        # --------------------------------------------------------

        elif direction == -1:
            if speed_level > 0:
                speed_level -= 1

                velocity = -SPEED_LEVELS[speed_level]
                write_velocity(velocity)

            else:
                # At minimum retract speed:
                # opposite command stops the winch.
                return stop_motor()

        # --------------------------------------------------------
        # Currently stopped
        # Start deploying at speed level 1
        # --------------------------------------------------------

        else:
            direction = 1
            speed_level = 0

            velocity = SPEED_LEVELS[speed_level]
            write_velocity(velocity)

        return get_motor_state()

    except Exception as exc:
        return {
            "success": False,
            "error": str(exc),
        }


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
        motor_state,
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
