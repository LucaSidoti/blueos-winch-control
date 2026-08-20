#!/usr/bin/env python3

from pathlib import Path
from threading import Lock, Thread

from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    COMM_SUCCESS,
)

from litestar import Litestar, get, post
from litestar.static_files.config import StaticFilesConfig

from pymavlink import mavutil


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

DYNAMIXEL_RPM_PER_UNIT = 0.229

SPEED_LEVELS = [20, 40, 60, 80, 100]


# ============================================================
# MAVLINK SETTINGS
# ============================================================

MAVLINK_PORT = 14560

# MAV_CMD_USER_1
WINCH_MAV_CMD = 31000

WINCH_RETRACT = 1
WINCH_DEPLOY = 2
WINCH_STOP = 3


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

initialized = False
torque_enabled = False

motor_lock = Lock()


# ============================================================
# DYNAMIXEL HELPERS
# ============================================================

def open_motor_port():
    """
    Open the Dynamixel serial port and configure the baud rate.
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
    Return the current application-side motor state.
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
        "initialized": initialized,
        "torque_enabled": torque_enabled,
        "status": status_text,
        "direction": direction,
        "speed_level": (
            speed_level + 1 if direction != 0 else 0
        ),
        "max_speed_level": len(SPEED_LEVELS),
        "velocity": current_velocity,
        "rpm": round(rpm, 1),
    }


def require_motor_ready():
    """
    Prevent movement unless the motor has been initialized
    and torque has been enabled.
    """

    if not initialized:
        raise RuntimeError(
            "Motor is not initialized"
        )

    if not torque_enabled:
        raise RuntimeError(
            "Motor torque is not enabled"
        )


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
        "mavlink_port": MAVLINK_PORT,
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

    global direction
    global speed_level
    global current_velocity
    global initialized
    global torque_enabled

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
                # Set velocity mode
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
                # Set profile acceleration
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

                # ------------------------------------------------
                # Ensure zero goal velocity
                # ------------------------------------------------

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
                    "Set zero velocity",
                )

                direction = 0
                speed_level = 0
                current_velocity = 0

                initialized = True
                torque_enabled = False

                return {
                    "success": True,
                    "initialized": True,
                    "torque_enabled": False,
                    "motor_id": DXL_ID,
                    "operating_mode": "velocity",
                    "profile_acceleration": PROFILE_ACCELERATION,
                }

            finally:

                port.closePort()

    except Exception as exc:

        initialized = False
        torque_enabled = False

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
                "Initialize the motor before enabling torque"
            )

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

                torque_enabled = True

                return {
                    "success": True,
                    "initialized": True,
                    "torque_enabled": True,
                    "motor_id": DXL_ID,
                }

            finally:

                port.closePort()

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

    global direction
    global speed_level
    global current_velocity
    global torque_enabled

    try:

        with motor_lock:

            port, packet = open_motor_port()

            try:

                # ------------------------------------------------
                # Stop first
                # ------------------------------------------------

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

                # ------------------------------------------------
                # Disable torque
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

                direction = 0
                speed_level = 0
                current_velocity = 0
                torque_enabled = False

                return {
                    "success": True,
                    "initialized": initialized,
                    "torque_enabled": False,
                    "motor_id": DXL_ID,
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

        require_motor_ready()

        # --------------------------------------------------------
        # Already retracting:
        # increase retract speed
        # --------------------------------------------------------

        if direction == -1:

            if speed_level < len(SPEED_LEVELS) - 1:
                speed_level += 1

            velocity = -SPEED_LEVELS[speed_level]

            write_velocity(velocity)

        # --------------------------------------------------------
        # Currently deploying:
        # reduce deploy speed first
        # --------------------------------------------------------

        elif direction == 1:

            if speed_level > 0:

                speed_level -= 1

                velocity = SPEED_LEVELS[speed_level]

                write_velocity(velocity)

            else:

                return stop_motor()

        # --------------------------------------------------------
        # Currently stopped:
        # start retracting at speed level 1
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

        require_motor_ready()

        # --------------------------------------------------------
        # Already deploying:
        # increase deploy speed
        # --------------------------------------------------------

        if direction == 1:

            if speed_level < len(SPEED_LEVELS) - 1:
                speed_level += 1

            velocity = SPEED_LEVELS[speed_level]

            write_velocity(velocity)

        # --------------------------------------------------------
        # Currently retracting:
        # reduce retract speed first
        # --------------------------------------------------------

        elif direction == -1:

            if speed_level > 0:

                speed_level -= 1

                velocity = -SPEED_LEVELS[speed_level]

                write_velocity(velocity)

            else:

                return stop_motor()

        # --------------------------------------------------------
        # Currently stopped:
        # start deploying at speed level 1
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
# MAVLINK LISTENER
# ============================================================

def mavlink_listener():
    """
    Listen for custom winch commands forwarded by the
    BlueOS MAVLink Router.

    IMPORTANT:
    This test version only prints received commands.
    It does not control the Dynamixel motor.
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


    while True:

        try:

            message = connection.recv_match(
                blocking=True,
                timeout=1,
            )

            if message is None:
                continue


            message_type = message.get_type()


            if message_type != "COMMAND_LONG":
                continue


            if message.command != WINCH_MAV_CMD:
                continue


            command = int(
                round(message.param1)
            )


            if command == WINCH_RETRACT:

                print(
                    "MAVLink winch command: RETRACT",
                    flush=True,
                )


            elif command == WINCH_DEPLOY:

                print(
                    "MAVLink winch command: DEPLOY",
                    flush=True,
                )


            elif command == WINCH_STOP:

                print(
                    "MAVLink winch command: STOP",
                    flush=True,
                )


            else:

                print(
                    f"MAVLink winch command: UNKNOWN ({command})",
                    flush=True,
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
