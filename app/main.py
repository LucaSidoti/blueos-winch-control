#!/usr/bin/env python3

from pathlib import Path

from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    COMM_SUCCESS,
)
from litestar import Litestar, get, post


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
ADDR_PROFILE_ACCELERATION = 108

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
VELOCITY_MODE = 1

PROFILE_ACCELERATION = 1


# ============================================================
# HELPERS
# ============================================================

def open_motor_port():
    port = PortHandler(DEVICE_NAME)
    packet = PacketHandler(PROTOCOL_VERSION)

    if not Path(DEVICE_NAME).exists():
        raise RuntimeError(f"{DEVICE_NAME} does not exist")

    if not port.openPort():
        raise RuntimeError(f"Could not open {DEVICE_NAME}")

    if not port.setBaudRate(BAUDRATE):
        port.closePort()
        raise RuntimeError(f"Could not set baud rate to {BAUDRATE}")

    return port, packet


def check_result(packet, comm_result, dxl_error, action):
    if comm_result != COMM_SUCCESS:
        raise RuntimeError(
            f"{action}: {packet.getTxRxResult(comm_result)}"
        )

    if dxl_error != 0:
        raise RuntimeError(
            f"{action}: {packet.getRxPacketError(dxl_error)}"
        )


# ============================================================
# STATUS
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
# PING
# ============================================================

@get("/motor/ping", sync_to_thread=True)
def ping_motor() -> dict:
    try:
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
# INITIALIZE MOTOR
# ============================================================

@post("/motor/initialize", sync_to_thread=True)
def initialize_motor() -> dict:
    try:
        port, packet = open_motor_port()

        try:
            # Disable torque before changing operating mode
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

            # Set Velocity Control Mode
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

            # Set profile acceleration
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
# TORQUE ENABLE
# ============================================================

@post("/motor/torque/enable", sync_to_thread=True)
def enable_torque() -> dict:
    try:
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


# ============================================================
# TORQUE DISABLE
# ============================================================

@post("/motor/torque/disable", sync_to_thread=True)
def disable_torque() -> dict:
    try:
        port, packet = open_motor_port()

        try:
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
# APPLICATION
# ============================================================

app = Litestar(
    route_handlers=[
        status,
        ping_motor,
        initialize_motor,
        enable_torque,
        disable_torque,
    ],
)
