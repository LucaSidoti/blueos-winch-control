#!/usr/bin/env python3

from pathlib import Path

from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    COMM_SUCCESS,
)
from litestar import Litestar, get


DEVICE_NAME = "/dev/ttyUSB0"
BAUDRATE = 1_000_000
PROTOCOL_VERSION = 2.0
DXL_ID = 7


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


@get("/motor/ping", sync_to_thread=True)
def ping_motor() -> dict:
    port = PortHandler(DEVICE_NAME)
    packet = PacketHandler(PROTOCOL_VERSION)

    if not Path(DEVICE_NAME).exists():
        return {
            "connected": False,
            "error": f"{DEVICE_NAME} does not exist",
        }

    if not port.openPort():
        return {
            "connected": False,
            "error": f"Could not open {DEVICE_NAME}",
        }

    try:
        if not port.setBaudRate(BAUDRATE):
            return {
                "connected": False,
                "error": f"Could not set baud rate to {BAUDRATE}",
            }

        model_number, comm_result, dxl_error = packet.ping(
            port,
            DXL_ID,
        )

        if comm_result != COMM_SUCCESS:
            return {
                "connected": False,
                "motor_id": DXL_ID,
                "error": packet.getTxRxResult(comm_result),
            }

        if dxl_error != 0:
            return {
                "connected": False,
                "motor_id": DXL_ID,
                "error": packet.getRxPacketError(dxl_error),
            }

        return {
            "connected": True,
            "device": DEVICE_NAME,
            "baudrate": BAUDRATE,
            "protocol_version": PROTOCOL_VERSION,
            "motor_id": DXL_ID,
            "model_number": model_number,
        }

    finally:
        port.closePort()


app = Litestar(
    route_handlers=[
        status,
        ping_motor,
    ],
)
