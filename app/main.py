#!/usr/bin/env python3

from pathlib import Path

from dynamixel_sdk import PortHandler
from litestar import Litestar, get


DEVICE_NAME = "/dev/ttyUSB0"


@get("/status", sync_to_thread=False)
def status() -> dict:
    device_exists = Path(DEVICE_NAME).exists()

    return {
        "extension": "winch-control",
        "device": DEVICE_NAME,
        "device_exists": device_exists,
    }


app = Litestar(
    route_handlers=[status],
)
