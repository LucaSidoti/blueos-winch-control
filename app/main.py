#!/usr/bin/env python3

from pathlib import Path
from threading import Thread
from litestar import Litestar, get, post
from litestar.static_files.config import StaticFilesConfig
import config
import control
from mavlink_bridge import mavlink_listener

# ============================================================
# GENERAL STATUS
# ============================================================

@get("/status", sync_to_thread=False)
def status() -> dict:
    return {
        "extension": "winch-control",
        "device": config.DEVICE_NAME,
        "device_exists": Path(config.DEVICE_NAME).exists(),
        "protocol_version": config.PROTOCOL_VERSION,
        "winch_motor_id": config.WINCH_ID,
        "winch_baudrate": config.WINCH_BAUDRATE,
        "lock_motor_id": config.LOCK_ID,
        "lock_baudrate": config.LOCK_BAUDRATE,
        "mavlink_port": config.MAVLINK_PORT,
    }


# ============================================================
# SYSTEM PING
# ============================================================

@get("/motor/ping", sync_to_thread=True)
def ping_motor() -> dict:
    return control.ping_motor()

# ============================================================
# SYSTEM INITIALIZATION
# ============================================================

@post("/motor/initialize", sync_to_thread=True)
def initialize_motor() -> dict:
    return control.initialize_motor()

# ============================================================
# TORQUE ENABLE
# ============================================================

@post("/motor/torque/enable", sync_to_thread=True)
def enable_torque() -> dict:
    return control.enable_torque()

# ============================================================
# TORQUE DISABLE
# ============================================================

@post("/motor/torque/disable", sync_to_thread=True)
def disable_torque() -> dict:
    return control.disable_torque()

# ============================================================
# LOCK TOGGLE
# ============================================================

@post("/lock/toggle", sync_to_thread=True)
def toggle_lock() -> dict:
    return control.toggle_lock()

# ============================================================
# MOTOR STATE / MOTION HTTP ENDPOINTS
# ============================================================

@get("/motor/telemetry", sync_to_thread=True)
def motor_telemetry() -> dict:
    try:
        return control.read_all_telemetry()
    except Exception as exc:
        return {
            "success": False,
            "error": str(exc),
        }

@get("/motor/state", sync_to_thread=False)
def motor_state() -> dict:
    return control.get_motor_state()

@post("/motor/stop", sync_to_thread=True)
def stop_motor() -> dict:
    try:
        return control.execute_stop()
    except Exception as exc:
        return {
            "success": False,
            "error": str(exc),
        }

@post("/motor/retract", sync_to_thread=True)
def command_retract() -> dict:
    try:
        return control.execute_retract()
    except Exception as exc:
        return {
            "success": False,
            "error": str(exc),
        }

@post("/motor/deploy", sync_to_thread=True)
def command_deploy() -> dict:
    try:
        return control.execute_deploy()
    except Exception as exc:
        return {
            "success": False,
            "error": str(exc),
        }

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
