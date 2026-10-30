#!/usr/bin/env python3

import logging
from pathlib import Path
from threading import Thread
from litestar import Litestar, get, post
from litestar.static_files.config import StaticFilesConfig
import config
import control
import depth_control
from ctd_mission import CTDMissionCoordinator
from mavlink_bridge import mavlink_listener
from mission_audit import mission_audit
from ctd_simulation import ctd_simulator
from state import mission_monitor

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
# SAFETY FAULT RESET
# ============================================================

@post("/motor/safety/reset", sync_to_thread=True)
def reset_safety_fault() -> dict:
    return control.reset_safety_fault()

@post("/motor/hardware/recover", sync_to_thread=True)
def recover_winch_hardware() -> dict:
    try:
        return control.recover_winch_hardware()
    except Exception as exc:
        return {"success": False, "error": str(exc)}

@post("/motor/home-recovery/start", sync_to_thread=True)
def start_home_recovery() -> dict:
    try:
        return control.start_home_recovery()
    except Exception as exc:
        return {"success": False, "error": str(exc)}

@post("/motor/home-recovery/jog", sync_to_thread=True)
def home_recovery_jog() -> dict:
    # Backward-compatible alias: the original recovery jog retracts.
    try:
        return control.home_recovery_retract_jog()
    except Exception as exc:
        return {"success": False, "error": str(exc)}

@post("/motor/home-recovery/retract", sync_to_thread=True)
def home_recovery_retract() -> dict:
    try:
        return control.home_recovery_retract_jog()
    except Exception as exc:
        return {"success": False, "error": str(exc)}

@post("/motor/home-recovery/deploy", sync_to_thread=True)
def home_recovery_deploy() -> dict:
    try:
        return control.home_recovery_deploy_jog()
    except Exception as exc:
        return {"success": False, "error": str(exc)}

@post("/motor/home-recovery/finish", sync_to_thread=True)
def finish_home_recovery() -> dict:
    try:
        return control.finish_home_recovery()
    except Exception as exc:
        return {"success": False, "error": str(exc)}

# ============================================================
# OPERATIONAL HOME ADJUSTMENT
# ============================================================

@post("/motor/home-adjust/start", sync_to_thread=True)
def start_home_adjustment() -> dict:
    try:
        return control.start_home_adjustment()
    except Exception as exc:
        return {"success": False, "error": str(exc)}

@post("/motor/home-adjust/retract", sync_to_thread=True)
def home_adjust_retract() -> dict:
    try:
        return control.home_adjust_retract_jog()
    except Exception as exc:
        return {"success": False, "error": str(exc)}

@post("/motor/home-adjust/deploy", sync_to_thread=True)
def home_adjust_deploy() -> dict:
    try:
        return control.home_adjust_deploy_jog()
    except Exception as exc:
        return {"success": False, "error": str(exc)}

@post("/motor/home-adjust/set", sync_to_thread=True)
def set_operational_home() -> dict:
    try:
        return control.set_operational_home_here()
    except Exception as exc:
        return {"success": False, "error": str(exc)}

@post("/motor/home-adjust/cancel", sync_to_thread=True)
def cancel_home_adjustment() -> dict:
    try:
        return control.cancel_home_adjustment()
    except Exception as exc:
        return {"success": False, "error": str(exc)}

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
        depth_control.cancel(stop=False)
        return control.execute_stop()
    except Exception as exc:
        return {
            "success": False,
            "error": str(exc),
        }

@post("/motor/retract", sync_to_thread=True)
def command_retract() -> dict:
    try:
        depth_control.cancel(stop=False)
        return control.execute_retract()
    except Exception as exc:
        return {
            "success": False,
            "error": str(exc),
        }

@post("/motor/deploy", sync_to_thread=True)
def command_deploy() -> dict:
    try:
        depth_control.cancel(stop=False)
        return control.execute_deploy()
    except Exception as exc:
        return {
            "success": False,
            "error": str(exc),
        }


# ============================================================
# AUTOMATIC DEPTH CONTROL
# ============================================================

@get("/depth", sync_to_thread=True)
def depth_status() -> dict:
    return depth_control.get_status(read_position=True)

@post("/depth/target", sync_to_thread=True)
def set_depth_target(data: dict) -> dict:
    try:
        if "depth_m" not in data:
            raise ValueError("JSON body must contain depth_m")
        return depth_control.set_target(data["depth_m"])
    except Exception as exc:
        return {"success": False, "error": str(exc)}


@post("/depth/home", sync_to_thread=True)
def return_home() -> dict:
    try:
        return depth_control.go_home()
    except Exception as exc:
        return {"success": False, "error": str(exc)}

@post("/depth/cancel", sync_to_thread=True)
def cancel_depth_target() -> dict:
    try:
        return depth_control.cancel(stop=True)
    except Exception as exc:
        return {"success": False, "error": str(exc)}

# ============================================================
# CTD MISSION CONFIGURATION (READ-ONLY PROTOTYPE)
# ============================================================

CTD_CONFIG_PATH = Path("/app/ctd_stations.json")
ctd_coordinator = CTDMissionCoordinator()

def _ctd_monitor_snapshot() -> dict:
    """Shared read-only snapshot; callable independently of Litestar routes."""
    telemetry = mission_monitor.snapshot()
    telemetry["stationMatch"] = ctd_coordinator.match_sequence(
        telemetry["currentSequence"], telemetry["telemetryFresh"]
    )
    telemetry["stationReached"] = False
    telemetry["missionValidated"] = mission_audit.snapshot()["missionValidated"]
    return telemetry


@get("/ctd/mission/monitor", sync_to_thread=False)
def ctd_mission_monitor() -> dict:
    """Read-only MAVLink mission progress; never triggers CTD execution."""
    return _ctd_monitor_snapshot()

@get("/ctd/mission/coordination", sync_to_thread=False)
def ctd_mission_coordination() -> dict:
    """Read-only mission readiness. Never authorizes autonomous execution."""
    try:
        telemetry = _ctd_monitor_snapshot()
        audit = mission_audit.snapshot()
        config_status = ctd_coordinator.status()

        match = telemetry.get("stationMatch")
        configured = bool(config_status.get("stationCount", 0))
        validated = bool(audit.get("missionValidated", False))
        fresh = bool(telemetry.get("telemetryFresh", False))
        ready = configured and validated and fresh
        at_loiter = isinstance(match, dict) and bool(match.get("atLoiterSequence"))

        return {
            "mode": "supervised_observation",
            "executionEnabled": False,
            "automaticWinchEnabled": False,
            "automaticAdvanceEnabled": False,
            "configurationLoaded": configured,
            "missionValidated": validated,
            "telemetryFresh": fresh,
            "readyForObservation": ready,
            "readyForAutonomousExecution": False,
            "currentSequence": telemetry.get("currentSequence"),
            "stationMatch": match,
            "phase": ("loiter_observed" if at_loiter
                      else "approaching_station" if match else "unmatched_or_stale"),
            "note": "Sequence observation alone does not prove arrival or safe station keeping.",
        }
    except Exception:
        # Log the complete traceback inside the container, but do not expose
        # internal implementation details to unauthenticated HTTP clients.
        logging.exception("CTD coordination endpoint failed")
        return {
            "mode": "supervised_observation",
            "executionEnabled": False,
            "automaticWinchEnabled": False,
            "automaticAdvanceEnabled": False,
            "configurationLoaded": False,
            "missionValidated": False,
            "telemetryFresh": False,
            "readyForObservation": False,
            "readyForAutonomousExecution": False,
            "currentSequence": None,
            "stationMatch": None,
            "phase": "error",
            "note": "Coordination status unavailable; inspect BlueOS extension logs.",
        }

@get("/ctd/mission/validation", sync_to_thread=False)
def ctd_mission_validation() -> dict:
    return mission_audit.snapshot()

@post("/ctd/mission/validation/request", sync_to_thread=False)
def request_ctd_mission_validation() -> dict:
    """Request a read-only MAVLink mission download; never changes vehicle state."""
    source = mission_monitor.snapshot()
    if not source["telemetryFresh"]:
        return {"success": False, "error": "Autopilot telemetry is stale"}
    success, error = mission_audit.start(
        ctd_coordinator.stations_snapshot(),
        source["sourceSystem"], source["sourceComponent"],
    )
    return {"success": success, "error": error, **mission_audit.snapshot()}

@get("/ctd/simulation", sync_to_thread=False)
def ctd_simulation_status() -> dict:
    return ctd_simulator.snapshot()

@post("/ctd/simulation/start", sync_to_thread=False)
def ctd_simulation_start() -> dict:
    try:
        return {"success": True, **ctd_simulator.start(ctd_coordinator.stations_snapshot())}
    except ValueError as exc:
        return {"success": False, "error": str(exc), **ctd_simulator.snapshot()}

@post("/ctd/simulation/advance", sync_to_thread=False)
def ctd_simulation_advance() -> dict:
    try:
        return {"success": True, **ctd_simulator.advance()}
    except ValueError as exc:
        return {"success": False, "error": str(exc), **ctd_simulator.snapshot()}

@post("/ctd/simulation/abort", sync_to_thread=False)
def ctd_simulation_abort() -> dict:
    return {"success": True, **ctd_simulator.abort()}

@get("/ctd/mission", sync_to_thread=False)
def ctd_mission_status() -> dict:
    return ctd_coordinator.status()

@post("/ctd/mission/load", sync_to_thread=True)
def load_ctd_mission() -> dict:
    try:
        result = ctd_coordinator.load(CTD_CONFIG_PATH)
        mission_audit.invalidate()
        ctd_simulator.reset()
        return {"success": True, **result}
    except (OSError, ValueError, TypeError) as exc:
        return {
            "success": False,
            "error": str(exc),
            **ctd_coordinator.status(),
        }

@post("/ctd/mission/config", sync_to_thread=True)
def upload_ctd_mission_config(data: dict) -> dict:
    """Validate an uploaded station list; never actuate motors or MAVLink."""
    try:
        result = ctd_coordinator.load_data(data)
        mission_audit.invalidate()
        ctd_simulator.reset()
        return {"success": True, **result}
    except (ValueError, TypeError) as exc:
        return {
            "success": False,
            "error": str(exc),
            **ctd_coordinator.status(),
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
        reset_safety_fault,
        recover_winch_hardware,
        start_home_recovery,
        home_recovery_jog,
        home_recovery_retract,
        home_recovery_deploy,
        finish_home_recovery,
        start_home_adjustment,
        home_adjust_retract,
        home_adjust_deploy,
        set_operational_home,
        cancel_home_adjustment,
        disable_torque,
        toggle_lock,
        motor_state,
        motor_telemetry,
        stop_motor,
        command_retract,
        command_deploy,
        depth_status,
        set_depth_target,
        return_home,
        cancel_depth_target,
        ctd_mission_status,
        ctd_simulation_status,
        ctd_simulation_start,
        ctd_simulation_advance,
        ctd_simulation_abort,
        ctd_mission_monitor,
        ctd_mission_coordination,
        ctd_mission_validation,
        request_ctd_mission_validation,
        load_ctd_mission,
        upload_ctd_mission_config,
    ],
    static_files_config=[
        StaticFilesConfig(
            directories=["/app/static"],
            path="/",
            html_mode=True,
        )
    ],
)
