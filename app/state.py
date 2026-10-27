"""One shared state object and lock for HTTP, MAVLink, and safety monitoring."""

from dataclasses import dataclass, field
from functools import wraps
from threading import RLock
import time

@dataclass
class MotionState:
    direction: int = 0  # -1 retract, 0 stopped, +1 deploy
    speed_level: int = 0
    velocity: int = 0

@dataclass
class UnlockDiagnostics:
    position_deg: float | None = None
    current_a: float | None = None
    position_error_deg: float | None = None
    success: bool | None = None

@dataclass
class RetractLimitState:
    reference: int | None = None  # Signed multi-turn encoder counts.
    last_position: int | None = None
    reached: bool = False
    fault: str | None = None
    stopping: bool = False
    raw_position: int | None = None  # Last raw single-turn encoder sample.
    deployed_counts: int = 0         # Continuous payout coordinate; 0 = stored.

@dataclass
class DepthState:
    active: bool = False
    mode: str = "idle"  # idle, depth, or home
    target_m: float | None = None
    phase: str = "idle"
    command_velocity: int = 0
    last_error: str | None = None

@dataclass
class MissionMonitorState:
    current_sequence: int | None = None
    source_system: int | None = None
    source_component: int | None = None
    last_received_monotonic: float | None = None
    lock: object = field(default_factory=RLock, repr=False)

    def update(self, sequence: int, system: int, component: int) -> None:
        with self.lock:
            self.current_sequence = sequence
            self.source_system = system
            self.source_component = component
            self.last_received_monotonic = time.monotonic()

    def snapshot(self) -> dict:
        with self.lock:
            age = (time.monotonic() - self.last_received_monotonic
                   if self.last_received_monotonic is not None else None)
            return {
                "executionEnabled": False,
                "currentSequence": self.current_sequence,
                "sourceSystem": self.source_system,
                "sourceComponent": self.source_component,
                "telemetryAgeSeconds": round(age, 2) if age is not None else None,
                "telemetryFresh": age is not None and age <= 5.0,
            }


mission_monitor = MissionMonitorState()

@dataclass
class ApplicationState:
    initialized: bool = False
    torque_enabled: bool = False
    lock_state: str = "locked"
    motion: MotionState = field(default_factory=MotionState)
    unlock: UnlockDiagnostics = field(default_factory=UnlockDiagnostics)
    retract_limit: RetractLimitState = field(default_factory=RetractLimitState)
    depth: DepthState = field(default_factory=DepthState)
    home_recovery_mode: bool = False
    home_recovery_required: bool = False
    home_adjust_mode: bool = False
    # Snapshot of the continuous payout coordinate when ADJUST HOME starts.
    # CANCEL uses this to physically return to the pre-adjustment position.
    home_adjust_start_counts: int | None = None
    operational_home_counts: int = 0  # Calibrated HOME; SET HOME HERE also rebases the hard retract reference.
    bus_lock: object = field(default_factory=RLock, repr=False)

state = ApplicationState()

def serialized_control(function):
    """Keep an entire command's state changes atomic with bus/monitor access."""
    @wraps(function)
    def wrapped(*args, **kwargs):
        with state.bus_lock:
            return function(*args, **kwargs)
    return wrapped
