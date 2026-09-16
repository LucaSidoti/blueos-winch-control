"""One shared state object and lock for HTTP, MAVLink, and safety monitoring."""

from dataclasses import dataclass, field
from functools import wraps
from threading import RLock

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

@dataclass
class ApplicationState:
    initialized: bool = False
    torque_enabled: bool = False
    lock_state: str = "locked"
    motion: MotionState = field(default_factory=MotionState)
    unlock: UnlockDiagnostics = field(default_factory=UnlockDiagnostics)
    retract_limit: RetractLimitState = field(default_factory=RetractLimitState)
    bus_lock: object = field(default_factory=RLock, repr=False)

state = ApplicationState()

def serialized_control(function):
    """Keep an entire command's state changes atomic with bus/monitor access."""
    @wraps(function)
    def wrapped(*args, **kwargs):
        with state.bus_lock:
            return function(*args, **kwargs)
    return wrapped
