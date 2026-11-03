"""Central winch write arbitration, fail-closed for future autonomy.

Every Dynamixel GOAL_VELOCITY write must pass this gate. Zero velocity
(STOP) is always permitted. Autonomous acquisition is intentionally disabled.
"""
from contextlib import contextmanager
from contextvars import ContextVar
from threading import RLock

_requester = ContextVar('ctd_motor_requester', default='manual')

class WinchOwnership:
    def __init__(self):
        self._lock = RLock()
        self._owner = 'manual'
        self._generation = 0

    def authorize(self, velocity):
        if velocity == 0:
            return
        with self._lock:
            if _requester.get() != self._owner:
                raise RuntimeError('Winch command rejected: motor ownership conflict')
            if self._owner != 'manual':
                raise RuntimeError('Autonomous winch commands are not commissioned')

    def snapshot(self):
        with self._lock:
            return {'owner': self._owner, 'generation': self._generation,
                    'centralVelocityWriteGateEnabled': True,
                    'autonomousOwnershipAvailable': False,
                    'stopAlwaysAllowed': True,
                    'automaticWinchEnabled': False}

    def acquire_autonomous(self):
        raise RuntimeError('Autonomous ownership is disabled pending commissioning')

    def reset_manual(self):
        # Intended for controlled startup only; not an emergency recovery API.
        with self._lock:
            self._owner = 'manual'
            self._generation += 1

ownership = WinchOwnership()

@contextmanager
def command_as(owner):
    if owner not in ('manual', 'autonomous'):
        raise ValueError('Unknown motor command owner')
    token = _requester.set(owner)
    try:
        yield
    finally:
        _requester.reset(token)
