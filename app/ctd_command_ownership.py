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
            if self._owner not in ('manual', 'bench_cycle'):
                raise RuntimeError('Uncommissioned motor owner')

    def acquire_bench_cycle(self):
        with self._lock:
            if self._owner != 'manual':
                raise RuntimeError('Winch is already reserved')
            self._owner = 'bench_cycle'
            self._generation += 1

    def release_bench_cycle(self):
        with self._lock:
            if self._owner == 'bench_cycle':
                self._owner = 'manual'
                self._generation += 1

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
    if owner not in ('manual', 'autonomous', 'bench_cycle'):
        raise ValueError('Unknown motor command owner')
    token = _requester.set(owner)
    try:
        yield
    finally:
        _requester.reset(token)

def current_requester():
    return _requester.get()
