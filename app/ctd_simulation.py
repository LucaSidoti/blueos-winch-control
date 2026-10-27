"""Hardware-isolated CTD mission rehearsal. No imports of motor or MAVLink drivers."""
from threading import RLock


class CTDSimulator:
    PHASES = ('idle', 'arriving', 'descending', 'sampling', 'retracting', 'complete', 'aborted')

    def __init__(self):
        self.lock = RLock()
        self.reset()

    def reset(self):
        with self.lock:
            self.stations = []
            self.index = None
            self.phase = 'idle'
            self.error = None

    def start(self, stations):
        with self.lock:
            if self.phase not in ('idle', 'complete', 'aborted'):
                raise ValueError('Simulation already running; abort or reset first')
            if not stations:
                raise ValueError('No CTD stations configured')
            self.stations = [s.to_dict() for s in stations]
            self.index = 0
            self.phase = 'arriving'
            self.error = None
            return self.snapshot()

    def advance(self):
        """Explicitly advance the simulated state; no real telemetry or actuator commands."""
        with self.lock:
            transitions = {
                'arriving': 'descending',
                'descending': 'sampling',
                'sampling': 'retracting',
            }
            if self.phase == 'retracting':
                if self.index + 1 < len(self.stations):
                    self.index += 1
                    self.phase = 'arriving'
                else:
                    self.phase = 'complete'
            elif self.phase in transitions:
                self.phase = transitions[self.phase]
            else:
                raise ValueError(f'Cannot advance simulation in phase {self.phase}')
            return self.snapshot()

    def abort(self):
        with self.lock:
            self.phase = 'aborted'
            self.error = 'Operator aborted simulation'
            return self.snapshot()

    def snapshot(self):
        with self.lock:
            return {
                'mode': 'simulation_only',
                'executionEnabled': False,
                'hardwareCommandsSent': False,
                'phase': self.phase,
                'stationIndex': self.index,
                'stationCount': len(self.stations),
                'station': self.stations[self.index] if self.index is not None and self.index < len(self.stations) else None,
                'error': self.error,
            }


ctd_simulator = CTDSimulator()
