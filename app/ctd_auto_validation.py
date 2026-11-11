"""Conservative automatic mission-readback scheduler.

Only requests mission metadata/items via existing MissionAudit. No MAVLink
mission modifications, motor commands, or navigation commands.
"""
import time
from threading import RLock


class AutoValidation:
    def __init__(self, retry_s=10.0, refresh_age_s=40.0):
        self.lock = RLock()
        self.retry_s = retry_s
        self.refresh_age_s = refresh_age_s
        self.last_attempt = None
        self.last_result = 'not_started'

    def tick(self, *, audit, coordinator, mission, preview_phase=None, physical_cycle_phase=None, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            state = audit.snapshot()
            phase = state.get('status')
            if phase in ('request_list', 'waiting_count', 'downloading'):
                self.last_result = 'download_in_progress'
                return False
            if physical_cycle_phase in ('armed_waiting_for_simulated_arrival', 'deploying',
                                        'sampling', 'retracting', 'verifying_home',
                                        'home_confirmed_pending_stop'):
                self.last_result = 'paused_during_physical_cycle'
                return False
            if preview_phase in ('descending', 'sampling', 'retracting', 'ready_to_continue'):
                self.last_result = 'paused_during_ctd_cycle'
                return False
            if not mission.get('telemetryFresh') or not mission.get('sourceSystem') or not mission.get('sourceComponent'):
                self.last_result = 'waiting_for_autopilot'
                return False
            stations = coordinator.stations_snapshot()
            if not stations:
                self.last_result = 'waiting_for_configuration'
                return False
            age = state.get('snapshotAgeSeconds')
            if state.get('missionValidated') and age is not None and age < self.refresh_age_s:
                self.last_result = 'validated'
                return False
            if self.last_attempt is not None and now - self.last_attempt < self.retry_s:
                self.last_result = 'retry_cooldown'
                return False
            self.last_attempt = now
            ok, error = audit.start(stations, mission['sourceSystem'], mission['sourceComponent'])
            self.last_result = 'requested' if ok else ('request_failed: ' + str(error))
            return ok

    def snapshot(self):
        with self.lock:
            return {'enabled': True, 'lastResult': self.last_result,
                    'lastAttemptMonotonic': self.last_attempt,
                    'requestsReadOnlyMissionData': True,
                    'executionEnabled': False}


auto_validation = AutoValidation()
