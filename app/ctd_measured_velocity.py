"""Measured Dynamixel velocity diagnostic; independent of commanded velocity.

Reads the Present Velocity register directly. Does not write motor commands.
"""
import math
import time
import config
from state import state

STOP_RPM_TOLERANCE = 0.5  # diagnostic threshold, NOT commissioned safety tolerance
MAX_SAMPLE_AGE_S = 2.0

def assess_raw(raw, *, sampled_at, now=None):
    now = time.monotonic() if now is None else now
    age = now - sampled_at
    fresh = 0 <= age <= MAX_SAMPLE_AGE_S
    value = int(raw) & 0xffffffff
    signed = value - 0x100000000 if value & 0x80000000 else value
    rpm = signed * config.PRESENT_VELOCITY_RPM_PER_UNIT
    valid = math.isfinite(rpm)
    return {
        'mode': 'measured_velocity_diagnostics_only',
        'measuredVelocityRpm': round(rpm, 3) if valid else None,
        'measuredVelocityFresh': fresh,
        'measuredVelocityAgeSeconds': round(age, 3),
        'measuredStoppedCandidate': bool(valid and fresh and abs(rpm) <= STOP_RPM_TOLERANCE),
        'stoppedThresholdRpm': STOP_RPM_TOLERANCE,
        'motorCommandedStopped': state.motion.velocity == 0,
        'homePhysicallyVerified': False,
        'missionContinuationAuthorized': False,
        'automaticWinchEnabled': False,
        'note': 'One measured velocity sample is not proof of sustained standstill or physical stowage.',
    }

def read():
    try:
        import motor_bus
        with motor_bus.session(state.bus_lock) as bus:
            bus.set_baudrate(config.WINCH_BAUDRATE)
            raw = bus.read_velocity(config.WINCH_ID, action='Read winch measured velocity')
            sampled_at = time.monotonic()
        return assess_raw(raw, sampled_at=sampled_at)
    except Exception as exc:
        return {'mode': 'measured_velocity_diagnostics_only', 'error': str(exc),
                'measuredVelocityFresh': False, 'measuredStoppedCandidate': False,
                'homePhysicallyVerified': False, 'missionContinuationAuthorized': False,
                'automaticWinchEnabled': False}
