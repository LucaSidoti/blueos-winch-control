"""Cached, non-actuating CTD hardware-readiness diagnostics.

Never calls get_status(read_position=True): that can initiate a motor-bus read.
Cached encoder/depth state is *not* proof of physical HOME.
"""
from state import state
from ctd_command_ownership import ownership


def snapshot(*, mission, audit, gps, depth_status):
    motor_fault = depth_status.get('safety_fault') or depth_status.get('last_error')
    return {
        'mode': 'diagnostics_only',
        'configurationValidated': bool(audit.get('missionValidated')),
        'missionTelemetryFresh': bool(mission.get('telemetryFresh')),
        'gpsFixValid': bool(gps.get('gpsFixValid')),
        'winchInitialized': bool(state.initialized),
        'winchHomeReferenceSet': bool(depth_status.get('home_set')),
        'cachedFeedbackTrustworthy': bool(depth_status.get('feedback_trustworthy')),
        'cachedMotorFault': motor_fault,
        'depthControlActive': bool(depth_status.get('active')),
        'cachedDepthMeters': depth_status.get('depth_m'),
        'homePhysicallyVerified': False,
        'motorCommandOwnershipEnforced': ownership.snapshot()['centralVelocityWriteGateEnabled'],
        'autonomousOwnershipAvailable': False,
        'autonomousModeVerified': False,
        'missionContinuationHandshakeVerified': False,
        'readyForAutonomousExecution': False,
        'automaticWinchEnabled': False,
        'automaticAdvanceEnabled': False,
        'note': 'Cached diagnostics only. No physical HOME or actuation authorization.'
    }
