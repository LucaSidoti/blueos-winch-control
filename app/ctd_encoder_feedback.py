"""Read-only encoder feedback assessment for calibrated CTD HOME.

Never sends motor commands. A passing assessment is NOT independent proof
that the instrument is physically stowed, and never authorizes navigation.
"""
import time
import math
import config
from state import state


def assess(status, *, sampled_at=None, now=None, max_age_s=2.0):
    now = time.monotonic() if now is None else now
    age = None if sampled_at is None else now - sampled_at
    fresh = age is not None and 0 <= age <= max_age_s
    position = status.get('continuous_deployed_counts')
    reference = state.operational_home_counts
    payout = status.get('cable_payout_m')
    valid_position = isinstance(position, (int, float)) and math.isfinite(position)
    valid_payout = isinstance(payout, (int, float)) and math.isfinite(payout)
    remaining = abs(payout) if valid_payout else None
    fault = status.get('safety_fault') or status.get('last_error')
    calibrated = bool(state.initialized and status.get('home_set'))
    stopped = (not status.get('active') and
               status.get('command_velocity') == 0 and
               state.motion.velocity == 0)
    # Both count and payout checks are derived from the SAME encoder; they
    # are not independent physical evidence.
    within_tolerance = bool(valid_payout and remaining <= config.HOME_TOLERANCE_M)
    candidate = bool(status.get('success') and fresh and calibrated and
                     valid_position and valid_payout and
                     status.get('feedback_trustworthy') and not fault and
                     stopped and within_tolerance and
                     not state.home_adjust_mode and not state.home_recovery_mode)
    return {
        'mode': 'encoder_feedback_diagnostics_only',
        'feedbackFresh': fresh,
        'feedbackAgeSeconds': round(age, 3) if age is not None else None,
        'homeReferenceCalibrated': calibrated,
        'homePositionCounts': reference,
        'currentEncoderCounts': position,
        'cablePayoutMeters': payout,
        'homeToleranceMeters': config.HOME_TOLERANCE_M,
        'withinEncoderHomeTolerance': within_tolerance,
        'motorCommandedStopped': stopped,
        'feedbackTrustworthy': bool(status.get('feedback_trustworthy')),
        'fault': fault,
        'encoderHomeCandidate': candidate,
        'homePhysicallyVerified': False,
        'missionContinuationAuthorized': False,
        'automaticWinchEnabled': False,
        'automaticAdvanceEnabled': False,
        'note': 'Encoder-based assessment only; no independent physical stowage detection or mission authorization.',
    }
