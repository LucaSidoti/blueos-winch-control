"""Hardware, motor, timing, and MAVLink configuration."""

# ============================================================
# USB / PROTOCOL SETTINGS
# ============================================================

DEVICE_NAME = "/dev/ttyUSB0"
PROTOCOL_VERSION = 2.0


# ============================================================
# WINCH MOTOR - XW540-T140-R
# ============================================================

WINCH_ID = 7
WINCH_BAUDRATE = 1_000_000

ADDR_OPERATING_MODE = 11
ADDR_DRIVE_MODE = 10
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104
ADDR_PROFILE_ACCELERATION = 108

ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_INPUT_VOLTAGE = 144
ADDR_PRESENT_TEMPERATURE = 146

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
VELOCITY_MODE = 1

WINCH_PROFILE_ACCELERATION = 1
DYNAMIXEL_RPM_PER_UNIT = 0.229
SPEED_LEVELS = [20, 40, 60, 80, 100]

PRESENT_CURRENT_MA_PER_UNIT = 2.69
PRESENT_VELOCITY_RPM_PER_UNIT = 0.229
PRESENT_VOLTAGE_V_PER_UNIT = 0.1
POSITION_DEG_PER_COUNT = 360.0 / 4096.0

# Software retract limit, captured at each successful initialization.
# These are engineering defaults: validate stopping distance on the real winch.
RETRACT_LIMIT_POLL_INTERVAL = 0.05
RETRACT_LIMIT_REACTION_TIME = 0.25
RETRACT_LIMIT_BRAKING_FACTOR = 1.5
RETRACT_LIMIT_MARGIN_DEG = 1.0
DYNAMIXEL_ACCEL_RPM_PER_MIN_PER_UNIT = 214.577


# ============================================================
# LOCK MOTOR - XW430-T200-R
# ============================================================

LOCK_ID = 0
LOCK_BAUDRATE = 115_200

LOCK_ADDR_OPERATING_MODE = 11
LOCK_ADDR_TORQUE_ENABLE = 64
LOCK_ADDR_PROFILE_ACCELERATION = 108
LOCK_ADDR_PROFILE_VELOCITY = 112
LOCK_ADDR_GOAL_POSITION = 116

POSITION_MODE = 3

UNLOCK_POSITION_DEG = 205.0
UNLOCK_POSITION_RAW = round(
    UNLOCK_POSITION_DEG / 360.0 * 4096
)

LOCK_PROFILE_ACCELERATION = 5
LOCK_PROFILE_VELOCITY = 20

# Minimum time to let the pawl begin lifting before checking position.
LOCK_COMMAND_DELAY = 0.6

# Unlock confirmation settings.
UNLOCK_POSITION_TOLERANCE_DEG = 12.0
UNLOCK_VERIFY_TIMEOUT = 1.5
UNLOCK_VERIFY_POLL_INTERVAL = 0.05


# ============================================================
# RATCHET / PAWL LOAD RELIEF
# ============================================================

# The ratchet has 24 teeth -> 15 degrees per ratchet tooth.
# 2 motor revolutions = 1 spool revolution.
WINCH_COUNTS_PER_REV = 4096
WINCH_GEAR_RATIO = 2.0

UNLOCK_RELIEF_MOTOR_DEG = 6.0
UNLOCK_RELIEF_COUNTS = round(
    WINCH_COUNTS_PER_REV * UNLOCK_RELIEF_MOTOR_DEG / 360.0
)

UNLOCK_RELIEF_VELOCITY = 20
UNLOCK_RELIEF_TIMEOUT = 2.0
UNLOCK_RELIEF_POLL_INTERVAL = 0.02
UNLOCK_RELIEF_SETTLE_DELAY = 0.2

# Give the spring-loaded pawl time to engage before any XW540
# reboot or torque loss.
LOCK_ENGAGE_SETTLE_DELAY = 0.35


# ============================================================
# HOME-LOSS RECOVERY
# ============================================================

# HOME-loss recovery is deliberately slow and jog-only.
# Each jog is bounded by encoder travel; there is no automatic
# retract limit because HOME is unknown.
RECOVERY_JOG_VELOCITY = 20
RECOVERY_JOG_MOTOR_DEG = 45.0
RECOVERY_JOG_COUNTS = round(
    WINCH_COUNTS_PER_REV * RECOVERY_JOG_MOTOR_DEG / 360.0
)
RECOVERY_JOG_TIMEOUT = 3.0
RECOVERY_JOG_POLL_INTERVAL = 0.02


# ============================================================
# MAVLINK SETTINGS
# ============================================================

MAVLINK_PORT = 14560
SERVO_NUMBER = 10

PWM_RETRACT = 1100
PWM_STOP = 1300
PWM_IDLE = 1500
PWM_DEPLOY = 1900


# ============================================================
# DEPTH CONTROL / SPOOL MODEL
# ============================================================

# Measured/estimated geometry.
SPOOL_FULL_DIAMETER_M = 0.15
SPOOL_CORE_DIAMETER_M = 0.040
SPOOL_USABLE_WIDTH_M = 0.070
CABLE_DIAMETER_M = 0.003
CABLE_NOMINAL_LENGTH_M = 200.0


# ============================================================
# OPERATIONAL HOME ADJUSTMENT
# ============================================================

# INITIALIZE captures the hard retract safety reference.
# ADJUST HOME changes only the operational storage position.
#
# HOME adjustment uses small ~1 cm jogs.
HOME_ADJUST_JOG_M = 0.01
HOME_ADJUST_JOG_VELOCITY = 20

HOME_ADJUST_JOG_COUNTS = max(
    1,
    round(
        HOME_ADJUST_JOG_M
        * WINCH_COUNTS_PER_REV
        * WINCH_GEAR_RATIO
        / (3.141592653589793 * SPOOL_FULL_DIAMETER_M)
    ),
)

HOME_ADJUST_JOG_TIMEOUT = 2.0
HOME_ADJUST_JOG_POLL_INTERVAL = 0.02

# CANCEL returns to the physical position where ADJUST HOME was entered before
# restoring normal safety monitoring.
HOME_ADJUST_CANCEL_TIMEOUT = 20.0
HOME_ADJUST_CANCEL_TOLERANCE_COUNTS = 8


# ============================================================
# DEPTH CONTROL
# ============================================================

# Sensor height above the water when the reference is captured.
SENSOR_HEIGHT_ABOVE_WATER_M = 0.0

MAX_TARGET_DEPTH_M = 200.0
DEPTH_TOLERANCE_M = 0.03
DEPTH_CONTROL_POLL_INTERVAL = 0.10

# Automatic speed schedule:
# (distance-to-target threshold in metres, velocity)
# First matching threshold is used, from near to far.
DEPTH_SPEED_PROFILE = [
    (0.25, 20),
    (0.75, 40),
    (2.00, 60),
    (5.00, 80),
    (float("inf"), 100),
]

# Final retract/storage approach.
# These limits override the normal target speed.
STORAGE_SLOWDOWN_PAYOUT_M = 0.50
STORAGE_CREEP_PAYOUT_M = 0.30

STORAGE_SLOW_VELOCITY = 40
STORAGE_CREEP_VELOCITY = 20

# Dedicated return-to-storage profile.
HOME_NORMAL_VELOCITY = 60

# Dedicated precision approach for RETURN TO STORAGE.
# Speed level 1 (20) has a conservative braking envelope of about 2 cm with the
# current model, so use a much lower velocity for the final 10 cm. The normal
# retract safety remains fully active and will still stop before encoder zero.
HOME_FINAL_APPROACH_M = 0.10
HOME_FINAL_VELOCITY = 5
HOME_TOLERANCE_M = 0.0


# ============================================================
# ENCODER SAFETY
# ============================================================

# Position tracker sanity limit. The safety monitor samples during
# all motion, so a larger jump indicates lost/invalid encoder tracking.
ENCODER_MAX_DELTA_COUNTS = 600