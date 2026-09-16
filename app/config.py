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
# With the 2:1 reduction, 6 motor degrees = 3 ratchet degrees.
WINCH_COUNTS_PER_REV = 4096
UNLOCK_RELIEF_MOTOR_DEG = 6.0
UNLOCK_RELIEF_COUNTS = round(
    WINCH_COUNTS_PER_REV * UNLOCK_RELIEF_MOTOR_DEG / 360.0
)

UNLOCK_RELIEF_VELOCITY = 20
UNLOCK_RELIEF_TIMEOUT = 2.0
UNLOCK_RELIEF_POLL_INTERVAL = 0.02
UNLOCK_RELIEF_SETTLE_DELAY = 0.2


# ============================================================
# MAVLINK SETTINGS
# ============================================================

MAVLINK_PORT = 14560
SERVO_NUMBER = 10

PWM_RETRACT = 1100
PWM_STOP = 1300
PWM_IDLE = 1500
PWM_DEPLOY = 1900
