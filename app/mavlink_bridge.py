"""Translate MAVLink servo signals into shared motor control commands."""

from pymavlink import mavutil

import config
import control


# ============================================================
# MAVLINK HELPERS
# ============================================================

def reset_servo_signal(
    connection,
    target_system,
    target_component,
):
    """Reset SERVO10 to 1500 so the next D-pad press is detectable."""

    connection.mav.command_long_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        config.SERVO_NUMBER,
        config.PWM_IDLE,
        0,
        0,
        0,
        0,
        0,
    )


# ============================================================
# MAVLINK LISTENER
# ============================================================

def mavlink_listener():
    """
    Xbox/QGroundControl command bridge via SERVO10 telemetry.

      1100 -> Retract / faster
      1300 -> Stop
      1500 -> Idle
      1900 -> Deploy / faster
    """

    print(
        f"MAVLink winch listener starting on UDP port {config.MAVLINK_PORT}",
        flush=True,
    )

    try:
        connection = mavutil.mavlink_connection(
            f"udpin:0.0.0.0:{config.MAVLINK_PORT}"
        )
    except Exception as exc:
        print(
            f"Could not start MAVLink listener: {exc}",
            flush=True,
        )
        return

    command_ready = False

    while True:
        try:
            message = connection.recv_match(
                type="SERVO_OUTPUT_RAW",
                blocking=True,
                timeout=1,
            )

            if message is None:
                continue

            target_system = message.get_srcSystem()
            target_component = message.get_srcComponent()
            pwm = message.servo10_raw

            if not command_ready:
                if pwm == config.PWM_IDLE:
                    command_ready = True
                    print(
                        "MAVLink winch control ready",
                        flush=True,
                    )
                else:
                    reset_servo_signal(
                        connection,
                        target_system,
                        target_component,
                    )
                continue

            if pwm == config.PWM_IDLE:
                continue

            command_ready = False

            if pwm == config.PWM_RETRACT:
                print(
                    "Xbox command: RETRACT",
                    flush=True,
                )
                try:
                    result = control.execute_retract()
                    print(
                        f"Winch state: {result['status']}, "
                        f"speed level {result['speed_level']}",
                        flush=True,
                    )
                except Exception as exc:
                    print(
                        f"Retract command rejected: {exc}",
                        flush=True,
                    )

            elif pwm == config.PWM_STOP:
                print(
                    "Xbox command: STOP",
                    flush=True,
                )
                try:
                    control.execute_stop()
                    print(
                        "Winch stopped",
                        flush=True,
                    )
                except Exception as exc:
                    print(
                        f"Stop command failed: {exc}",
                        flush=True,
                    )

            elif pwm == config.PWM_DEPLOY:
                print(
                    "Xbox command: DEPLOY",
                    flush=True,
                )
                try:
                    result = control.execute_deploy()
                    print(
                        f"Winch state: {result['status']}, "
                        f"speed level {result['speed_level']}",
                        flush=True,
                    )
                except Exception as exc:
                    print(
                        f"Deploy command rejected: {exc}",
                        flush=True,
                    )

            else:
                print(
                    f"SERVO10 value ignored: {pwm}",
                    flush=True,
                )

            reset_servo_signal(
                connection,
                target_system,
                target_component,
            )

        except Exception as exc:
            print(
                f"MAVLink listener error: {exc}",
                flush=True,
            )
