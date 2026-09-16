# QuickStart Python Extension

A quick-start repository for building and uploading a Python-focused BlueOS Extension.

## Intent

This is intended to showcase:
1. How to make a basic Extension with a simple web interface, using Python and some HTML
2. The difference between code running on the frontend vs the backend
    - Backend code has access to vehicle hardware and other service APIs, as well as the filesystem (for things like persistent logging)
    - Frontend code is in charge of the display, and runs in the browser interface (instead of on the vehicle's onboard computer)

## Usage

Forking the repository will try to automatically package and upload your Extension variant to a Docker registry (Docker Hub), using the built in GitHub Action.
This process makes use of some [GitHub Variables](https://github.com/BlueOS-community/Deploy-BlueOS-Extension#input-variables) that you can configure for your fork.

It is also possible to manually run the Action (via the Actions tab), or to build and deploy the extension manually on your local machine (although this requires installing the relevant build tools and cloning the repository onto your computer).

>💡**Note:** If you are forking this repository as a starting point for creating your own [BlueOS Extension](https://blueos.cloud/docs/blueos/latest/development/extensions), it is recommended to enable `Issues` in your fork (via the `Settings` tab at the top), so that users and co-developers of your Extension can raise problems and make suggestions.

## Code organization

- `app/control.py`: motor commands and movement sequences (initialize, unlock,
  lock, deploy, retract, and stop). Edit here to change motor behavior.
- `app/motor_bus.py`: reusable Dynamixel reads, writes, error handling, telemetry,
  and port cleanup. 
- `app/state.py`: shared motion, lock, and safety state, plus the bus lock.
- `app/retract_safety.py`: reference point, braking calculations, retract checks,
  and the background safety monitor.
- `app/config.py`: hardware settings, speeds, thresholds etc..
- `app/main.py` and `app/mavlink_bridge.py`: HTTP and MAVLink interfaces that call
  the shared control functions.

## Retract limit

Each successful **Initialize** captures the winch's current signed, multi-turn
encoder position as the retraction limit. Initialize with the winch stopped
at the intended fully retracted position. 

Initializing again deliberately sets a new reference; torque enable/disable and unlocking do not change it. 

The reference is held in memory, so initialize again after an application restart. After motor
power loss/reboot, re-establish the intended reference before using the winch:
the motor's multi-turn position can reset on reboot.

The monitor commands zero velocity **before** the reference, allowing for the
configured acceleration profile, actual/commanded speed, communication delay,
and an additional margin. At higher speeds it will stop farther short of
the reference; it does not automatically creep forward afterwards. 

Braking settings are in `app/config.py` (`RETRACT_LIMIT_*`). The calculation assumes a
velocity-based profile and positive `WINCH_PROFILE_ACCELERATION`.

The existing retraction movement during unlocking is exempt from the normal limit. 

If feedback fails while guarding retraction, the controller attempts to stop,
records a safety fault, and rejects further movement until reinitialization.

This is a software travel limit, not a physical hard stop. It can be delayed
by other serial operations, and loss of the process/serial connection can
prevent stopping. Therefore the braking defaults should use a low-speed
