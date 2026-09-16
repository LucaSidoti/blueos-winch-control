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
  lock, deploy, retract, and stop). Start here when editing motor behavior.
- `app/motor_bus.py`: reusable Dynamixel reads, writes, error handling, telemetry,
  and port cleanup.
- `app/state.py`: shared motion, lock, and safety state, plus the bus lock.
- `app/retract_safety.py`: reference capture, braking calculations, retract checks,
  and the background safety monitor.
- `app/config.py`: hardware settings, speeds, and thresholds.
- `app/main.py` and `app/mavlink_bridge.py`: HTTP and MAVLink interfaces that call
  the shared control functions.

## Retract limit

Each successful **Initialize** captures the winch's current signed, multi-turn
encoder position as the normal retract limit. Initialize with the winch stopped
at the intended fully retracted position. Initializing again deliberately sets a
new reference; torque enable/disable and unlocking do not change it. The reference
is held in memory, so initialize again after an application restart. After motor
power loss/reboot, re-establish the intended reference before using the winch:
the motor's multi-turn position can reset on reboot.

Normal positive-velocity commands are checked against this reference, whether
issued through HTTP, MAVLink, or `control.write_velocity()`. A background monitor
starts on initialization and checks ongoing retraction every 50 ms, independent
of the browser and further commands. Motor commands and monitor checks share a
reentrant bus lock. Importing `control.py` alone starts no thread or movement.

The monitor commands zero velocity **before** the reference, allowing for the
configured acceleration profile, actual/commanded speed, communication delay,
and an additional margin. At higher speeds it therefore stops farther short of
the reference; it does not automatically creep forward afterwards. Braking
settings are in `app/config.py` (`RETRACT_LIMIT_*`). The calculation assumes a
velocity-based profile and positive `WINCH_PROFILE_ACCELERATION`; initialization
rejects an incompatible profile or a moving encoder reading.

The existing 6-degree retract movement during unlocking is explicitly exempt
from the normal limit. Its requested endpoint may extend only as far as the
initial reference plus `UNLOCK_RELIEF_COUNTS`. The reference is never advanced,
and another unlock is rejected if it would extend that allowance. Deployment
away from the normal limit remains available when unlocked. As with normal
motion, physical stopping distance can exceed the requested relief movement.

If feedback fails while guarding retraction, the controller attempts to stop,
records a safety fault, and rejects further movement until reinitialization.
Failed stop commands are retried by the monitor, and the application does not
report a successful stop based on a failed write. Stop and torque-disable remain
available during a fault. Resolve the cause and confirm the winch is stationary
before reinitializing. The interface displays **RETRACT LIMIT**, **STOPPING**, or
**SAFETY FAULT**; `/motor/state` includes a `retract_limit` diagnostic object.

This is a software travel limit, not a physical hard stop. Polling can be delayed
by other serial operations, and loss of the process or serial connection can
prevent stopping. The braking defaults require low-speed, unloaded validation
on the real mechanism before relying on them. A hardware limit switch/interlock
is needed for an independent physical travel guarantee. See the
[ROBOTIS XW540 manual](https://emanual.robotis.com/docs/en/dxl/x/xw540-t140/)
for velocity mode, profile acceleration, and encoder reset behavior.

Run the hardware-free regression tests from the repository root:

```sh
python -m unittest discover -s tests -v
```

The tests simulate the serial bus, command paths, unlock exception, braking,
communication failures, and background monitoring; they do not operate motors.
