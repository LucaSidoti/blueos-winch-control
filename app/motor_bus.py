"""
Higher level driver for Dynamixels than the SDK.
Deals with motor communication.
Control functions are in control.py, which uses this module to talk to the motors.

"""
from contextlib import contextmanager
from pathlib import Path

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

import config

def signed_16(value):
    return value - 0x10000 if value & 0x8000 else value

def signed_32(value):
    return value - 0x100000000 if value & 0x80000000 else value

class MotorBus:
    def __init__(self, port, packet):
        self.port = port
        self.packet = packet

    def close(self):
        self.port.closePort()

    def set_baudrate(self, baudrate):
        if not self.port.setBaudRate(baudrate):
            raise RuntimeError(f"Could not set bus baud rate to {baudrate}")

    def _check(self, result, error, action):
        if result != COMM_SUCCESS:
            raise RuntimeError(f"{action}: {self.packet.getTxRxResult(result)}")
        if error != 0:
            raise RuntimeError(f"{action}: {self.packet.getRxPacketError(error)}")

    def read_register(self, motor_id, address, size, *, action):
        if size not in (1, 2, 4):
            raise ValueError("Register size must be 1, 2, or 4 bytes")
        read = getattr(self.packet, f"read{size}ByteTxRx")
        value, result, error = read(self.port, motor_id, address)
        self._check(result, error, action)
        return value

    def read_register_status(self, motor_id, address, size, *, action):
        """Read a register while preserving the device status-byte diagnostic.

        Hardware-shutdown Dynamixels can answer normally while setting the Alert
        bit in every status packet.  For diagnostics we must still accept the
        returned register value; transport failures remain fatal.
        """
        if size not in (1, 2, 4):
            raise ValueError("Register size must be 1, 2, or 4 bytes")
        read = getattr(self.packet, f"read{size}ByteTxRx")
        value, result, error = read(self.port, motor_id, address)
        if result != COMM_SUCCESS:
            raise RuntimeError(f"{action}: {self.packet.getTxRxResult(result)}")
        return {
            "value": int(value),
            "device_error": int(error),
            "device_error_text": self.packet.getRxPacketError(error) if error else None,
        }

    def write_register(self, motor_id, address, value, size, *, action, checked=True):
        if size not in (1, 2, 4):
            raise ValueError("Register size must be 1, 2, or 4 bytes")
        write = getattr(self.packet, f"write{size}ByteTxRx")
        result, error = write(self.port, motor_id, address, value)
        if checked:
            self._check(result, error, action)

    def ping(self, motor_id, *, action):
        model, result, error = self.packet.ping(self.port, motor_id)
        self._check(result, error, action)
        return model

    def reboot(self, motor_id, *, action, allow_device_error=False):
        """Reboot a Dynamixel.

        A motor in hardware shutdown can acknowledge REBOOT while still returning
        its latched device-error bit in that same status packet.  For recovery we
        may accept that device error, but transport/communication failures remain
        fatal.  The caller must verify the motor after the reboot delay.
        """
        result, error = self.packet.reboot(self.port, motor_id)
        if result != COMM_SUCCESS:
            raise RuntimeError(f"{action}: {self.packet.getTxRxResult(result)}")
        if error != 0 and not allow_device_error:
            raise RuntimeError(f"{action}: {self.packet.getRxPacketError(error)}")
        return int(error)

    def ping_status(self, motor_id, *, action):
        """Ping while distinguishing transport loss from a responding motor fault.

        A Dynamixel can reply to PING with a device error bit set (for example after
        a hardware shutdown).  That still proves that the serial link and motor are
        present, so connection status must not collapse to DISCONNECTED.
        """
        model, result, error = self.packet.ping(self.port, motor_id)
        if result != COMM_SUCCESS:
            raise RuntimeError(f"{action}: {self.packet.getTxRxResult(result)}")
        return {
            "model": model,
            "device_error": int(error),
            "device_error_text": self.packet.getRxPacketError(error) if error else None,
        }

    def set_torque(self, motor_id, enabled, *, action, checked=True):
        address = (config.LOCK_ADDR_TORQUE_ENABLE if motor_id == config.LOCK_ID
                   else config.ADDR_TORQUE_ENABLE)
        self.write_register(motor_id, address, enabled, 1,
                            action=action, checked=checked)

    def set_operating_mode(self, motor_id, mode, *, action):
        address = (config.LOCK_ADDR_OPERATING_MODE if motor_id == config.LOCK_ID
                   else config.ADDR_OPERATING_MODE)
        self.write_register(motor_id, address, mode, 1, action=action)

    def set_profile_acceleration(self, motor_id, acceleration, *, action):
        address = (config.LOCK_ADDR_PROFILE_ACCELERATION if motor_id == config.LOCK_ID
                   else config.ADDR_PROFILE_ACCELERATION)
        self.write_register(
            motor_id,
            address,
            acceleration,
            4,
            action=action,
        )

    def set_profile_velocity(self, motor_id, velocity, *, action):
        self.write_register(
            motor_id,
            config.LOCK_ADDR_PROFILE_VELOCITY,
            velocity,
            4,
            action=action,
        )

    def set_velocity(self, motor_id, velocity, *, action):
        self.write_register(
            motor_id,
            config.ADDR_GOAL_VELOCITY,
            velocity & 0xFFFFFFFF,
            4,
            action=action,
        )

    def set_position(self, motor_id, position, *, action):
        self.write_register(
            motor_id,
            config.LOCK_ADDR_GOAL_POSITION,
            position,
            4,
            action=action,
        )

    def read_position(self, motor_id, *, action):
        return self.read_register(motor_id, config.ADDR_PRESENT_POSITION, 4, action=action)

    def read_velocity(self, motor_id, *, action):
        return self.read_register(motor_id, config.ADDR_PRESENT_VELOCITY, 4, action=action)

    def read_current(self, motor_id, *, action):
        return self.read_register(motor_id, config.ADDR_PRESENT_CURRENT, 2, action=action)

    def read_torque(self, motor_id, *, action):
        return self.read_register(motor_id, config.ADDR_TORQUE_ENABLE, 1, action=action)

    def read_hardware_error_status(self, motor_id, *, action):
        return self.read_register_status(
            motor_id,
            config.ADDR_HARDWARE_ERROR_STATUS,
            1,
            action=action,
        )

    def read_drive_mode(self, motor_id, *, action):
        return self.read_register(motor_id, config.ADDR_DRIVE_MODE, 1, action=action)

    def read_telemetry(self, motor_id, baudrate):
        self.set_baudrate(baudrate)
        current = self.read_current(motor_id, action=f"Read motor {motor_id} current")
        velocity = self.read_velocity(motor_id, action=f"Read motor {motor_id} velocity")
        position = self.read_position(motor_id, action=f"Read motor {motor_id} position")
        voltage = self.read_register(motor_id, config.ADDR_PRESENT_INPUT_VOLTAGE, 2,
                                     action=f"Read motor {motor_id} voltage")
        temperature = self.read_register(motor_id, config.ADDR_PRESENT_TEMPERATURE, 1,
                                         action=f"Read motor {motor_id} temperature")
        return {
            "current_a": round(signed_16(current) * config.PRESENT_CURRENT_MA_PER_UNIT / 1000.0, 3),
            "rpm": round(signed_32(velocity) * config.PRESENT_VELOCITY_RPM_PER_UNIT, 2),
            "position_deg": round(signed_32(position) * config.POSITION_DEG_PER_COUNT, 1),
            "voltage_v": round(voltage * config.PRESENT_VOLTAGE_V_PER_UNIT, 1),
            "temperature_c": int(temperature),
        }

def open_bus():
    """Open without taking a lock, for the monitor's explicit retry/cleanup path."""
    if not Path(config.DEVICE_NAME).exists():
        raise RuntimeError(f"{config.DEVICE_NAME} does not exist")
    port = PortHandler(config.DEVICE_NAME)
    if not port.openPort():
        raise RuntimeError(f"Could not open {config.DEVICE_NAME}")
    try:
        return MotorBus(port, PacketHandler(config.PROTOCOL_VERSION))
    except Exception:
        port.closePort()
        raise

@contextmanager
def session(lock):
    with lock:
        bus = open_bus()
        try:
            yield bus
        finally:
            bus.close()
