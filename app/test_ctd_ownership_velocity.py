import unittest
from unittest.mock import patch
from ctd_command_ownership import ownership, command_as
from ctd_measured_velocity import assess_raw
import config

class OwnershipVelocityTests(unittest.TestCase):
    def test_manual_command_allowed(self):
        with command_as('manual'):
            ownership.authorize(-100)
    def test_autonomous_command_blocked(self):
        with command_as('autonomous'):
            with self.assertRaises(RuntimeError):
                ownership.authorize(-100)
    def test_stop_allowed_for_any_requester(self):
        with command_as('autonomous'):
            ownership.authorize(0)
    def test_autonomous_acquisition_disabled(self):
        with self.assertRaises(RuntimeError):
            ownership.acquire_autonomous()
    def test_bus_write_gate(self):
        import sys, types
        fake = types.ModuleType("dynamixel_sdk")
        fake.COMM_SUCCESS = 0
        fake.PacketHandler = lambda *args: None
        fake.PortHandler = lambda *args: None
        with patch.dict(sys.modules, {"dynamixel_sdk": fake}):
            from motor_bus import MotorBus
        class Packet:
            def write4ByteTxRx(self, port, motor_id, address, value):
                self.calls += 1
                return 0, 0
            calls = 0
        packet = Packet()
        bus = MotorBus(None, packet)
        with command_as('autonomous'):
            with self.assertRaises(RuntimeError):
                bus.set_velocity(config.WINCH_ID, -20, action='test')
        self.assertEqual(packet.calls, 0)
        with command_as('manual'):
            bus.set_velocity(config.WINCH_ID, -20, action='test')
        self.assertEqual(packet.calls, 1)
    def test_measured_stopped(self):
        r=assess_raw(0,sampled_at=5,now=5.1)
        self.assertTrue(r['measuredStoppedCandidate'])
        self.assertFalse(r['homePhysicallyVerified'])
    def test_measured_moving(self):
        self.assertFalse(assess_raw(100,sampled_at=5,now=5.1)['measuredStoppedCandidate'])
    def test_stale_measurement_fails_closed(self):
        self.assertFalse(assess_raw(0,sampled_at=5,now=8)['measuredStoppedCandidate'])
    def test_negative_velocity_signed(self):
        r=assess_raw(0xffffffff,sampled_at=5,now=5.1)
        self.assertLess(r['measuredVelocityRpm'],0)
    def test_read_error_fails_closed(self):
        from ctd_measured_velocity import read
        import sys, types
        fake = types.ModuleType('motor_bus')
        fake.session = lambda lock: (_ for _ in ()).throw(RuntimeError('bus offline'))
        with patch.dict(sys.modules, {'motor_bus': fake}):
            r=read()
        self.assertFalse(r['measuredStoppedCandidate'])

if __name__=='__main__':unittest.main()
