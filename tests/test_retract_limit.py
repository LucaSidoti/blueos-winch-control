"""Run from the repository root: python -m unittest discover -s tests -v.

All serial communication is simulated. No motor or MAVLink connection is opened.
"""
import importlib.util
from pathlib import Path
import sys
import threading
from types import ModuleType
import unittest
from unittest.mock import Mock, patch

APP = Path(__file__).resolve().parents[1] / 'app'


class MotorBus:
    def __init__(self, config):
        self.config = config
        self.position = -12000
        self.velocity = 0
        self.torque = 0
        self.drive_mode = 0
        self.writes = []
        self.read_error = None
        self.fail_stops = False
        self.fail_next_motion = False
        self.relief_positions = None
        self.stopped = threading.Event()

    def read4ByteTxRx(self, port, motor, address):
        if address == self.read_error:
            return 0, 1, 0
        if address == self.config.ADDR_PRESENT_POSITION:
            if self.relief_positions is not None:
                self.position = next(self.relief_positions)
            value = self.position
        elif address == self.config.ADDR_PRESENT_VELOCITY:
            value = self.velocity
        else:
            raise AssertionError(f'Unexpected read: {address}')
        return value & 0xFFFFFFFF, 0, 0

    def read1ByteTxRx(self, port, motor, address):
        if address == self.config.ADDR_DRIVE_MODE:
            return self.drive_mode, 0, 0
        if address == self.config.ADDR_TORQUE_ENABLE:
            return self.torque, 0, 0
        raise AssertionError(f'Unexpected read: {address}')

    def write4ByteTxRx(self, port, motor, address, value):
        self.writes.append((motor, address, value))
        if motor == self.config.WINCH_ID and address == self.config.ADDR_GOAL_VELOCITY:
            if value == 0 and self.fail_stops:
                return 1, 0
            self.velocity = value if value < 0x80000000 else value - 0x100000000
            if value != 0 and self.fail_next_motion:
                self.fail_next_motion = False
                return 1, 0  # Motor accepted it, but acknowledgement was lost.
            if value == 0:
                self.stopped.set()
        return 0, 0

    def write1ByteTxRx(self, port, motor, address, value):
        self.writes.append((motor, address, value))
        if motor == self.config.WINCH_ID and address == self.config.ADDR_TORQUE_ENABLE:
            self.torque = value
        return 0, 0

    def getTxRxResult(self, code):
        return 'simulated communication error'


class RetractLimitTests(unittest.TestCase):
    def setUp(self):
        config_spec = importlib.util.spec_from_file_location('config', APP / 'config.py')
        self.config = importlib.util.module_from_spec(config_spec)
        config_spec.loader.exec_module(self.config)
        sdk = ModuleType('dynamixel_sdk')
        sdk.PortHandler = Mock()
        sdk.PacketHandler = Mock()
        sdk.COMM_SUCCESS = 0
        spec = importlib.util.spec_from_file_location('control_under_test', APP / 'control.py')
        self.control = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {'config': self.config, 'dynamixel_sdk': sdk}):
            spec.loader.exec_module(self.control)
        self.bus = MotorBus(self.config)
        self.port = Mock()
        for name, value in [('open_bus', Mock(return_value=self.port)),
                            ('PacketHandler', Mock(return_value=self.bus)),
                            ('Thread', Mock()), ('logger', Mock())]:
            patcher = patch.object(self.control, name, value)
            patcher.start()
            self.addCleanup(patcher.stop)
        self.control.Thread.return_value.is_alive.return_value = True
        self.addCleanup(self.stop_monitor)

    def stop_monitor(self):
        self.control._limit_monitor_shutdown.set()
        thread = self.control._limit_monitor_thread
        if isinstance(thread, threading.Thread):
            thread.join(timeout=2)
            self.assertFalse(thread.is_alive())

    def initialize(self):
        result = self.control.initialize_motor()
        self.assertTrue(result['success'], result)
        return result

    def ready(self, distance=20000):
        self.initialize()
        self.assertTrue(self.control.enable_torque()['success'])
        self.control.lock_state = 'unlocked'
        self.bus.position -= distance

    def goals(self):
        return [value for motor, address, value in self.bus.writes
                if motor == self.config.WINCH_ID and address == self.config.ADDR_GOAL_VELOCITY]

    def test_import_does_not_start_monitor_or_open_bus(self):
        self.assertIsNone(self.control._limit_monitor_thread)
        self.control.open_bus.assert_not_called()

    def test_initialization_captures_signed_position_and_reuses_monitor(self):
        self.assertEqual(self.initialize()['retract_limit']['position_counts'], -12000)
        self.bus.position = 15000
        self.assertEqual(self.initialize()['retract_limit']['position_counts'], 15000)
        self.control.Thread.assert_called_once()
        self.control.Thread.return_value.start.assert_called_once()

    def test_failed_capture_invalidates_reference(self):
        self.initialize()
        self.bus.read_error = self.config.ADDR_PRESENT_POSITION
        self.assertFalse(self.control.initialize_motor()['success'])
        self.assertIsNone(self.control.retract_limit_position)
        self.assertFalse(self.control.initialized)
        self.assertFalse(self.control.enable_torque()['success'])

    def test_invalid_profile_rejected(self):
        self.bus.drive_mode = 4
        self.assertFalse(self.control.initialize_motor()['success'])
        self.bus.drive_mode = 0
        self.config.WINCH_PROFILE_ACCELERATION = 0
        self.assertFalse(self.control.initialize_motor()['success'])

    def test_at_and_beyond_home_retract_blocked_but_deploy_allowed(self):
        self.ready(distance=0)
        for offset in [0, 20, 5000]:
            self.bus.position = self.control.retract_limit_position + offset
            with self.assertRaisesRegex(RuntimeError, 'Retract limit reached'):
                self.control.execute_retract()
            self.assertEqual(self.control.direction, 0)
            self.assertEqual(self.control.current_velocity, 0)
            self.assertFalse(any(goal > 0 for goal in self.goals()))
        result = self.control.execute_deploy()
        self.assertEqual(result['velocity'], -20)
        self.assertEqual(self.goals()[-1], (-20 & 0xFFFFFFFF))

    def test_multiturn_deployment_and_retraction(self):
        self.ready(distance=4096 * 10)
        for speed in [20, 40, 60, 80, 100, 100]:
            self.assertEqual(self.control.execute_retract()['velocity'], speed)
        for speed in [80, 60, 40, 20, 0, -20]:
            self.assertEqual(self.control.execute_deploy()['velocity'], speed)

    def test_monitor_stops_before_reference_without_another_command(self):
        self.ready()
        self.control.execute_retract()
        self.bus.position = self.control.retract_limit_position - self.control._retract_stopping_counts(20)
        self.control.check_retract_limit()
        self.assertEqual(self.control.current_velocity, 0)
        self.assertEqual(self.control.direction, 0)
        self.assertTrue(self.control.retract_limit_reached)
        self.assertLess(self.bus.position, self.control.retract_limit_position)
        self.control.check_retract_limit()
        self.assertFalse(self.control._retract_stop_pending)

    def test_braking_envelope_accounts_for_measured_speed(self):
        self.ready()
        self.control.execute_retract()
        self.bus.velocity = 100  # Faster than the command currently stored.
        self.bus.position = self.control.retract_limit_position - self.control._retract_stopping_counts(100)
        self.control.check_retract_limit()
        self.assertEqual(self.control.current_velocity, 0)
        self.assertGreater(self.control._retract_stopping_counts(100), self.control._retract_stopping_counts(20))

    def test_speed_reduction_command_cannot_bypass_limit(self):
        self.ready()
        self.control.execute_retract()
        self.control.execute_retract()
        self.bus.position = self.control.retract_limit_position
        with self.assertRaisesRegex(RuntimeError, 'Retract limit reached'):
            self.control.execute_deploy()  # Would still command positive velocity.
        self.assertEqual(self.control.current_velocity, 0)

    def test_feedback_failure_stops_and_latches_fault_until_initialization(self):
        self.ready()
        self.control.execute_retract()
        self.bus.read_error = self.config.ADDR_PRESENT_POSITION
        self.control.check_retract_limit()
        self.assertEqual(self.goals()[-1], 0)
        self.assertIsNotNone(self.control.retract_limit_fault)
        with self.assertRaisesRegex(RuntimeError, 'safety fault'):
            self.control.execute_deploy()
        self.bus.read_error = None
        self.control.check_retract_limit()
        self.assertTrue(self.control.initialize_motor()['success'])
        self.assertIsNone(self.control.retract_limit_fault)

    def test_failed_stop_is_not_reported_as_success_and_is_retried(self):
        self.ready()
        self.control.execute_retract()
        self.bus.position = self.control.retract_limit_position
        self.bus.fail_stops = True
        self.control.check_retract_limit()
        self.assertEqual(self.control.current_velocity, 20)
        self.assertTrue(self.control._retract_stop_pending)
        self.bus.fail_stops = False
        self.control.check_retract_limit()
        self.assertEqual(self.control.current_velocity, 0)
        self.assertIsNotNone(self.control.retract_limit_fault)

    def test_lost_motion_acknowledgement_triggers_stop(self):
        self.ready()
        self.bus.fail_next_motion = True
        with self.assertRaisesRegex(RuntimeError, 'communication error'):
            self.control.execute_retract()
        self.assertEqual(self.bus.velocity, 0)
        self.assertEqual(self.goals()[-2:], [20, 0])
        self.assertIsNotNone(self.control.retract_limit_fault)
        self.assertEqual(self.control.direction, 0)

    def test_unlock_exception_keeps_original_reference(self):
        self.ready(distance=0)
        self.control.lock_state = 'locked'
        reference = self.control.retract_limit_position
        self.bus.relief_positions = iter([reference, reference, reference + self.config.UNLOCK_RELIEF_COUNTS])
        with patch.object(self.control.time, 'sleep'), patch.object(self.control, 'verify_unlock', return_value=(205.0, 0.1, 0.0)):
            self.control.unlock_mechanism()
        self.bus.relief_positions = None
        self.assertEqual(self.control.lock_state, 'unlocked')
        self.assertEqual(self.control.retract_limit_position, reference)
        self.assertIn(self.config.UNLOCK_RELIEF_VELOCITY, self.goals())
        with self.assertRaisesRegex(RuntimeError, 'Retract limit reached'):
            self.control.execute_retract()
        self.assertEqual(self.control.execute_deploy()['velocity'], -20)

    def test_repeated_unlock_cannot_accumulate_extra_retraction(self):
        self.ready(distance=-self.config.UNLOCK_RELIEF_COUNTS)
        self.control.lock_state = 'locked'
        with self.assertRaisesRegex(RuntimeError, 'exceed its allowance'):
            self.control.unlock_mechanism()
        self.assertNotIn(self.config.UNLOCK_RELIEF_VELOCITY, self.goals())
        self.assertEqual(self.control.lock_state, 'locked')
        self.assertFalse(self.control.last_unlock_success)

    def test_direct_relief_cannot_bypass_normal_limit(self):
        self.ready(distance=0)
        with self.assertRaisesRegex(RuntimeError, 'only allowed during unlocking'):
            self.control.relieve_pawl_load()
        with self.assertRaisesRegex(RuntimeError, 'Retract limit reached'):
            self.control.write_velocity(20)

    def test_relief_stop_failure_closes_bus_and_monitor_retries(self):
        self.ready(distance=0)
        self.control.lock_state = 'unlocking'
        reference = self.control.retract_limit_position
        self.bus.relief_positions = iter([reference, reference + self.config.UNLOCK_RELIEF_COUNTS])
        original_write = self.bus.write4ByteTxRx
        def fail_after_start(port, motor, address, value):
            result = original_write(port, motor, address, value)
            if address == self.config.ADDR_GOAL_VELOCITY and value == 20:
                self.bus.fail_stops = True
            return result
        with patch.object(self.bus, 'write4ByteTxRx', side_effect=fail_after_start):
            with self.assertRaisesRegex(RuntimeError, 'communication error'):
                self.control.relieve_pawl_load()
        self.port.closePort.assert_called()
        self.assertEqual(self.control.current_velocity, 20)
        self.assertIsNotNone(self.control.retract_limit_fault)
        self.bus.fail_stops = False
        self.bus.relief_positions = None
        self.control.check_retract_limit()
        self.assertEqual(self.control.current_velocity, 0)

    def test_reference_not_reset_by_torque_cycle(self):
        self.ready()
        reference = self.control.retract_limit_position
        self.assertTrue(self.control.disable_torque()['success'])
        self.assertTrue(self.control.enable_torque()['success'])
        self.assertEqual(self.control.retract_limit_position, reference)

    def test_initialize_while_retracting_is_rejected(self):
        self.ready()
        self.control.execute_retract()
        reference = self.control.retract_limit_position
        self.assertFalse(self.control.initialize_motor()['success'])
        self.assertEqual(self.control.retract_limit_position, reference)

    def test_dead_monitor_blocks_new_motion(self):
        self.ready()
        self.control._limit_monitor_thread.is_alive.return_value = False
        with self.assertRaisesRegex(RuntimeError, 'monitor is not running'):
            self.control.execute_retract()

    def test_moving_encoder_cannot_be_used_as_reference(self):
        original_read = self.bus.read4ByteTxRx
        def moving_read(port, motor, address):
            if address == self.config.ADDR_PRESENT_VELOCITY:
                return 1, 0, 0
            return original_read(port, motor, address)
        with patch.object(self.bus, 'read4ByteTxRx', side_effect=moving_read):
            result = self.control.initialize_motor()
        self.assertFalse(result['success'])
        self.assertIn('stationary', result['error'])
        self.assertIsNone(self.control.retract_limit_position)

    def test_port_failure_faults_and_retries_when_connection_returns(self):
        self.ready()
        self.control.execute_retract()
        with patch.object(self.control, 'open_bus', side_effect=RuntimeError('disconnected')):
            self.control.check_retract_limit()
        self.assertEqual(self.control.current_velocity, 20)
        self.assertIn('disconnected', self.control.retract_limit_fault)
        self.control.check_retract_limit()
        self.assertEqual(self.control.current_velocity, 0)

    def test_torque_loss_requires_reinitialization(self):
        self.ready()
        self.control.execute_retract()
        self.bus.torque = 0
        self.control.check_retract_limit()
        self.assertIn('torque was lost', self.control.retract_limit_fault)
        self.assertFalse(self.control.enable_torque()['success'])

    def test_simulated_profile_brakes_before_reference_at_all_speed_levels(self):
        self.ready(distance=50000)
        reference = self.control.retract_limit_position
        original_write = self.bus.write4ByteTxRx
        def profiled_write(port, motor, address, value):
            actual_velocity = self.bus.velocity
            result = original_write(port, motor, address, value)
            self.bus.velocity = actual_velocity  # Real motor cannot stop instantly.
            return result
        acceleration = (self.config.WINCH_PROFILE_ACCELERATION
                        * self.config.DYNAMIXEL_ACCEL_RPM_PER_MIN_PER_UNIT
                        / 60 / self.config.DYNAMIXEL_RPM_PER_UNIT)
        with patch.object(self.bus, 'write4ByteTxRx', side_effect=profiled_write):
            for speed in self.config.SPEED_LEVELS:
                with self.subTest(speed=speed):
                    position = float(reference - 30000)
                    actual_velocity = float(speed)
                    self.bus.position = int(position)
                    self.bus.velocity = speed
                    self.control.write_velocity(speed)
                    for step in range(20000):
                        if step % 5 == 0:
                            self.control.check_retract_limit()
                        old_velocity = actual_velocity
                        target = self.goals()[-1]
                        actual_velocity = max(float(target), actual_velocity - acceleration * 0.01)
                        position += ((old_velocity + actual_velocity) / 2
                                     * self.config.DYNAMIXEL_RPM_PER_UNIT
                                     * self.config.WINCH_COUNTS_PER_REV / 60 * 0.01)
                        self.bus.position = int(position)
                        self.bus.velocity = round(actual_velocity)
                        self.assertLessEqual(position, reference)
                        if target == 0 and actual_velocity == 0:
                            break
                    else:
                        self.fail('Simulation never stopped')
                    self.control.check_retract_limit()

    def test_http_commands_use_guard_and_report_monitor_stop(self):
        web = ModuleType('litestar')
        web.get = web.post = lambda *args, **kwargs: lambda fn: fn
        web.Litestar = Mock()
        static = ModuleType('litestar.static_files.config')
        static.StaticFilesConfig = Mock()
        bridge = ModuleType('mavlink_bridge')
        bridge.mavlink_listener = Mock()
        spec = importlib.util.spec_from_file_location('main_under_test', APP / 'main.py')
        main = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {
            'config': self.config, 'control': self.control, 'litestar': web,
            'litestar.static_files': ModuleType('litestar.static_files'),
            'litestar.static_files.config': static, 'mavlink_bridge': bridge,
        }), patch('threading.Thread'):
            spec.loader.exec_module(main)
        self.ready(distance=0)
        self.assertFalse(main.command_retract()['success'])
        self.assertEqual(main.command_deploy()['velocity'], -20)
        main.stop_motor()
        self.bus.position -= 20000
        self.assertTrue(main.command_retract()['success'])
        self.bus.position = self.control.retract_limit_position
        self.control.check_retract_limit()
        self.assertEqual(main.motor_state()['velocity'], 0)
        self.assertTrue(main.motor_state()['retract_limit']['reached'])

    def test_mavlink_retract_uses_same_guard(self):
        mav = ModuleType('pymavlink')
        mav.mavutil = Mock()
        spec = importlib.util.spec_from_file_location('bridge_under_test', APP / 'mavlink_bridge.py')
        bridge = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {'config': self.config, 'control': self.control, 'pymavlink': mav}):
            spec.loader.exec_module(bridge)
        class EndMessages(BaseException):
            pass
        messages = []
        for pwm in [self.config.PWM_IDLE, self.config.PWM_RETRACT]:
            message = Mock(servo10_raw=pwm)
            message.get_srcSystem.return_value = 1
            message.get_srcComponent.return_value = 1
            messages.append(message)
        connection = mav.mavutil.mavlink_connection.return_value
        connection.recv_match.side_effect = messages + [EndMessages()]
        self.ready(distance=0)
        with patch('builtins.print'), self.assertRaises(EndMessages):
            bridge.mavlink_listener()
        self.assertFalse(any(goal > 0 for goal in self.goals()))
        connection.mav.command_long_send.assert_called_once()

    def test_background_monitor_really_runs(self):
        self.control.Thread = threading.Thread
        self.config.RETRACT_LIMIT_POLL_INTERVAL = 0.005
        self.ready()
        self.control.execute_retract()
        self.bus.stopped.clear()
        self.bus.position = self.control.retract_limit_position
        self.assertTrue(self.bus.stopped.wait(timeout=2), 'Background monitor did not stop motor')
        with self.control.bus_lock:
            self.assertEqual(self.control.current_velocity, 0)
            self.assertTrue(self.control.retract_limit_reached)


if __name__ == '__main__':
    unittest.main()
