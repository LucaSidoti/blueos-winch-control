import unittest
from ctd_orchestrator import CTDOrchestratorPreview, PreviewInput

STATIONS = [
    {'loiterSequence': 2, 'targetDepth': 1.3, 'samplingDuration': 15},
    {'loiterSequence': 4, 'targetDepth': 1.0, 'samplingDuration': 5},
]

def observation(t, seq=2, sample=None, **overrides):
    data = dict(now=t, sequence=seq, mission_validated=True,
                mission_telemetry_fresh=True, mission_signature='validated-A',
                position_fresh=True, gps_fix_valid=True, position_sample_id=sample,
                position_error_m=1, speed_m_s=.1, depth_m=1.3,
                depth_feedback_ok=True, home_verified=False)
    data.update(overrides)
    return PreviewInput(**data)

class PreviewTests(unittest.TestCase):
    def setUp(self):
        self.p = CTDOrchestratorPreview()
        self.p.configure(STATIONS, 'validated-A', now=0)

    def dwell(self, seq=2, start=0):
        for i in range(11):
            result = self.p.step(observation(start+i, seq, sample=100+int(start)*100+i))
        return result

    def test_full_two_station_cycle(self):
        self.assertEqual(self.dwell()['lifecycle']['phase'], 'descending')
        self.assertEqual(self.p.step(observation(11,sample=111))['lifecycle']['phase'], 'sampling')
        self.assertEqual(self.p.step(observation(26,sample=112))['status'], 'fault')  # GPS gap fail closed
        self.assertFalse(self.p.snapshot()['hardwareCommandsSent'])

    def test_full_cycle_with_continuous_samples(self):
        self.dwell()
        self.p.step(observation(11,sample=111))
        for t in range(12,27):
            result=self.p.step(observation(t,sample=100+t))
        self.assertEqual(result['lifecycle']['phase'],'retracting')
        result=self.p.step(observation(27,sample=127,home_verified=True))
        self.assertEqual(result['status'],'awaiting_simulated_advance')
        with self.assertRaises(ValueError):self.p.simulate_advance(28,999)
        self.p.simulate_advance(28,4)
        for t in range(29,40):self.p.step(observation(t,4,sample=200+t,depth_m=1.0))
        self.p.step(observation(40,4,sample=240,depth_m=1.0))
        for t in range(41,47):self.p.step(observation(t,4,sample=200+t,depth_m=1.0))
        self.assertEqual(self.p.step(observation(47,4,sample=247,depth_m=1.0,home_verified=True))['status'],'awaiting_simulated_advance')
        self.assertEqual(self.p.simulate_advance(48,None)['status'],'complete')

    def test_no_gps_fix(self):
        for t in range(12):
            self.p.step(observation(t,sample=t,gps_fix_valid=False))
        self.assertNotEqual(self.p.snapshot()['lifecycle']['phase'],'descending')

    def test_repeated_sample_does_not_accumulate_dwell(self):
        self.p.step(observation(0,sample=1))
        for t in (1,2):self.p.step(observation(t,sample=1))
        self.assertNotEqual(self.p.snapshot()['lifecycle']['phase'],'descending')

    def test_mission_change_fault_latches(self):
        self.dwell()
        self.assertEqual(self.p.step(observation(11,sample=111,mission_signature='changed'))['status'],'fault')
        self.assertEqual(self.p.step(observation(12,sample=112))['status'],'fault')

    def test_home_required(self):
        self.dwell();self.p.step(observation(11,sample=111))
        for t in range(12,27):self.p.step(observation(t,sample=100+t))
        self.assertEqual(self.p.snapshot()['lifecycle']['phase'],'retracting')
        with self.assertRaises(ValueError):self.p.simulate_advance(27,4)

    def test_position_loss_during_descent_fault(self):
        self.dwell()
        self.assertEqual(self.p.step(observation(11,sample=111,position_fresh=False))['status'],'fault')

    def test_sequence_change_during_descent_fault(self):
        self.dwell()
        self.assertEqual(self.p.step(observation(11,seq=0,sample=111))['status'],'fault')

    def test_telemetry_loss_fault(self):
        self.assertEqual(self.p.step(observation(0,sample=1,mission_telemetry_fresh=False))['status'],'fault')

    def test_no_live_interfaces(self):
        self.assertFalse(self.p.snapshot()['executionEnabled'])
        self.assertFalse(self.p.snapshot()['automaticAdvanceEnabled'])

if __name__ == '__main__':unittest.main()
