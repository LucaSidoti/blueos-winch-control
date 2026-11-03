import unittest
from unittest.mock import patch
from ctd_encoder_feedback import assess
from state import state

class EncoderFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.saved=(state.initialized,state.operational_home_counts,
                    state.motion.velocity,state.home_adjust_mode,state.home_recovery_mode)
        state.initialized=True;state.operational_home_counts=0
        state.motion.velocity=0;state.home_adjust_mode=False;state.home_recovery_mode=False
        self.data=dict(success=True,home_set=True,continuous_deployed_counts=0,
                       cable_payout_m=0.0,active=False,command_velocity=0,
                       feedback_trustworthy=True,safety_fault=None,last_error=None)
    def tearDown(self):
        (state.initialized,state.operational_home_counts,state.motion.velocity,
         state.home_adjust_mode,state.home_recovery_mode)=self.saved
    def read(self,**updates):
        return assess({**self.data,**updates},sampled_at=10,now=10.2)
    def test_encoder_candidate_not_physical_verification(self):
        r=self.read();self.assertTrue(r['encoderHomeCandidate'])
        self.assertFalse(r['homePhysicallyVerified']);self.assertFalse(r['missionContinuationAuthorized'])
    def test_stale_sample_rejected(self):
        self.assertFalse(assess(self.data,sampled_at=0,now=10)['encoderHomeCandidate'])
    def test_fault_rejected(self):
        self.assertFalse(self.read(safety_fault='fault')['encoderHomeCandidate'])
    def test_motion_rejected(self):
        self.assertFalse(self.read(command_velocity=10)['encoderHomeCandidate'])
    def test_outside_tolerance_rejected(self):
        self.assertFalse(self.read(cable_payout_m=0.2)['encoderHomeCandidate'])
    def test_uncalibrated_rejected(self):
        state.initialized=False
        self.assertFalse(self.read()['encoderHomeCandidate'])
    def test_home_adjust_rejected(self):
        state.home_adjust_mode=True
        self.assertFalse(self.read()['encoderHomeCandidate'])

if __name__ == '__main__':unittest.main()
