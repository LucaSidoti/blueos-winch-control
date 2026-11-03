import unittest
from ctd_auto_validation import AutoValidation

class Audit:
    def __init__(self):
        self.phase='idle'; self.valid=False; self.age=None; self.calls=0
    def snapshot(self):
        return {'status':self.phase,'missionValidated':self.valid,'snapshotAgeSeconds':self.age}
    def start(self, stations, system, component):
        self.calls+=1; self.phase='request_list'; return True,None
class Coordinator:
    def stations_snapshot(self): return [object()]

MISSION={'telemetryFresh':True,'sourceSystem':1,'sourceComponent':1}
class Tests(unittest.TestCase):
    def setUp(self):
        self.scheduler=AutoValidation(retry_s=10,refresh_age_s=40)
        self.audit=Audit(); self.coordinator=Coordinator()
    def tick(self,t,**kwargs):
        return self.scheduler.tick(audit=self.audit,coordinator=self.coordinator,
                                   mission=kwargs.pop('mission',MISSION),now=t,**kwargs)
    def test_requests_automatically(self):
        self.assertTrue(self.tick(0)); self.assertEqual(self.audit.calls,1)
    def test_no_duplicate_while_download_active(self):
        self.tick(0); self.assertFalse(self.tick(20)); self.assertEqual(self.audit.calls,1)
    def test_revalidates_after_expiry(self):
        self.tick(0); self.audit.phase='complete'; self.audit.valid=True; self.audit.age=41
        self.assertTrue(self.tick(41)); self.assertEqual(self.audit.calls,2)
    def test_valid_snapshot_not_restarted(self):
        self.audit.valid=True; self.audit.age=10
        self.assertFalse(self.tick(0))
    def test_no_telemetry(self):
        self.assertFalse(self.tick(0,mission={**MISSION,'telemetryFresh':False}))
    def test_no_revalidation_during_ctd_cycle(self):
        self.assertFalse(self.tick(0,preview_phase='descending'))
        self.assertFalse(self.tick(0,preview_phase='retracting'))

if __name__=='__main__': unittest.main()
