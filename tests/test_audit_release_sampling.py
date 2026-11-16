import unittest
import audit_phase6
from audit_phase6 import recorded_release_consistent

class AuditReleaseSampling(unittest.TestCase):
    def test_hard_cap_alone_does_not_prove_normal_direction_permission(self):
        method=getattr(audit_phase6,'normal_direction_permission',None)
        self.assertTrue(callable(method),'Audit still confuses hard cap with actual direction permission')
        state={'formation':{'member_speed_cap':6.,'direction_limit':{
            'stamp':10.,'limited':True,'cap_mps':2.1,'limited_xy':[0.,2.1]}}}
        self.assertFalse(method(state,10.05))
        state['formation']['direction_limit'].update(limited=False,cap_mps=6.)
        self.assertTrue(method(state,10.05))
        self.assertFalse(method(state,11.))

    def test_next_sample_can_follow_a_release_in_an_unrecorded_cycle(self):
        sample={'active':False,'distance_m':25.02,'triggers':1,'releases':1}
        self.assertTrue(recorded_release_consistent(True,0,sample))

    def test_unaccounted_release_or_counter_reset_is_rejected(self):
        self.assertFalse(recorded_release_consistent(True,0,{'active':False,'triggers':1,'releases':0}))
        self.assertFalse(recorded_release_consistent(False,2,{'active':False,'triggers':2,'releases':1}))
        self.assertFalse(recorded_release_consistent(False,0,{'active':True,'triggers':2,'releases':0}))
        self.assertFalse(recorded_release_consistent(False,0,{'active':False,'triggers':1,'releases':1},previous_triggers=2))

if __name__=='__main__':unittest.main()
