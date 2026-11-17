import unittest
from audit_phase6 import observed_order_consumption,observed_injection_front_slow


class OrderConsumptionAudit(unittest.TestCase):
    def state(self,cap=2.,speed=2.,stamp=10.):
        return dict(velocity=[0.,speed,0.],formation=dict(direction_limit=dict(
            stamp=stamp,policy_cap_mps=cap,limited_xy=[0.,min(cap,2.)],hard_pause=False,observation_fresh=True)))

    def test_producer_cap_cannot_replace_actual_consumer_evidence(self):
        state=self.state(cap=6.);state['formation']['formation_speed']={'speed_cap':2.}
        self.assertEqual(observed_order_consumption(state,10.05),(False,False))

    def test_new_consumer_stamp_does_not_make_old_geometry_valid(self):
        state=self.state();state['formation']['direction_limit'].update(observation_fresh=False,source_stamp=9.)
        self.assertEqual(observed_order_consumption(state,10.05),(False,False))

    def test_consumer_permission_and_physical_response_are_distinct(self):
        self.assertEqual(observed_order_consumption(self.state(speed=6.),10.05),(True,False))
        self.assertEqual(observed_order_consumption(self.state(speed=2.2),10.05),(True,True))

    def test_stale_or_hard_pause_is_not_formation_consumption_evidence(self):
        self.assertEqual(observed_order_consumption(self.state(stamp=9.),10.05),(False,False))
        state=self.state();state['formation']['direction_limit']['hard_pause']=True
        self.assertEqual(observed_order_consumption(state,10.05),(False,False))

    def test_preinjection_or_stale_slowdown_cannot_prove_injection_response(self):
        self.assertFalse(observed_injection_front_slow(self.state(),10.05,11.,31.))
        self.assertTrue(observed_injection_front_slow(self.state(),10.05,9.,29.))
        state=self.state();state['formation']['direction_limit']['observation_fresh']=False
        self.assertFalse(observed_injection_front_slow(state,10.05,9.,29.))


if __name__=='__main__':unittest.main()
