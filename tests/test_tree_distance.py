import unittest
import numpy as np
try:
    from tree_distance import TreeDistanceGate
except ImportError:
    TreeDistanceGate=None

TEMPLATE=np.array([[-6.,-3.],[6.,-3.],[0.,0.],[-3.,-6.],[3.,-6.]])

class TreeDistance(unittest.TestCase):
    def gate(self):
        self.assertIsNotNone(TreeDistanceGate,'Hierarchical distance control is not implemented')
        # Pin literal 50/25 hysteresis separately from the proactive stop guard.
        return TreeDistanceGate(TEMPLATE,trigger=50.,resume=25.,slow_factor=.35,lookahead=0.,braking_margin=0.)

    def state(self):
        return np.c_[TEMPLATE,np.zeros(5)],np.tile([0.,6.,0.],(5,1))

    def test_only_predecessor_slows_and_hysteresis_holds_until_25(self):
        gate=self.gate();p,v=self.state();p[0,:2]=[0.,-50.]
        self.assertEqual(gate.update(p,v)['speed_caps'],[6.]*5)
        p[0,1]=-50.01
        np.testing.assert_allclose(gate.update(p,v)['speed_caps'],[6.,6.,2.1,6.,6.])
        p[0,1]=-30.
        self.assertLess(gate.update(p,v)['speed_caps'][2],6.)
        p[0,1]=-25.
        self.assertEqual(gate.update(p,v)['speed_caps'],[6.]*5)

    def test_rear_follower_slows_its_own_parent_not_leader(self):
        gate=self.gate();p,v=self.state();p[3]=p[0]+[0.,-55.,0.]
        np.testing.assert_allclose(gate.update(p,v)['speed_caps'],[2.1,6.,6.,6.,6.])

    def test_second_child_keeps_parent_slow_after_first_recovers(self):
        gate=self.gate();p,v=self.state();p[0,:2]=[0.,-55.];p[1,:2]=[0.,-60.]
        gate.update(p,v);p[0,1]=-20.
        self.assertLess(gate.update(p,v)['speed_caps'][2],6.)

    def test_blocked_child_slows_parent_to_two_without_reducing_child_cap(self):
        gate=self.gate();p,v=self.state();p[0,:2]=[0.,-55.];v[0]=0.
        caps=gate.update(p,v)['speed_caps']
        self.assertEqual(caps[2],2.)
        self.assertEqual(caps[0],6.)

    def test_groups_do_not_share_latched_edges(self):
        a,b=self.gate(),self.gate();p,v=self.state();p[0,:2]=[0.,-55.];a.update(p,v)
        p,v=self.state()
        self.assertEqual(b.update(p+1000,v)['speed_caps'],[6.]*5)

    def test_missing_or_nonfinite_observation_cannot_release_parent(self):
        gate=self.gate();p,v=self.state()
        self.assertEqual(gate.update(p,v,fresh=False)['speed_caps'],[2.]*5)
        p[0,0]=np.nan
        with self.assertRaises(ValueError):gate.update(p,v)

    def test_overtaking_child_slows_actual_front_aircraft(self):
        gate=self.gate();p,v=self.state();p[0,:2]=[0.,55.]
        caps=gate.update(p,v)['speed_caps']
        self.assertLess(caps[0],6.)
        self.assertEqual(caps[2],6.)

    def test_predictive_slowdown_reserves_braking_distance(self):
        gate=TreeDistanceGate(TEMPLATE);p,v=self.state();p[0,:2]=[0.,-40.];v[0]=0.
        result=gate.update(p,v)
        self.assertEqual(result['speed_caps'][2],2.)
        self.assertLess(result['edges'][0]['distance_m'],50.)

    def test_equal_speed_pair_keeps_braking_room_for_acceleration_or_role_change(self):
        gate=TreeDistanceGate(TEMPLATE);p,v=self.state();p[0,:2]=[0.,-35.]
        result=gate.update(p,v)
        self.assertLess(result['speed_caps'][2],6.)
        self.assertGreater(result['edges'][0]['guard_distance_m'],50.)
        p[0,1]=-25.
        self.assertFalse(gate.update(p,v)['edges'][0]['active'])

    def test_actual_course_overrides_original_star_axis(self):
        gate=self.gate();p,v=self.state();p[0,:2]=[55.,-1.];v[:]=[6.,0.,0.]
        caps=gate.update(p,v)['speed_caps']
        self.assertLess(caps[0],6.)
        self.assertEqual(caps[2],6.)

    def test_stopped_pair_retains_last_measured_course(self):
        gate=self.gate();p,v=self.state();p[0,:2]=[55.,-1.];v[:]=[6.,0.,0.]
        gate.update(p,v);v[:]=0.
        result=gate.update(p,v)
        self.assertEqual(result['edges'][0]['front'],0)

    def test_latched_edge_changes_front_after_measured_turn(self):
        gate=self.gate();p,v=self.state();p[0,:2]=[0.,-55.]
        gate.update(p,v);v[:]=[0.,-6.,0.]
        result=gate.update(p,v)
        self.assertTrue(result['edges'][0]['active'])
        self.assertEqual(result['edges'][0]['front'],0)
        self.assertEqual(result['speed_caps'][2],6.)

    def test_rotation_and_vertical_offset_do_not_change_xy_policy(self):
        angle=.9;r=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
        gate=TreeDistanceGate(TEMPLATE@r.T);p,v=self.state();p[0,1]=-55.
        p[:,:2]=p[:,:2]@r.T;v[:,:2]=v[:,:2]@r.T;p[:,2]=np.arange(5)*100.
        self.assertAlmostEqual(gate.update(p,v)['speed_caps'][2],2.1)

if __name__=='__main__':unittest.main()
