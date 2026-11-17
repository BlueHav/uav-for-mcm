import unittest
import numpy as np
import tree_distance

class DirectionalCohesion(unittest.TestCase):
    def reference(self):
        p=np.array([[0.,0.,0.],[6.,-3.,0.],[0.,35.,0.],[-3.,-6.,0.],[3.,-6.,0.]])
        v=np.tile([0.,6.,0.],(5,1))
        c=dict(stamp=10.,sample_stamps=[10.]*5,fresh=True,positions=p.tolist(),velocities=v.tolist(),slow_factor=.35,
               edges=[dict(parent=a,child=b,active=(a,b)==(2,0)) for a,b in tree_distance.STAR_EDGES])
        return dict(stamp=10.,speed=6.,cohesion_required=True,cohesion=c),p,v

    def cap(self,ref,member,p,desired,now=10.05):
        method=getattr(tree_distance,'directional_speed_limit',None)
        self.assertTrue(callable(method),'Direction-specific radial cohesion is missing')
        return method(ref,member,p,desired,now=now)

    def test_front_slows_and_same_heading_rear_keeps_normal_speed(self):
        r,p,v=self.reference()
        self.assertAlmostEqual(self.cap(r,2,p[2],[0.,6.,0.]),2.1)
        self.assertEqual(self.cap(r,0,p[0],[0.,6.,0.]),6.)

    def test_sideways_neighbor_is_not_counted_as_radial_catchup(self):
        r,p,v=self.reference();v[0]=[6.,0.,0.];r['cohesion']['velocities']=v.tolist()
        self.assertEqual(self.cap(r,2,p[2],[6.,6.,0.]),2.)

    def test_both_outward_directions_keep_two_and_inward_keeps_normal_speed(self):
        r,p,v=self.reference();v[0]=[0.,-6.,0.];r['cohesion']['velocities']=v.tolist()
        self.assertEqual(self.cap(r,2,p[2],[0.,6.,0.]),2.)
        self.assertEqual(self.cap(r,0,p[0],[0.,-6.,0.]),2.)
        self.assertEqual(self.cap(r,2,p[2],[0.,-6.,0.]),6.)
        self.assertEqual(self.cap(r,0,p[0],[0.,6.,0.]),6.)

    def test_oblique_outward_rate_is_bounded_by_radial_rear_rate(self):
        r,p,v=self.reference();v[0]=[0.,.2,0.];r['cohesion']['velocities']=v.tolist()
        cap=self.cap(r,2,p[2],[1.,1.,0.])
        self.assertEqual(cap,2.,'Minimum speed takes precedence over an unattainable radial rate')

    def test_hard_pause_remains_absolute_for_inward_motion(self):
        r,p,v=self.reference();r['member_speed_caps']=[6.,6.,0.,6.,6.]
        self.assertEqual(self.cap(r,2,p[2],[0.,-6.,0.]),0.)

    def test_soft_factor_does_not_multiply_an_existing_hard_cap_again(self):
        r,p,v=self.reference();r['member_speed_caps']=[6.,6.,1.,6.,6.]
        self.assertEqual(self.cap(r,2,p[2],[0.,6.,0.]),1.)

    def test_fresh_packet_cannot_hide_old_kinematic_samples(self):
        r,p,v=self.reference();r['cohesion']['sample_stamps']=[9.6]*5
        self.assertEqual(self.cap(r,2,p[2],[0.,-6.,0.]),2.)

    def test_shape_cap_is_applied_to_inward_motion_without_changing_heading(self):
        r,p,v=self.reference();r['cohesion']['formation_speed_caps']=[6.,6.,3.,6.,6.]
        self.assertEqual(self.cap(r,2,p[2],[0.,-6.,0.]),3.)

    def test_distance_limit_also_applies_to_order_recovering_parent(self):
        r,p,v=self.reference();r['cohesion']['order_recovery_members']=[2]
        r['cohesion']['formation_speed_caps']=[2.,6.,6.,6.,6.]
        self.assertAlmostEqual(self.cap(r,2,p[2],[0.,6.,0.]),2.1)
        self.assertEqual(self.cap(r,2,p[2],[0.,-6.,0.]),6.)

    def test_real_priority_conflict_does_not_bypass_another_wings_distance_gate(self):
        import json
        from pathlib import Path
        r=json.loads((Path(__file__).parent/'fixtures/order-priority-distance-conflict.json').read_text())
        self.assertGreater(r['original_direction_limit']['policy_cap_mps'],5.9)
        self.assertGreater(r['distance_m'],50.)
        self.assertIn(r['member'],r['cohesion']['order_recovery_members'])
        cap=self.cap(r,r['member'],r['position'],r['desired'],now=r['stamp'])
        self.assertGreaterEqual(cap,2.)
        self.assertLessEqual(cap,2.100001)

    def test_shape_cap_below_two_is_rejected(self):
        r,p,v=self.reference();r['cohesion']['formation_speed_caps']=[6.,6.,0.,6.,6.]
        with self.assertRaises(ValueError):self.cap(r,2,p[2],[0.,6.,0.])

    def test_all_incident_active_edges_apply_the_most_restrictive_cap(self):
        r,p,v=self.reference();v[1]=0.;r['cohesion']['velocities']=v.tolist()
        r['cohesion']['edges'][1]['active']=True
        self.assertEqual(self.cap(r,2,p[2],[0.,6.,0.]),2.)

    def test_missing_malformed_and_expired_constraints_fail_closed(self):
        r,p,v=self.reference()
        self.assertEqual(self.cap(r,2,p[2],[0.,-6.,0.],now=10.5),2.)
        r['cohesion']['fresh']=False
        self.assertEqual(self.cap(r,2,p[2],[0.,-6.,0.]),2.)
        del r['cohesion']
        with self.assertRaises(ValueError):self.cap(r,2,p[2],[0.,-6.,0.])
        r,p,v=self.reference();r['cohesion']['positions'][0][0]=float('nan')
        with self.assertRaises(ValueError):self.cap(r,2,p[2],[0.,-6.,0.])

    def test_current_own_position_is_used_instead_of_old_producer_position(self):
        r,p,v=self.reference()
        # The current member has moved to the other side of its neighbor.
        self.assertEqual(self.cap(r,2,[0.,-1.,0.],[0.,6.,0.]),6.)

if __name__=='__main__':unittest.main()
