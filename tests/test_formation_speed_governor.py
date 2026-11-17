import unittest
import numpy as np
try:
    from formation_speed_governor import FormationSpeedGovernor
except ImportError:
    FormationSpeedGovernor=None

TEMPLATE=np.array([[-6.,-3.],[6.,-3.],[0.,0.],[-3.,-6.],[3.,-6.]])

class FormationSpeed(unittest.TestCase):
    def governor(self):
        self.assertIsNotNone(FormationSpeedGovernor,'Swarm-Formation coefficient speed governor is missing')
        return FormationSpeedGovernor(TEMPLATE,minimum_speed=2.,nominal_speed=6.)

    def state(self):return np.c_[TEMPLATE,np.zeros(5)],np.tile([0.,6.,0.],(5,1))

    def test_correct_star_has_zero_coefficient_and_full_speed(self):
        g=self.governor();p,v=self.state();r=g.update(p,v,10.)
        self.assertAlmostEqual(r['graph_cost'],0.,places=12)
        self.assertEqual(r['speed_caps'],[6.]*5)
        self.assertEqual(r['culprit_members'],[])

    def test_member_repair_identifies_the_single_displaced_aircraft(self):
        g=self.governor();p,v=self.state();p[0,1]+=20.
        r=g.update(p,v,10.)
        self.assertGreater(r['graph_cost'],0.)
        self.assertEqual(int(np.argmax(r['repair_improvements'])),0)
        self.assertAlmostEqual(sum(r['row_contributions']),r['graph_cost'],places=12)
        self.assertEqual(r['speed_caps'][0],2.)
        self.assertEqual(r['speed_caps'][2],6.)

    def test_leader_can_be_identified_as_the_shape_error_source(self):
        g=self.governor();p,v=self.state();p[2,1]+=20.
        r=g.update(p,v,10.)
        self.assertEqual(int(np.argmax(r['repair_improvements'])),2)
        self.assertEqual(r['speed_caps'][2],2.)
        self.assertEqual(r['speed_caps'][0],6.)

    def test_lagging_leaf_keeps_speed_while_its_parent_slows(self):
        g=self.governor();p,v=self.state();p[3,1]-=20.
        r=g.update(p,v,10.)
        self.assertEqual(r['speed_caps'][0],2.)
        self.assertEqual(r['speed_caps'][3],6.)

    def test_predictive_order_constraint_slows_the_closing_child(self):
        g=self.governor();p,v=self.state();v[2]=[0.,2.,0.]
        r=g.update(p,v,10.)
        self.assertEqual(r['speed_caps'][0],2.)
        self.assertEqual(r['speed_caps'][1],2.)
        self.assertTrue(r['order_edges'][0]['predictive_risk'])

    def test_scaled_star_still_needs_metric_spacing_even_with_zero_graph_cost(self):
        g=self.governor();p,v=self.state();p[:,:2]*=3.
        r=g.update(p,v,10.)
        self.assertAlmostEqual(r['graph_cost'],0.,places=12)
        self.assertLess(min(r['speed_caps']),6.)

    def test_all_governor_outputs_stay_between_two_and_six(self):
        g=self.governor();rng=np.random.RandomState(17)
        for k in range(30):
            p,v=self.state();p[:,:2]+=rng.normal(0,15,(5,2));v[:,:2]=rng.normal(0,4,(5,2))
            caps=np.array(g.update(p,v,10.+k*.05)['speed_caps'])
            self.assertTrue(np.isfinite(caps).all())
            self.assertTrue(np.all((caps>=2.)&(caps<=6.)))

    def test_nonfinite_observation_is_rejected(self):
        g=self.governor();p,v=self.state();p[0,0]=np.nan
        with self.assertRaises(ValueError):g.update(p,v,10.)

    def test_order_uses_measured_turn_without_waiting_for_template_rotation(self):
        g=self.governor();p,v=self.state()
        p[:,:2]=-(p[:,:2]@np.array([[0.,-1.],[1.,0.]]))
        v[:]=[6.,0.,0.]
        r=g.update(p,v,10.)
        self.assertAlmostEqual(r['graph_cost'],0.,places=12)
        self.assertTrue(r['order_edges'][0]['actual_violation'])
        self.assertEqual(r['speed_caps'][0],2.)
        self.assertEqual(r['speed_caps'][2],6.)

if __name__=='__main__':unittest.main()
