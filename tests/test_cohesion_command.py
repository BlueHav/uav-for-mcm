import unittest
from unittest import mock
from types import SimpleNamespace
import numpy as np
from mission_node import Mission
from local_map import RollingLocalMap

class CohesionCommand(unittest.TestCase):
    def mission(self,cap):
        m=Mission.__new__(Mission);m.id=2;m.count=10;m.phase='TERRAIN_FOLLOW';m.speed=cap
        m.five=SimpleNamespace(reference={'independent_terminal':True,'speed':6.,'member_speed_caps':[6.,6.,cap,6.,6.]})
        method=getattr(m,'cohesion_velocity',None)
        self.assertTrue(callable(method),'Final member cap is missing from the planner/control command boundary')
        return m

    def test_low_positive_cap_limits_real_vertical_xy_correction(self):
        m=self.mission(.01);p=np.array([.25,.25,5.25])
        m.map=RollingLocalMap(size=(12,12,12));m.map.update_pose(p);m.map.state[:]=m.map.FREE;m.map.ground[:]=0.
        m.hold=p+[.5,0.,0.];m.height=5.5;m.mode='ABS';m.ground=0.
        m.vertical_plan(p,np.zeros(3))
        self.assertGreater(m.desired[0],.01)
        command=m.cohesion_velocity(m.desired)
        self.assertAlmostEqual(float(np.linalg.norm(command[:2])),.01)
        self.assertEqual(command[2],m.desired[2])

    def test_recovery_and_retreat_demands_cannot_override_latest_cap(self):
        m=self.mission(.01)
        for demand in ([.6,0.,0.],[.3,0.,.1],[1.,1.,-.2]):
            self.assertLessEqual(float(np.linalg.norm(m.cohesion_velocity(demand)[:2])),.010001)
        # Control sees a newly tightened reference before the next plan prepare.
        m.five.reference['member_speed_caps'][2]=0.
        np.testing.assert_array_equal(m.cohesion_velocity([.6,0.,.2]),[0.,0.,.2])

    def test_landing_is_outside_terminal_xy_speed_gate(self):
        m=self.mission(0.);m.phase='LAND'
        np.testing.assert_array_equal(m.cohesion_velocity([.1,0.,-.3]),[.1,0.,-.3])

    def test_real_mission_direction_gate_keeps_two_outward_and_six_inward(self):
        from tree_distance import STAR_EDGES
        m=self.mission(6.);m.p=np.array([0.,35.,0.])
        p=np.zeros((5,3));p[2]=m.p
        m.five.reference.update(stamp=10.,cohesion_required=True,cohesion=dict(
            stamp=10.,sample_stamps=[10.]*5,fresh=True,positions=p.tolist(),velocities=np.zeros((5,3)).tolist(),slow_factor=.35,
            edges=[dict(parent=a,child=b,active=(a,b)==(2,0)) for a,b in STAR_EDGES]))
        with mock.patch('mission_node.rospy.Time.now',return_value=SimpleNamespace(to_sec=lambda:10.05)):
            np.testing.assert_array_equal(m.cohesion_velocity([0.,6.,0.]),[0.,2.,0.])
            np.testing.assert_array_equal(m.cohesion_velocity([0.,-6.,0.]),[0.,-6.,0.])

    def test_positive_policy_cap_does_not_mask_a_path_stall(self):
        from tree_distance import STAR_EDGES
        m=self.mission(6.);m.p=np.array([0.,35.,0.]);p=np.zeros((5,3));p[2]=m.p
        m.five.reference.update(stamp=10.,cohesion_required=True,cohesion=dict(
            stamp=10.,sample_stamps=[10.]*5,fresh=True,positions=p.tolist(),velocities=np.zeros((5,3)).tolist(),slow_factor=.35,
            edges=[dict(parent=a,child=b,active=(a,b)==(2,0)) for a,b in STAR_EDGES]))
        with mock.patch('mission_node.rospy.Time.now',return_value=SimpleNamespace(to_sec=lambda:10.05)):
            m.cohesion_velocity([0.,6.,0.],record=True)
        self.assertFalse(m.five.cohesion_limited)
        self.assertTrue(m.five.direction_limit['limited'])
        self.assertEqual(m.five.direction_limit['cap_mps'],2.)

    def test_soft_policy_does_not_raise_a_safe_slower_requested_command(self):
        from tree_distance import STAR_EDGES
        m=self.mission(6.);m.p=np.array([0.,35.,0.]);p=np.zeros((5,3));p[2]=m.p
        m.five.reference.update(stamp=10.,cohesion_required=True,cohesion=dict(
            stamp=10.,sample_stamps=[10.]*5,fresh=True,positions=p.tolist(),velocities=np.zeros((5,3)).tolist(),slow_factor=.35,
            edges=[dict(parent=a,child=b,active=(a,b)==(2,0)) for a,b in STAR_EDGES]))
        with mock.patch('mission_node.rospy.Time.now',return_value=SimpleNamespace(to_sec=lambda:10.05)):
            command=m.cohesion_velocity([0.,.6,.2],record=True)
        np.testing.assert_array_equal(command,[0.,.6,.2])
        self.assertEqual(m.five.direction_limit['policy_cap_mps'],2.)

    def test_current_consumer_stamp_reports_stale_original_samples(self):
        from tree_distance import STAR_EDGES
        m=self.mission(6.);m.p=np.array([0.,35.,0.]);p=np.zeros((5,3));p[2]=m.p
        m.five.reference.update(stamp=10.,cohesion_required=True,cohesion=dict(
            stamp=10.,sample_stamps=[9.]*5,fresh=True,positions=p.tolist(),velocities=np.zeros((5,3)).tolist(),slow_factor=.35,
            edges=[dict(parent=a,child=b,active=(a,b)==(2,0)) for a,b in STAR_EDGES]))
        with mock.patch('mission_node.rospy.Time.now',return_value=SimpleNamespace(to_sec=lambda:10.05)):
            m.cohesion_velocity([0.,6.,0.],record=True)
        self.assertEqual(m.five.direction_limit['stamp'],10.05)
        self.assertFalse(m.five.direction_limit['observation_fresh'])
        self.assertEqual(m.five.direction_limit['policy_cap_mps'],2.)

    def test_first_inward_restart_after_long_wait_has_a_separate_failure_deadline(self):
        from tree_distance import STAR_EDGES
        m=self.mission(6.);m.p=np.array([0.,35.,0.]);m.progress_stamp=60.
        m.cohesion_wait_active=True;m.cohesion_recovery=True;m.five.cohesion_limited=True
        p=np.zeros((5,3));p[2]=m.p
        m.five.reference.update(stamp=10.,cohesion_required=True,cohesion=dict(
            stamp=10.,sample_stamps=[10.]*5,fresh=True,positions=p.tolist(),
            velocities=np.zeros((5,3)).tolist(),slow_factor=.35,
            edges=[dict(parent=a,child=b,active=(a,b)==(2,0)) for a,b in STAR_EDGES]))
        with mock.patch('mission_node.rospy.Time.now',return_value=SimpleNamespace(to_sec=lambda:10.05)),mock.patch('mission_node.time.monotonic',return_value=100.):
            command=m.cohesion_velocity([0.,-6.,0.],record=True)
        self.assertLess(command[1],0.)
        self.assertEqual(m.progress_stamp,60.,'Sensor scan progress must remain tied to measured displacement')
        age=getattr(m,'local_progress_age',None)
        self.assertTrue(callable(age),'Restart deadline is missing after a legitimate cohesion wait')
        self.assertLess(age(100.01),1.)

if __name__=='__main__':unittest.main()
