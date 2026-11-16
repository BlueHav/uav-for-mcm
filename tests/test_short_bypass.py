import time,unittest
from unittest import mock
from types import SimpleNamespace
import numpy as np
from local_map import RollingLocalMap
from mission_node import Mission


class ShortBypass(unittest.TestCase):
    def mission(self, occupied=False):
        m=Mission.__new__(Mission)
        p=np.array([0.25,0.25,5.25])
        m.map=RollingLocalMap(size=(12,12,12),resolution=.5)
        m.map.update_pose(p);m.map.state[:]=m.map.FREE;m.map.ground[:]=.25
        m.count=10;m.id=2;m.p=p.copy();m.mode='AGL';m.phase='TERRAIN_FOLLOW';m.height=5.;m.speed=6.
        m.goal=np.array([100.,0.25,5.25]);m.detour=np.array([p,p+[.275,0,0],p+[4,0,0]])
        # Only the nearby path vertex is visible around the obstacle corner.
        # Real map occupancy blocks the long shortcut, without mocking safety.
        cell=np.floor((p+[1.5,0,0])/.5).astype(int)-m.map.origin
        m.map.state[tuple(cell)]=m.map.OCCUPIED
        if occupied:
            cell=np.floor((p+[.5,0,0])/.5).astype(int)-m.map.origin
            m.map.state[tuple(cell)]=m.map.OCCUPIED
        m.five=SimpleNamespace(reference={'independent_terminal':True,'speed':6.})
        m.refv=np.zeros(3);m.refa=np.zeros(3);m.desired=np.zeros(3)
        m.yaw=0.;m.heading=0.;m.detour_scan_start=None
        m.progress_stamp=time.monotonic()-5.
        m.frames={'front':(0.,None)};m.rot=np.eye(3)
        def unnecessary_inference(*args,**kwargs):
            self.fail('Validated 0.275 m waypoint incorrectly enters minimum-two-sample neural corridor selection')
        m.infer=unnecessary_inference
        return m,p

    def test_observed_short_vertex_can_be_tracked_without_network_sampling(self):
        m,p=self.mission();before=m.map.state.copy()
        m.horizontal_plan(p,np.zeros(3))
        self.assertGreater(m.desired[0],0.)
        self.assertLessEqual(np.linalg.norm(m.desired),.6)
        np.testing.assert_array_equal(m.map.state,before)

    def test_blocked_short_vertex_does_not_command_forward(self):
        m,p=self.mission(occupied=True)
        # The ordinary recovery remains stopped while observing another heading.
        m.detour_scan_anchor=p.copy();m.detour_scan_step=0;m.detour_scan_observed=None
        m.horizontal_plan(p,np.zeros(3))
        self.assertEqual(float(np.linalg.norm(m.desired)),0.)

    def test_minimum_two_policy_allows_observed_short_vertex_to_progress(self):
        from tree_distance import STAR_EDGES
        m,p=self.mission();positions=np.tile(p,(5,1));positions[0]=p+[-35.,0.,0.]
        m.five.reference.update(stamp=10.,cohesion_required=True,cohesion=dict(stamp=10.,sample_stamps=[10.]*5,fresh=True,
            positions=positions.tolist(),velocities=np.zeros((5,3)).tolist(),slow_factor=.35,
            edges=[dict(parent=a,child=b,active=(a,b)==(2,0)) for a,b in STAR_EDGES]))
        with mock.patch('mission_node.rospy.Time.now',return_value=SimpleNamespace(to_sec=lambda:10.05)):
            m.horizontal_plan(p,np.zeros(3))
        self.assertGreater(m.desired[0],0.)
        self.assertLessEqual(float(np.linalg.norm(m.desired[:2])),.6)
        self.assertIsNone(m.detour_scan_start)

    def attach_cohesion(self,m,p,neighbor):
        from tree_distance import STAR_EDGES
        positions=np.tile(p,(5,1));positions[0]=neighbor
        m.five.reference.update(stamp=10.,cohesion_required=True,cohesion=dict(
            stamp=10.,sample_stamps=[10.]*5,fresh=True,positions=positions.tolist(),
            velocities=np.zeros((5,3)).tolist(),slow_factor=.35,
            edges=[dict(parent=a,child=b,active=(a,b)==(2,0)) for a,b in STAR_EDGES]))

    def test_scan_skips_unknown_far_ground_and_uses_observed_inward_short_point(self):
        m,p=self.mission();m.map.state[:]=m.map.FREE
        self.attach_cohesion(m,p,p+[0.,35.,0.])
        self.assertFalse(np.isfinite(m.map.ground_at(*(p[:2]+[0.,12.]))))
        with mock.patch('mission_node.rospy.Time.now',return_value=SimpleNamespace(to_sec=lambda:10.05)):
            self.assertTrue(m.scan_bypass_course(p,np.pi/2.))
        self.assertGreater(m.detour[-1,1],p[1])
        self.assertLessEqual(m.detour[-1,1],p[1]+5.001)

    def test_minimum_two_policy_allows_safe_direct_near_goal_shortcut(self):
        m,p=self.mission();m.map.state[:]=m.map.FREE;m.goal=p+[2.,0.,0.]
        m.detour=np.array([p,p+[-2.,0.,0.]]);m.yaw=np.pi
        self.attach_cohesion(m,p,p+[-35.,0.,0.])
        with mock.patch('mission_node.rospy.Time.now',return_value=SimpleNamespace(to_sec=lambda:10.05)):
            m.horizontal_plan(p,np.zeros(3))
        self.assertIsNone(m.detour)
        self.assertGreater(m.desired[0],0.)
        self.assertLessEqual(float(np.linalg.norm(m.desired[:2])),2.)

if __name__=='__main__':unittest.main()
