import unittest
import numpy as np
from types import SimpleNamespace
from unittest import mock
from ten_follower import TenFollower
from terminal_star_approach import StarApproach, certificates_ready, boundary_matches, settled_boundary


class TerminalStarApproach(unittest.TestCase):
    def inputs(self):
        end=np.array([[-6,-3,2],[6,-3,2],[0,0,2],[-3,-6,2],[3,-6,2]],float)
        return end-[0,20,0],np.zeros((5,3)),np.zeros((5,3)),end

    def test_synchronized_curve_preserves_boundary_and_star(self):
        p,v,a,q=self.inputs();plan=StarApproach(p,v,a,q,'run:1')
        for actual,expected in zip(plan.sample(0),(p,v,a)):np.testing.assert_allclose(actual,expected,atol=1e-8)
        for actual,expected in zip(plan.sample(plan.duration),(q,v,a)):np.testing.assert_allclose(actual,expected,atol=1e-7)
        for t in np.linspace(0,plan.duration,301):
            x,w,b=plan.sample(t)
            self.assertLessEqual(np.linalg.norm(w[:,:2],axis=1).max(),3.001)

    def test_crossing_and_outside_entry_are_rejected(self):
        p,v,a,q=self.inputs();p[[0,1]]=p[[1,0]]
        with self.assertRaises(ValueError):StarApproach(p,v,a,q,'cross')
        p,v,a,q=self.inputs();p[0,1]-=20
        with self.assertRaises(ValueError):StarApproach(p,v,a,q,'far')

    def test_all_five_fresh_matching_certificates_are_required(self):
        records=[dict(token='x',stamp=10.,clear=True) for _ in range(5)]
        self.assertTrue(certificates_ready(records,'x',10.1))
        self.assertFalse(certificates_ready(records[:4],'x',10.1))
        self.assertFalse(certificates_ready(records,'x',10.5))
        records[2]['token']='old'
        self.assertFalse(certificates_ready(records,'x',10.1))

    def test_commit_rejects_moved_or_accelerating_boundary(self):
        p,v,a,q=self.inputs();plan=StarApproach(p,v,a,q,'x')
        self.assertTrue(boundary_matches(plan,p,v,a))
        shifted=p.copy();shifted[0,0]+=.5
        self.assertFalse(boundary_matches(plan,shifted,v,a))
        moving=v.copy();moving[2,1]=.4
        self.assertFalse(boundary_matches(plan,p,moving,a))

    def test_settled_reference_cannot_hide_measured_position_error(self):
        p,v,a,q=self.inputs()
        self.assertTrue(settled_boundary(p,v,(p,v,a)))
        shifted=p.copy();shifted[4,0]+=.6
        self.assertFalse(settled_boundary(shifted,v,(p,v,a)))

    def test_existing_jerk_integrator_tracks_inside_certified_tube(self):
        from mission_core import smooth_velocity
        p,v,a,q=self.inputs();plan=StarApproach(p,v,a,q,'tracking')
        reference=p.copy();velocity=v.copy();acceleration=a.copy();error=0.
        for t in np.arange(0,plan.duration+3.,.02):
            target,feed,_=plan.sample(t)
            desired=feed+1.2*(target-reference)-.65*(velocity-feed)
            for i in range(5):
                new,acceleration[i]=smooth_velocity(velocity[i],acceleration[i],desired[i],.02,vertical_speed=7.,vertical_acc=2.5,vertical_jerk=8.)
                reference[i]+=(velocity[i]+new)*.01;velocity[i]=new
            error=max(error,float(np.linalg.norm(target-reference,axis=1).max()))
        self.assertLess(error,.25)

    def follower(self,start=12.):
        p,v,a,q=self.inputs();plan=StarApproach(p,v,a,q,'x')
        m=SimpleNamespace(desired=np.ones(3),inferences=42,detour=None,detour_scan_start=None)
        m.fail=lambda why:setattr(m,'failure',why)
        f=TenFollower.__new__(TenFollower);f.m=m;f.member_id=2
        f.reference=dict(star_approach=dict(plan.packet(),start=start))
        f.star_certificate=dict(token='x',stamp=10.,clear=True)
        return f,plan,p,v

    def test_commit_waits_for_common_start_without_neural_planning(self):
        f,plan,p,v=self.follower()
        with mock.patch('ten_follower.rospy.Time.now',return_value=SimpleNamespace(to_sec=lambda:10.1)):
            f.plan_terrain(p[2],v[2])
        np.testing.assert_array_equal(f.m.desired,np.zeros(3))
        self.assertTrue(f.star_started)
        self.assertEqual(f.star_inference_at_start,42)

    def test_committed_tracking_tube_and_reference_loss_are_faults(self):
        f,plan,p,v=self.follower(start=10.)
        with mock.patch('ten_follower.rospy.Time.now',return_value=SimpleNamespace(to_sec=lambda:10.1)):
            f.plan_terrain(p[2]+[1,0,0],v[2])
        self.assertIn('tracking tube',f.m.failure)
        f.reference={};f.m.failure=None
        f.plan_terrain(p[2],v[2])
        self.assertIn('reference lost',f.m.failure)

if __name__=='__main__':unittest.main()
