import unittest
from unittest import mock
from types import SimpleNamespace
import numpy as np
from quadrotor_msgs.msg import PositionCommand
from mission_node import Mission
from point_mission_core import PointMission
from point_mission_engine import GroupFlight
from pathlib import Path


class RelayTakeoverReference(unittest.TestCase):
    def mission(self):
        import threading
        m=Mission.__new__(Mission);m.lock=threading.RLock();m.relay_active=True
        m.p=np.array([10.,20.,30.]);m.v=np.array([0.,4.,0.]);m.refa=np.zeros(3)
        m.last_control_reference=dict(stamp=1.,acceleration=[0.,0.,0.])
        m.set_phase=lambda *a:None;m.fail=lambda reason:setattr(m,'failure',reason);m.failure=None
        return m

    def reference(self,stamp=10.):
        return dict(stamp=stamp,position=[10.,20.,30.],velocity=[0.,4.,0.],acceleration=[.2,-.3,.1])

    def test_actual_relay_command_updates_the_frozen_own_control_telemetry(self):
        m=self.mission();callback=getattr(m,'observe_relay_command',None)
        self.assertTrue(callable(callback),'Actual relay command observation is missing')
        msg=PositionCommand();msg.header.stamp=SimpleNamespace(to_sec=lambda:10.)
        for field,values in (('position',[10.,20.,30.]),('velocity',[0.,4.,0.]),('acceleration',[.2,-.3,.1])):
            for axis,value in zip(('x','y','z'),values):setattr(getattr(msg,field),axis,value)
        with mock.patch('mission_node.rospy.Time.now',return_value=SimpleNamespace(to_sec=lambda:10.05)):
            callback(msg)
        np.testing.assert_allclose(m.last_control_reference['acceleration'],[.2,-.3,.1])
        self.assertEqual(m.last_control_reference['stamp'],10.)
        msg.header.stamp=SimpleNamespace(to_sec=lambda:9.95)
        with mock.patch('mission_node.rospy.Time.now',return_value=SimpleNamespace(to_sec=lambda:10.05)):callback(msg)
        self.assertEqual(m.last_control_reference['stamp'],10.,'A reordered relay sample must not replace the latest command')
        m.relay_active=False;before=m.last_control_reference
        with mock.patch('mission_node.rospy.Time.now',return_value=SimpleNamespace(to_sec=lambda:10.05)):callback(msg)
        self.assertIs(m.last_control_reference,before)

    def test_finish_relay_inherits_fresh_issued_acceleration(self):
        m=self.mission();m.last_relay_reference=self.reference()
        with mock.patch('mission_node.rospy.Time.now',return_value=SimpleNamespace(to_sec=lambda:10.05)):
            m.finish_relay()
        self.assertFalse(m.relay_active)
        np.testing.assert_allclose(m.refa,[.2,-.3,.1])
        np.testing.assert_allclose(m.refp,m.p);np.testing.assert_allclose(m.refv,m.v)

    def test_missing_or_stale_actual_boundary_faults_instead_of_using_zero(self):
        for reference in (None,self.reference(9.)):
            m=self.mission();m.last_relay_reference=reference
            with mock.patch('mission_node.rospy.Time.now',return_value=SimpleNamespace(to_sec=lambda:10.05)):
                m.finish_relay()
            self.assertIsNotNone(m.failure)

    def test_actual_command_reference_rejects_missing_stale_future_and_nonfinite(self):
        from relay_core import validated_control_reference
        for ref in (None,self.reference(9.),self.reference(11.),dict(self.reference(),acceleration=[np.nan,0.,0.])):
            with self.assertRaises(ValueError):validated_control_reference(ref,10.05)
        self.assertEqual(validated_control_reference(self.reference(),10.05)['acceleration'],[.2,-.3,.1])

    def test_takeover_approach_requests_four_before_rearranging(self):
        model=PointMission(Path(__file__).resolve().parents[1]/'config/point.json')
        model.cruise_speed=20.;g=GroupFlight(model,0);g.relay=True;g.phase='RELAY_DESCENT'
        g.descent_turn=True;g.descent_profile={'to_z':60.}
        p=model.descent(0)[1].copy();p[:,:2]-=np.array([0.,1000.]);p[:,2]=60.
        g.reference=p.copy();g.merge_entry_acceleration=np.zeros((5,3))
        ref=g.relay_descent(p,np.tile([0.,6.5,0.],(5,1)),10.,False,False)
        self.assertLessEqual(ref['speed'],4.)
        self.assertIsNone(getattr(g,'relay_cut',None))

    def test_missing_actual_acceleration_waits_before_cut_and_faults_at_cut(self):
        model=PointMission(Path(__file__).resolve().parents[1]/'config/point.json');model.cruise_speed=20.
        g=GroupFlight(model,0);g.relay=True;g.phase='RELAY_DESCENT';g.descent_turn=True;g.descent_profile={'to_z':60.}
        p=model.descent(0)[1].copy();p[:,:2]-=[0.,1000.];p[:,2]=60.;g.reference=p.copy()
        g.merge_entry_acceleration=None;v=np.tile([0.,4.,0.],(5,1))
        g.relay_descent(p,v,10.,False,False)
        self.assertIsNone(getattr(g,'relay_cut',None))
        g.relay_cut=10.5
        with self.assertRaises(ValueError):g.relay_descent(p,v,10.5,False,False)

    def test_real_narrow_line_opens_lanes_without_immediate_fast_forward_motion(self):
        from moving_merge import RelayStarMerge
        from relay_core import GROUP_ORDER
        model=PointMission(Path(__file__).resolve().parents[1]/'config/point.json')
        direction=np.array([.35,np.sqrt(1.-.35**2)]);p=np.zeros((5,3))
        leader=model.assembly_anchor(0)-1000.*direction
        for rank,i in enumerate(GROUP_ORDER[0]):p[i,:2]=leader-rank*12.*direction
        p[:,2]=float(model.ground(leader[None,:])[0])+60.
        plan=RelayStarMerge(model,0,p,np.tile(np.r_[4.*direction,-.35],(5,1)),np.zeros((5,3)))
        q,v,a=plan.sample(12.)
        np.testing.assert_allclose((q[:,:2]-p[:,:2])@direction,48.,atol=1e-7)
        samples=[plan.sample(t) for t in np.linspace(0.,12.,241)]
        self.assertLessEqual(max(np.linalg.norm(s[1][:,:2],axis=1).max() for s in samples),6.5)

    def test_measured_failed_takeover_sequence_waits_for_the_lower_approach_speed(self):
        import json
        history=json.loads((Path(__file__).parent/'fixtures/full-takeover-speed-sequence.json').read_text())
        model=PointMission(Path(__file__).resolve().parents[1]/'config/point.json');model.cruise_speed=20.
        g=GroupFlight(model,0);g.relay=True;g.phase='RELAY_DESCENT';g.descent_turn=True;g.descent_profile={'to_z':60.}
        g.merge_entry_acceleration=np.zeros((5,3))
        # This regression isolates measured speed eligibility. It deliberately
        # does not claim the old frozen mission A was the actual relay command.
        for row in history['rows']:
            p=np.asarray(row['positions']);v=np.asarray(row['velocities']);g.reference=p.copy()
            g.relay_descent(p,v,row['stamp'],False,False)
            self.assertIsNone(getattr(g,'relay_cut',None))


if __name__=='__main__':unittest.main()
