import unittest
import json
import numpy as np
from pathlib import Path
from types import SimpleNamespace
from concurrent.futures import Future
from cloud_leader import goal_boundary,CloudLeaderPlanner
from swarm_native_bridge import NativePlanner,NativeTrajectory


class CloudSlowdownHorizon(unittest.TestCase):
    def test_recorded_twelve_to_four_slowdown_has_room_to_brake_forward(self):
        p=np.array([-182.07,974.84,86.11]);v=np.array([3.91,11.32,-.58])
        target=np.array([244.36219679,2210.0956674,64.15])
        goal,end=goal_boundary(p,v,target,4.)
        axis=(target-p)[:2];axis/=np.linalg.norm(axis)
        forward=float(v[:2]@axis)
        braking_distance=(forward**2-4.**2)/(2*1.2)
        self.assertGreater(np.linalg.norm(goal[:2]-p[:2]),braking_distance+forward*.5)
        self.assertAlmostEqual(np.linalg.norm(end[:2]),4.)
    def test_slow_or_accelerating_entry_retains_original_horizon(self):
        p=np.array([0.,0.,270.]);goal,_=goal_boundary(p,np.array([0.,2.,0.]),np.array([0.,1000.,270.]),4.)
        self.assertAlmostEqual(goal[1],18.)

    def test_real_native_solver_admits_forward_slowdown_with_reserved_room(self):
        class Flat:
            resolution=2.;origin=np.array([-100,-100,0]);ground=np.zeros((200,200))
            def points_clear(self,points,radius=.35):
                points=np.asarray(points)
                return (points[:,2]>radius)&(np.abs(points[:,:2]).max(axis=1)<199-radius)
        p=np.array([0.,0.,80.]);v=np.array([12.,0.,0.])
        q,e=goal_boundary(p,v,np.array([1000.,0.,80.]),4.);q[2]=80.
        binary=Path(__file__).resolve().parents[1]/'code/Swarm-formation/native/cloud_swarm_native'
        native=NativePlanner(binary)
        try:plan=native.plan(Flat(),p,v,np.zeros(3),q,e,max_speed=12.05)
        finally:native.close()
        self.assertIsNotNone(plan)
        samples=plan.samples(np.linspace(0,plan.duration,401))
        self.assertGreaterEqual(samples[:,1,0].min(),3.9)
        self.assertLessEqual(np.linalg.norm(samples[:,1,:2],axis=1).max(),12.05)

    def test_reversing_descent_is_never_published_to_delayed_followers(self):
        c=np.zeros((1,3,6));c[0,0,3]=-2.;c[0,0,4]=3.
        plan=NativeTrajectory([1.],c,0.);job=Future();job.set_result(plan)
        planner=CloudLeaderPlanner.__new__(CloudLeaderPlanner)
        planner.pending=(job,10.25,0);planner.epoch=0;planner.pending_forward=np.array([1.,0.])
        planner.next_plan=planner.active=planner.previous=None
        mission=SimpleNamespace(failure=None)
        mission.fail=lambda why:setattr(mission,'failure',why)
        planner.update(mission,10.,descent={})
        self.assertIn('reverses',mission.failure)
        self.assertIsNone(planner.active)

    def test_recorded_high_speed_turn_has_feasible_forward_native_path(self):
        root=Path(__file__).resolve().parents[1]
        record=json.loads((root/'tests/fixtures/descent-turn-boundary.json').read_text())
        p,v,a,target=[np.asarray(record[k]) for k in ('p','v','a','target')]
        axis=(target-p)[:2];axis/=np.linalg.norm(axis)
        q,end=goal_boundary(p,v,target,20.,turning=True)
        slope=(record['from_z']-record['to_z'])/record['level_after']
        q[2]=record['from_z']-slope*np.dot(q[:2]-record['origin'],axis)
        end[2]=-slope*np.linalg.norm(end[:2]);end*=20./np.linalg.norm(end)
        class Flat:
            resolution=2.;origin=np.floor(p[:2]/2)-100;ground=np.zeros((200,200))
            def points_clear(self,points,radius=.35):return np.asarray(points)[:,2]>radius
        native=NativePlanner(root/'code/Swarm-formation/native/cloud_swarm_native')
        try:plan=native.plan(Flat(),p,v,a,q,end,max_speed=20.)
        finally:native.close()
        self.assertIsNotNone(plan)
        motion=plan.samples(np.linspace(0,plan.duration,401))
        self.assertGreater((motion[:,1,:2]@axis).min(),18.)

if __name__=='__main__':unittest.main()
