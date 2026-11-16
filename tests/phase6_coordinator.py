#!/usr/bin/env python3
"""Focused real-flight fixture: start on the phase-six ground boundary.

Only the initial mission stages and high-altitude cruise acceptance are omitted.
Takeoff, sensing, CLOUD->YOPO ownership, independent planning and landing use
the production coordinator and per-aircraft nodes without teleporting in flight.
"""
import json, os, time, uuid
from pathlib import Path
import numpy as np
import rospy
from point_mission_node import Coordinator
from planning_mode import ModeGate
from std_msgs.msg import String

class LagPublisher:
    """Test-only slow flight of one leaf through the member-cap contract."""
    def __init__(self,publisher,seconds):
        self.publisher=publisher;self.seconds=seconds;self.origin=None;self.previous=None
        self.speed=float(os.environ.get('SWARM_TEST_LAG_SPEED','2'))
        if not 2.<=self.speed<=6.:raise ValueError('Invalid test slow speed')
    def publish(self,message):
        ref=json.loads(message.data)
        if ref.get('independent_terminal'):
            if self.origin is None:self.origin=ref['stamp']
            elapsed=ref['stamp']-self.origin
            active=10.<=elapsed<10.+self.seconds
            if active:
                caps=list(ref.get('member_speed_caps',[ref['speed']]*5));caps[3]=min(caps[3],self.speed)
                ref['member_speed_caps']=caps
            if active!=self.previous:
                path=Path(os.environ['SWARM_RUN_DIR'])/'lag-injection.jsonl'
                with path.open('a') as stream:stream.write(json.dumps(dict(stamp=ref['stamp'],member=3,active=active,elapsed=elapsed,speed_mps=self.speed))+'\n')
                self.previous=active
            message=String(json.dumps(ref))
        self.publisher.publish(message)

class PhaseSixCoordinator(Coordinator):
    def __init__(self):
        super().__init__()
        seconds=float(os.environ.get('SWARM_TEST_LAG_SECONDS','0'))
        if seconds>0:self.refs[0]=LagPublisher(self.refs[0],seconds)

    def command(self,msg):
        try:op=json.loads(msg.data).get('op')
        except (ValueError,TypeError):return
        if op!='phase6_start':return super().command(msg)
        with self.lock:
            if self.stage!='IDLE':return
            problem=self.ready(rospy.Time.now().to_sec())
            if problem:self.reason='Fixture waiting: '+problem;return
            self.run_id='phase6-'+uuid.uuid4().hex[:10]
            self.planning_gate=ModeGate(self.run_id)
            self.directory=Path(os.environ['SWARM_RUN_DIR'])/'Reports/mission'
            self.directory.mkdir(parents=True,exist_ok=False)
            from ten_core import register_failure_directory
            register_failure_directory(os.environ['SWARM_RUN_DIR'],self.run_id,self.directory)
            self.report=dict(run_id=self.run_id,scope='focused phase-six real flight, not full mission',
                stages=[],errors=[],passed=False,hashes=self.model.hashes,group_events=[[],[]],
                min_body_distance=1e9,max_group_distance=0.,max_altitude=0.,
                terminal_mode='independent_yopo',terminal_speed=6.,
                terminal_peak_horizontal_mps={str(i):0. for i in self.active})
            self.report['fixture_lag_seconds']=float(os.environ.get('SWARM_TEST_LAG_SECONDS','0'))
            self.report['fixture_lag_speed_mps']=float(os.environ.get('SWARM_TEST_LAG_SPEED','2'))
            self.log=(self.directory/'telemetry.jsonl').open('w',buffering=1)
            self.send('takeoff',height_mode='AGL',height=5.,speed=2.)
            self.set_stage('PHASE6_TAKEOFF')

    def step(self):
        super().step()
        if self.stage!='PHASE6_TAKEOFF':return
        ready=all(i in self.states and self.states[i]['armed'] and not self.states[i]['contact']
            and abs(self.states[i]['agl']-5.)<.25 and np.linalg.norm(self.states[i]['velocity'])<.2 for i in self.active)
        if ready:
            self.fixture_ready_since=getattr(self,'fixture_ready_since',None) or time.monotonic()
            if time.monotonic()-self.fixture_ready_since>=2.:
                p=self.positions()
                for g in self.active_groups:
                    self.groups[g].reference=p[5*g:5*g+5].copy()
                    self.groups[g].event('DESCENT_ASSEMBLED',rospy.Time.now().to_sec())
                self.set_stage('RUNNING')
        else:self.fixture_ready_since=None

    def final_evidence_problem(self):
        # A stage-six fixture cannot claim to satisfy the omitted cruise gate.
        if self.report.get('errors'):return 'Prior fixture flight error'
        if self.scene.get('collision_events')!=0 or self.scene.get('bounds_events')!=0:return 'Unity collision/bounds'
        if self.report['min_body_distance']<1. or self.report['max_group_distance']>2000. or self.report['max_altitude']>300.:return 'Flight distance/height limit'
        return None

if __name__=='__main__':
    coordinator=PhaseSixCoordinator()
    rospy.spin()
