#!/usr/bin/env python3
"""ROS coordinator for one count1 executor in the fixed ten-aircraft scene."""
import json
import threading
import time
from pathlib import Path
import numpy as np
import rospy
from std_msgs.msg import String
from active_single_core import SingleFlight,scene_problem,SceneProgress,forward_climb_target
from point_mission_core import PointMission
from assembly_region import validate_assembly_region,validate_single_descent

class Coordinator:
    def __init__(self):
        rospy.init_node('active_single_coordinator')
        self.scene_progress=SceneProgress()
        self.lock=threading.RLock();self.state={};self.scene={};self.scene_stamp=0.;self.seq=0;self.reason='Awaiting start';self.fault=None;self.saved_terminal=None
        self.model=PointMission(rospy.get_param('~point'))
        self.id=2;self.group=0
        evidence=validate_assembly_region(self.model.config,self.model.world)
        validate_single_descent(self.model.config,self.model.world,self.group)
        geometry=evidence['groups'][0];entry=np.asarray(geometry['entry'])[2];handoff=np.asarray(geometry['targets'])[2]
        self.model.forward_departure=True
        spawn=self.model.spawn()[self.id]
        route=[[*p,270.] for p in self.model.routes[0][1:-1]]+[entry.tolist()]
        end=[*self.model.ends[0],float(self.model.ground(self.model.ends[0][None,:])[0])+2.]
        self.flight=SingleFlight(spawn,route,entry,handoff,end,climb_target=forward_climb_target(self.model,spawn,route[0]))
        self.last_task=None
        self.output=Path(rospy.get_param('~output'));self.output.mkdir(parents=True,exist_ok=True)
        self.pub=rospy.Publisher('/drone_2/task',String,queue_size=3)
        self.permit=rospy.Publisher('/shared_pair/decision',String,queue_size=1,tcp_nodelay=True)
        self.status=rospy.Publisher('/swarm/active/status',String,queue_size=1,latch=True)
        rospy.Subscriber('/drone_2/mission/status',String,self.state_cb,queue_size=1,tcp_nodelay=True)
        rospy.Subscriber('/scene/status',String,self.scene_cb,queue_size=1)
        rospy.Subscriber('/swarm/active/command',String,self.command,queue_size=1)
        self.timer=rospy.Timer(rospy.Duration(.05),self.tick)
        rospy.on_shutdown(self.shutdown)
    def state_cb(self,msg):
        try:
            value=json.loads(msg.data)
            with self.lock:self.state=value
        except (ValueError,TypeError):pass
    def scene_cb(self,msg):
        try:
            value=json.loads(msg.data)
            with self.lock:
                self.scene=value;self.scene_stamp=rospy.Time.now().to_sec()
                self.scene_progress.observe_scene(value,self.scene_stamp)
        except (ValueError,TypeError):pass
    def problem(self,now):
        s=self.state
        if not self.scene_progress.fresh(now):return 'Scene frame progression stale or invalid'
        if not -.05<=now-self.scene_stamp<=.6:return 'Scene observation stale'
        if self.scene.get('map_sha256')!=self.model.hashes['world']:return 'Scene map fingerprint mismatch'
        if not -.05<=now-s.get('stamp',-1e9)<=.5:return 'Active status stale'
        if s.get('failure') or s.get('controller_fault'):return 'Active local controller failure'
        if not -.05<=s.get('odom_age',1e9)<=.5:return 'Active odometry stale'
        if not -.05<=s.get('plan_age',1e9)<=.8:return 'Active planner stale'
        ages=s.get('frame_ages',{})
        if set(ages)!={'front','up','down'} or any(not np.isfinite(v) or not -.05<=v<=1.2 for v in ages.values()):return 'Active sensor input stale'
        return scene_problem(self.scene,self.id,s.get('position',[]),s.get('velocity',[]),active_ids=getattr(self,'active_ids',None))
    def command(self,msg):
        with self.lock:
            try:
                op=json.loads(msg.data)['op'];now=rospy.Time.now().to_sec()
                if op in ('hold','cancel'):
                    self.fault='User requested '+op;self.flight.stage='HOLD';self.pub.publish(String(json.dumps(dict(op='hold'))));return
                if op=='pause':
                    if self.fault:raise ValueError('Faulted mission cannot pause/resume')
                    self.flight.pause(now);self.pub.publish(String(json.dumps(dict(op='hold'))));return
                if op=='resume':
                    problem=self.problem(now)
                    if problem or self.fault:raise ValueError(problem or self.fault)
                    if np.linalg.norm(self.state['velocity'])>=.2:raise ValueError('Wait for stopped hover before resume')
                    self.flight.resume(now)
                    if self.flight.stage=='TAKEOFF' and self.state.get('armed'):
                        self.last_task=dict(op='goto',height_mode='AGL',height=2.,x=float(self.flight.spawn[0]),y=float(self.flight.spawn[1]),speed=.5,_stage='TAKEOFF_RESUME')
                    if self.flight.stage!='ASSEMBLED' and self.last_task:self.pub.publish(String(json.dumps(self.last_task)))
                    return
                if op not in ('start','assemble'):raise ValueError('Use start, assemble, pause, resume, hold or cancel')
                problem=self.problem(now)
                if problem:raise ValueError(problem)
                if self.state.get('armed') or not self.state.get('contact'):raise ValueError('Start requires grounded/disarmed aircraft')
                if np.linalg.norm(np.asarray(self.state['position'])-self.flight.spawn)>.5:raise ValueError('Active spawn differs from fixed scene')
                self.last_task=self.flight.start(now,auto=op=='start')
                self.pub.publish(String(json.dumps(self.last_task)))
            except (ValueError,KeyError,TypeError) as exc:self.reason='Command rejected: '+str(exc)
    def tick(self,_):
        with self.lock:
            now=rospy.Time.now().to_sec()
            try:problem=self.problem(now)
            except Exception as exc:problem='Invalid observation: '+repr(exc)
            running=self.flight.stage not in ('IDLE','HOLD','FAILED','LANDED')
            if running and problem:self.fault=self.fault or problem;self.flight.stage='FAILED'
            self.seq+=1
            allowed=not problem and not self.fault and self.flight.stage!='PAUSED'
            self.permit.publish(String(json.dumps(dict(seq=self.seq,stamp=now,allow={'2':allowed},speed_cap_3d={'2':10.5},cap_generation={'2':0},hard_fault=self.fault,reason=problem or self.flight.stage))))
            if running and not self.fault:
                try:
                    task=self.flight.step(self.state,now)
                    if task:
                        self.last_task=task;self.pub.publish(String(json.dumps(task)))
                except (ValueError,KeyError,TypeError) as exc:self.fault=str(exc);self.flight.stage='FAILED'
            result=dict(stage=self.flight.stage,active_ids=[2],scene_count=10,reason=self.fault or problem or (self.reason if self.flight.stage=='IDLE' else self.flight.stage),passed=self.flight.stage=='LANDED' and not self.fault and not problem,stamp=now,state=self.state)
            self.status.publish(String(json.dumps(result)))
            if self.flight.stage in ('FAILED','LANDED','HOLD') and self.saved_terminal!=self.flight.stage:
                temporary=self.output/'single-result.json.tmp'
                temporary.write_text(json.dumps(result,indent=2))
                temporary.replace(self.output/'single-result.json')
                self.saved_terminal=self.flight.stage
    def shutdown(self):
        if self.flight.stage not in ('IDLE','LANDED'):self.pub.publish(String(json.dumps(dict(op='hold'))))

if __name__=='__main__':Coordinator();rospy.spin()
