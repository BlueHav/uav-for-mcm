#!/usr/bin/env python3
"""One five-agent phase coordinator; never publishes flight control commands.

Reuses uav_swarm_pro's types 4/5/6 and graph metric, with common progress
gating and collision-separated transitions for the current Unity interface.
"""
import portable_paths
import json,time,math,uuid,threading
from pathlib import Path
import numpy as np
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point
from formation_core import shape,graph_cost,graph_gradient,min_separation,transition,cruise_step

ROOT=Path((str(portable_paths.SCENE)))

class Coordinator:
    def __init__(self):
        rospy.init_node('formation_coordinator');self.lock=threading.RLock()
        self.states={};self.scene={};self.stage='IDLE';self.reason='Send start to /swarm5/command'
        self.run_id='';self.targets=None;self.queue=[];self.center=np.array([45.,-50.,12.]);self.distance=0.
        self.dwell=None;self.started=time.monotonic();self.last=time.monotonic();self.last_progress=self.last;self.best_error=float('inf')
        self.pub=rospy.Publisher('/swarm5/reference',String,queue_size=1)
        self.status_pub=rospy.Publisher('/swarm5/status',String,queue_size=1,latch=True)
        self.markers=rospy.Publisher('/swarm5/markers',MarkerArray,queue_size=1)
        self.tasks=[rospy.Publisher('/drone_%d/task'%i,String,queue_size=1) for i in range(5)]
        self.report=None;self.directory=None;self.log=None;self.max_error=0.;self.graph_error=0.
        for i in range(5):rospy.Subscriber('/drone_%d/mission/status'%i,String,self.state,callback_args=i,queue_size=1)
        rospy.Subscriber('/scene/status',String,self.scene_cb,queue_size=1)
        rospy.Subscriber('/swarm5/command',String,self.command,queue_size=1)
        self.timer=rospy.Timer(rospy.Duration(.2),self.tick)
        rospy.on_shutdown(self.shutdown)

    def state(self,msg,i):
        try:
            with self.lock:self.states[i]=json.loads(msg.data)
        except ValueError:pass

    def scene_cb(self,msg):
        try:self.scene=json.loads(msg.data)
        except ValueError:pass

    def send_tasks(self,value):
        value=dict(value,_formation_internal=True)
        for pub in self.tasks:pub.publish(String(json.dumps(value)))

    def stop_reference(self):self.pub.publish(String('{"op":"stop"}'))

    def shutdown(self):
        if self.stage not in ('IDLE','LANDED'):self.stop_reference()
        if self.log:self.log.close()

    def ready(self,now):
        for i in range(5):
            s=self.states.get(i)
            if s is None or not -.1<=now-s.get('stamp',0)<=1.2:return 'drone_%d status stale'%i
            if s.get('failure') or s.get('controller_fault'):return 'drone_%d fault: %s'%(i,s.get('failure') or s.get('controller_fault'))
            if s.get('formation',{}).get('interrupted'):return 'drone_%d individual command interrupted the group'%i
            if not -.1<=s.get('odom_age',999)<=.5 or not 0<=s.get('plan_age',999)<=.8:return 'drone_%d odometry/planner stale'%i
            p=np.asarray(s.get('position',[]));v=np.asarray(s.get('velocity',[]))
            if p.shape!=(3,) or v.shape!=(3,) or not np.all(np.isfinite(np.r_[p,v])):return 'invalid member state'
            ages=s.get('frame_ages',{})
            if set(ages)!=set(('front','up','down')) or not all(np.isfinite(v) and -.1<=v<=1.2 for v in ages.values()):return 'drone_%d depth missing/stale'%i
        if self.scene.get('drone_count')!=5:return 'Unity has not reported five drones'
        if self.scene.get('collision_events',0)>0 or self.scene.get('bounds_events',0)>0:return 'Unity collision/boundary event'
        return None

    def command(self,msg):
        with self.lock:
            try:
                op=json.loads(msg.data)['op']
                if op=='start':
                    if self.stage!='IDLE':raise ValueError('Restart the five-mode scene before a new test')
                    problem=self.ready(rospy.Time.now().to_sec())
                    if problem:raise ValueError(problem)
                    if any(s['armed'] or not s['contact'] for s in self.states.values()):raise ValueError('All five must be grounded/disarmed')
                    self.run_id=time.strftime('%Y%m%dT%H%M%S')+'-'+uuid.uuid4().hex[:6]
                    self.directory=ROOT/'Reports'/'five-runs'/self.run_id;self.directory.mkdir(parents=True)
                    (ROOT/'Runtime/five-last-run').write_text(str(self.directory))
                    self.log=open(self.directory/'telemetry.jsonl','w',buffering=1)
                    self.report=dict(run_id=self.run_id,scope='five-agent assembly/line flight/final change short test',
                                     source='uav_swarm_pro/Swarm-Formation@967a4bd local five-cluster extension',
                                     implementation='shared phase/progress plus normalized-Laplacian graph feedback and per-agent YOPO/local safety',
                                     stages=[],passed=False,errors=[],min_separation=1e9,peak_slot_error=0.,cruise_distance=30.)
                    self.send_tasks(dict(op='takeoff',height_mode='ABS',height=12,speed=.8));self.set_stage('TAKEOFF')
                elif op in ('hold','cancel'):
                    self.fail('User requested coordinated '+op)
                elif op=='land':
                    if self.stage not in ('COMPLETE','HOLD','FAILED'):raise ValueError('Land after COMPLETE or coordinated hold')
                    problem=self.ready(rospy.Time.now().to_sec())
                    if problem:raise ValueError('Cannot land a faulted/unobserved group: '+problem+'; keep holding and explicitly stop/restart the simulation')
                    self.stop_reference();self.targets=None;self.set_stage('PRELAND')
                else:raise ValueError('Use start, hold, cancel, land')
            except (ValueError,KeyError,TypeError) as exc:self.reason='Command rejected: '+str(exc)

    def set_stage(self,stage,targets=None):
        self.stage=stage;self.started=time.monotonic();self.dwell=None;self.best_error=float('inf');self.last_progress=self.started
        if targets is not None:self.targets=np.array(targets,float)
        self.reason=stage
        if self.report is not None:
            self.report['stages'].append(dict(stage=stage,stamp=rospy.Time.now().to_sec()))
            self.save()
        rospy.loginfo('SWARM5_STAGE %s',stage)

    def save(self):
        if self.report is not None:
            target=self.directory/'report.json';temp=target.with_suffix('.tmp')
            temp.write_text(json.dumps(self.report,indent=2));temp.replace(target)

    def fail(self,reason):
        if self.stage=='FAILED':return
        self.stop_reference();self.targets=None;self.reason=reason
        if self.report is not None:
            self.report['passed']=False;self.report['errors'].append(reason)
        self.set_stage('FAILED');self.reason=reason;self.save()

    def queue_transition(self,name,actual,dest):
        waypoints=transition(actual,dest)
        self.queue=[(name+'_LIFT',waypoints[0]),(name+'_TRANSFER',waypoints[1]),(name,waypoints[2])]
        self.advance()

    def advance(self):
        stage,points=self.queue.pop(0);self.set_stage(stage,points)

    def settled(self,actual,velocity):
        ok=np.max(np.linalg.norm(actual-self.targets,axis=1))<.65 and np.max(np.linalg.norm(velocity,axis=1))<.25
        if not ok:self.dwell=None;return False
        if self.dwell is None:self.dwell=time.monotonic()
        return time.monotonic()-self.dwell>=2.

    def tick(self,_):
        with self.lock:
            try:self.step()
            except Exception as exc:
                rospy.logerr('Five coordinator error: %r',exc);self.fail('Coordinator exception: '+repr(exc))

    def step(self):
        now=rospy.Time.now().to_sec();mono=time.monotonic();dt=mono-self.last;self.last=mono
        active=self.stage not in ('IDLE','FAILED','LANDED')
        problem=self.ready(now)
        if active and problem:self.fail(problem)
        if len(self.states)==5:
            actual=np.array([self.states[i]['position'] for i in range(5)]);velocity=np.array([self.states[i]['velocity'] for i in range(5)])
            spacing=min_separation(actual)
        else:actual=velocity=None;spacing=None
        if active and spacing is not None and spacing<2.:self.fail('Actual member separation below 2m')
        elapsed=mono-self.started
        if self.stage=='PRELAND' and elapsed>.8:
            self.send_tasks(dict(op='land'));self.set_stage('LANDING')
        if self.stage=='LANDING':
            if all(s['phase']=='LANDED' and s['contact'] and not s['armed'] for s in self.states.values()):self.set_stage('LANDED')
            elif elapsed>90:self.fail('Landing timed out')
        if self.stage=='TAKEOFF':
            if all(s['phase']=='HOLD' and s['armed'] for s in self.states.values()):
                self.queue_transition('ASSEMBLY',actual,self.center+shape('assembly'))
            elif elapsed>70:self.fail('Five-agent takeoff timed out')
        if self.targets is not None and self.stage not in ('FAILED','PRELAND','LANDING','LANDED'):
            if any(not s['armed'] or s['contact'] for s in self.states.values()):
                self.fail('Member disarmed or contacted ground during formation');return
            self.max_error=float(np.max(np.linalg.norm(actual-self.targets,axis=1)))
            self.graph_error=graph_cost(actual,self.targets)
            if self.max_error<self.best_error-.15:self.best_error=self.max_error;self.last_progress=mono
            if self.stage=='CRUISE':
                blocked=any(s.get('formation',{}).get('blocked',False) for s in self.states.values())
                old=self.distance;self.distance=min(30.,cruise_step(self.distance,dt,self.max_error,blocked))
                if self.distance>old:self.last_progress=mono
                self.targets=self.center+shape('line');self.targets[:,0]+=self.distance
                if self.distance>=30. and self.settled(actual,velocity):
                    end=self.center+[30,0,0];self.queue_transition('FINAL',actual,end+shape('final'))
            elif self.stage!='COMPLETE' and self.settled(actual,velocity):
                if self.queue:self.advance()
                elif self.stage=='ASSEMBLY':self.queue_transition('LINE',actual,self.center+shape('line'))
                elif self.stage=='LINE':self.set_stage('CRUISE',self.center+shape('line'));self.distance=0.
                elif self.stage=='FINAL':
                    self.report['passed']=True;self.report['final_slot_error']=self.max_error
                    self.report['network_inferences']={str(i):s['network_inferences'] for i,s in self.states.items()}
                    self.report['map_bytes']={str(i):s['map']['bytes'] for i,s in self.states.items()}
                    self.report['collision_events']=self.scene.get('collision_events');self.set_stage('COMPLETE')
            if self.stage!='COMPLETE' and (mono-self.last_progress>30. or mono-self.started>180.):
                self.fail('Stage cannot make safe progress: '+self.stage);return
            # Team coupling: graph correction uses every member's live position;
            # common progress is held whenever any member is blocked/lagging.
            correction=-4.*graph_gradient(actual,self.targets)
            norms=np.linalg.norm(correction,axis=1);correction*=np.minimum(1.,.15/np.maximum(norms,1e-9))[:,None]
            corrected=self.targets+correction
            self.pub.publish(String(json.dumps(dict(run_id=self.run_id,stage=self.stage,stamp=now,targets=corrected.tolist(),speed=.8))))
            self.visualize(now)
        status=dict(stamp=now,stage=self.stage,reason=self.reason,run_id=self.run_id,distance=self.distance,
                    max_slot_error=self.max_error,graph_cost=self.graph_error,min_separation=spacing,report=None if self.directory is None else str(self.directory/'report.json'))
        self.status_pub.publish(String(json.dumps(status)))
        if self.report is not None and spacing is not None:
            self.report['min_separation']=min(self.report['min_separation'],spacing)
            self.report['peak_slot_error']=max(self.report['peak_slot_error'],self.max_error)
            terminal=self.stage in ('COMPLETE','FAILED','LANDED')
            if self.log:
                self.log.write(json.dumps(dict(group=status,states=self.states,targets=None if self.targets is None else self.targets.tolist()))+'\n')
                if terminal:self.save()
                if terminal and mono-self.started>15:
                    self.log.close();self.log=None

    def visualize(self,now):
        a=MarkerArray();m=Marker();m.header.frame_id='world';m.header.stamp=rospy.Time.from_sec(now);m.ns='swarm5';m.id=0;m.type=Marker.LINE_LIST;m.action=Marker.ADD;m.pose.orientation.w=1;m.scale.x=.08;m.color.g=1;m.color.a=.8
        for i in range(5):
            for j in range(i):m.points.extend([Point(*map(float,self.targets[i])),Point(*map(float,self.targets[j]))])
        a.markers.append(m);t=Marker();t.header=m.header;t.ns='swarm5';t.id=1;t.type=Marker.TEXT_VIEW_FACING;t.action=Marker.ADD;t.pose.orientation.w=1;t.pose.position=Point(*map(float,self.targets.mean(axis=0)+[0,0,5]));t.scale.z=1.3;t.color.r=t.color.g=t.color.b=t.color.a=1
        t.text='FIVE: %s\nslot max %.2f m; graph %.4f'%(self.stage,self.max_error,self.graph_error);a.markers.append(t);self.markers.publish(a)

if __name__=='__main__':Coordinator();rospy.spin()
