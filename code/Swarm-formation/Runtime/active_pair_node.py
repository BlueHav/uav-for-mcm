#!/usr/bin/env python3
"""Independent count1 missions supervised using real global odometry."""
import json,threading
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from active_single_core import SingleFlight,SceneProgress,forward_climb_target
from active_single_node import Coordinator as SingleCoordinator
from active_pair_core import PairSafety,CoordinatedClimbBudget
from point_mission_core import PointMission
from pair_climb_geometry import pair_climb_geometry
from assembly_region import validate_assembly_region,validate_single_descent

class PairCoordinator:
    ids=(2,7)
    def __init__(self):
        rospy.init_node('active_pair_coordinator');self.lock=threading.RLock();self.states={};self.odometry={};self.control_proofs={};self.scene={};self.scene_stamp=0.;self.scene_progress=SceneProgress();self.fault=None;self.reason='Awaiting start';self.last_tasks={};self.saved=None;self.auto_start=False
        self.model=PointMission(rospy.get_param('~point'));self.model.forward_departure=True
        geometry=validate_assembly_region(self.model.config,self.model.world);self.flights={};self.safety=PairSafety()
        self.parallel_climb_geometry=pair_climb_geometry(self.model)
        for group,i in enumerate(self.ids):
            validate_single_descent(self.model.config,self.model.world,group)
            entry=np.asarray(geometry['groups'][group]['entry'])[2];handoff=np.asarray(geometry['groups'][group]['targets'])[2]
            spawn=self.model.spawn()[i];route=[[*p,270.] for p in self.model.routes[group][1:-1]]+[entry.tolist()]
            end=[*self.model.ends[group],float(self.model.ground(self.model.ends[group][None,:])[0])+2.]
            self.flights[i]=SingleFlight(spawn,route,entry,handoff,end,np.asarray(self.parallel_climb_geometry['targets'][group]))
        self.output=Path(rospy.get_param('~output'));self.output.mkdir(parents=True,exist_ok=True)
        self.tasks={i:rospy.Publisher('/drone_%d/task'%i,String,queue_size=3) for i in self.ids}
        self.permit=rospy.Publisher('/shared_pair/decision',String,queue_size=1,tcp_nodelay=True)
        self.status=rospy.Publisher('/swarm/active/status',String,queue_size=1,latch=True)
        for i in self.ids:
            rospy.Subscriber('/drone_%d/mission/status'%i,String,self.state_cb,callback_args=i,queue_size=1,tcp_nodelay=True)
            rospy.Subscriber('/drone_%d/odom'%i,Odometry,self.odom_cb,callback_args=i,queue_size=1,tcp_nodelay=True)
            rospy.Subscriber('/drone_%d/parallel_climb/control_reference'%i,String,self.proof_cb,callback_args=i,queue_size=1,tcp_nodelay=True)
        rospy.Subscriber('/scene/status',String,self.scene_cb,queue_size=1)
        rospy.Subscriber('/swarm/active/command',String,self.command,queue_size=1)
        self.timer=rospy.Timer(rospy.Duration(.05),self.tick);rospy.on_shutdown(self.shutdown)
    def state_cb(self,msg,i):
        try:
            value=json.loads(msg.data)
            with self.lock:self.states[i]=value
        except (ValueError,TypeError):pass
    def proof_cb(self,msg,i):
        try:
            value=json.loads(msg.data)
            with self.lock:
                # Preserve the actual control source stamp. Arrival time is not evidence.
                previous=self.control_proofs.get(i)
                if not isinstance(value,dict) or not np.isfinite(value.get('stamp',float('nan'))):return
                if previous is None or value['stamp']>previous['stamp']:self.control_proofs[i]=value
        except (ValueError,TypeError):pass
    def odom_cb(self,msg,i):
        p=msg.pose.pose.position;v=msg.twist.twist.linear
        with self.lock:
            value=dict(p=[p.x,p.y,p.z],v=[v.x,v.y,v.z],stamp=msg.header.stamp.to_sec(),frame=msg.header.frame_id)
            previous=self.odometry.get(i)
            if previous is not None:
                interval=value['stamp']-previous['stamp']
                if interval>0:value.update(acceleration=((np.asarray(value['v'])-previous['v'])/interval).tolist(),acceleration_interval=interval)
            self.odometry[i]=value
    def scene_cb(self,msg):
        try:
            value=json.loads(msg.data)
            with self.lock:self.scene=value;self.scene_stamp=rospy.Time.now().to_sec();self.scene_progress.observe_scene(value,self.scene_stamp)
        except (ValueError,TypeError):pass
    def problem(self,now):
        for i in self.ids:
            view=SimpleNamespace(state=self.states.get(i,{}),scene=self.scene,scene_stamp=self.scene_stamp,scene_progress=self.scene_progress,model=self.model,id=i,active_ids=self.ids)
            problem=SingleCoordinator.problem(view,now)
            if problem:return 'drone%d: %s'%(i,problem)
        return None
    def send(self,i,task):
        if not hasattr(self,'last_task_sent_stamp'):self.last_task_sent_stamp={}
        self.last_tasks[i]=task;self.last_task_sent_stamp[i]=rospy.Time.now().to_sec();self.tasks[i].publish(String(json.dumps(task)))
    def command(self,msg):
        with self.lock:
            try:
                op=json.loads(msg.data)['op'];now=rospy.Time.now().to_sec()
                if op in ('hold','cancel'):
                    self.fault='User requested '+op
                    for i,f in self.flights.items():f.stage='HOLD';self.tasks[i].publish(String(json.dumps(dict(op='hold'))))
                    return
                if op=='pause':
                    if self.fault:raise ValueError(self.fault)
                    moving=[f for f in self.flights.values() if f.stage!='LANDED']
                    if any(f.stage not in ('TAKEOFF','ASSEMBLED','CLIMB','CRUISE','DESCENT','TERMINAL') for f in moving):raise ValueError('Pair phase cannot pause')
                    for i,f in self.flights.items():
                        if f.stage!='LANDED':f.pause(now);self.tasks[i].publish(String(json.dumps(dict(op='hold'))))
                    return
                if op=='resume':
                    problem=self.problem(now)
                    if problem or self.fault:raise ValueError(problem or self.fault)
                    if any(f.stage not in ('PAUSED','LANDED') or np.linalg.norm(self.states[i]['velocity'])>=.2 for i,f in self.flights.items()):raise ValueError('Both members must be stopped before resume')
                    for i,f in self.flights.items():
                        if f.stage=='LANDED':continue
                        f.resume(now);task=self.last_tasks.get(i)
                        if f.stage=='TAKEOFF' and self.states[i].get('armed'):task=dict(op='goto',height_mode='AGL',height=2.,x=float(f.spawn[0]),y=float(f.spawn[1]),speed=.5,_stage='TAKEOFF_RESUME')
                        if f.stage!='ASSEMBLED' and task:self.send(i,task)
                    return
                if op not in ('start','assemble'):raise ValueError('Use start, assemble, pause, resume, hold, cancel')
                problem=self.problem(now)
                if problem:raise ValueError(problem)
                if self.fault:raise ValueError(self.fault)
                for i,f in self.flights.items():
                    s=self.states[i]
                    if f.stage!='IDLE' or s.get('armed') or not s.get('contact') or np.linalg.norm(np.asarray(s['position'])-f.spawn)>.5:raise ValueError('Both members must be idle at measured ground slots')
                self.auto_start=op=='start'
                for i,f in self.flights.items():self.send(i,f.start(now,auto=False))
            except (ValueError,KeyError,TypeError) as exc:self.reason='Command rejected: '+str(exc)
    def advance_flight(self,i,now,decision):
        f=self.flights[i];other=7 if i==2 else 2
        if not hasattr(self,'phase_events'):self.phase_events=[]
        if f.stage=='CLIMB_READY':
            if now-f.started>120.:raise ValueError('Common climb arrival wait timeout')
            state=self.states[i];p=np.asarray(state['position']);v=np.asarray(state['velocity'])
            if state.get('phase')!='HOLD' or np.linalg.norm(p[:2]-f.climb_target[:2])>=.5 or abs(p[2]-270.)>=.3 or np.linalg.norm(v)>=.2:
                raise ValueError('Common climb arrival hold lost')
            return None
        if f.stage=='CRUISE_WAIT':
            elapsed=now-f.started
            if elapsed>120.:raise ValueError('Cruise wait timeout: drone%d'%i)
            state=self.states[i];p=np.asarray(state['position'],float);v=np.asarray(state['velocity'],float)
            if (p.shape!=(3,) or v.shape!=(3,) or not np.isfinite(np.r_[p,v]).all()
                    or state.get('phase')!='HOLD' or not state.get('armed') or state.get('contact')
                    or abs(p[2]-270.)>=.3 or np.linalg.norm(v)>=.2):
                raise ValueError('Cruise wait hover lost: drone%d'%i)
            if not decision.get('cruise_admission_ready',False):return None
            self.phase_events.append(dict(drone_id=i,stage='CRUISE_WAIT_COMPLETE',stamp=now,elapsed=elapsed,
                                          distance=decision.get('distance'),required_distance=decision.get('cruise_admission_guard')))
            f.transition('CRUISE',now);return f.goto(f.route[0],10.5)
        before=f.stage;started=f.started
        budget=getattr(self,'climb_budgets',{}).get(i)
        state=self.states[i]
        if getattr(self,'parallel_climb_geometry',None) and f.stage=='CLIMB' and np.linalg.norm(np.asarray(state['position'])[:2]-f.climb_target[:2])>=.5:
            state=dict(state,phase='GOTO') # Actual XY arrival is mandatory before stable completion.
        task=f.step(state,now,climb_elapsed=budget.active_seconds,climb_limit=budget.active_limit) if f.stage=='CLIMB' and budget is not None else f.step(state,now)
        if getattr(self,'parallel_climb_geometry',None) and before=='CLIMB' and f.stage=='CRUISE':
            f.transition('CLIMB_READY',now)
            self.phase_events.append(dict(drone_id=i,stage='CLIMB_READY',stamp=now,reason='Actual climb target reached; waiting for both arrivals'))
            return None
        if (before=='CLIMB' and f.stage=='CRUISE' and self.flights[other].stage=='CRUISE'
                and not decision.get('cruise_admission_ready',False)):
            f.transition('CRUISE_WAIT',now)
            self.phase_events.append(dict(drone_id=i,stage='CRUISE_WAIT',stamp=now,climb_seconds=now-started,
                                          budget_seconds=120.,reason='Awaiting measured separation and stopping reserve',
                                          distance=decision.get('distance'),required_distance=decision.get('cruise_admission_guard')))
            return None
        return task
    def tick(self,_):
        with self.lock:
            now=rospy.Time.now().to_sec();running=any(f.stage not in ('IDLE','HOLD','FAILED','LANDED') for f in self.flights.values())
            try:
                problem=self.problem(now)
                goals={}
                for i,f in self.flights.items():
                    measured=self.odometry.get(i,{}).get('p',f.spawn.tolist())
                    goals[i]=f.climb_target if f.stage in ('CLIMB','CLIMB_READY') else f.route[min(f.index,len(f.route)-1)] if f.stage=='CRUISE' else f.handoff if f.stage=='DESCENT' else f.end if f.stage=='TERMINAL' else measured
                proof_states={i:dict(s,control_reference=getattr(self,'control_proofs',{}).get(i,{})) for i,s in self.states.items()}
                decision=self.safety.decide(self.odometry,goals,now,{i:f.stage for i,f in self.flights.items()},proof_states,getattr(self,'parallel_climb_geometry',None))
                if running and (problem or decision.get('hard_fault')):self.fault=self.fault or problem or decision['hard_fault']
            except Exception as exc:
                problem='Invalid pair observation: '+repr(exc)
                if running:self.fault=self.fault or problem
                self.safety.policy.serial+=1;decision=dict(seq=self.safety.policy.serial,stamp=now,allow={i:False for i in self.ids},speed_cap_3d={i:10.5 for i in self.ids},cap_generation={i:0 for i in self.ids},hard_fault=self.fault,reason=problem)
            if problem or self.fault:decision.update(allow={i:False for i in self.ids},hard_fault=self.fault,reason=self.fault or problem)
            if any(f.stage=='PAUSED' for f in self.flights.values()):decision.update(allow={i:False for i in self.ids},reason='User pair pause')
            self.permit.publish(String(json.dumps(decision)))
            if running and not self.fault:
                try:
                    if not hasattr(self,'climb_budgets'):self.climb_budgets={}
                    for i,f in self.flights.items():
                        if f.stage=='CLIMB' or (f.stage=='PAUSED' and getattr(f,'paused_stage',None)=='CLIMB'):
                            if i not in self.climb_budgets:self.climb_budgets[i]=CoordinatedClimbBudget(i,f.started,getattr(self,'parallel_climb_geometry',{}).get('climb_active_limit',180.))
                            self.climb_budgets[i].observe(now,decision,healthy=not problem and not self.fault)
                    distance=decision.get('distance',0.)
                    for i,f in self.flights.items():
                        task=self.advance_flight(i,now,decision)
                        if task:self.send(i,task)
                    if all(f.stage=='CLIMB_READY' for f in self.flights.values()) and decision.get('cruise_admission_ready',False) and all(decision['allow'].get(i,False) for i in self.ids):
                        for i,f in self.flights.items():
                            f.transition('CRUISE',now);self.send(i,f.goto(f.route[0],10.5))
                        self.phase_events.append(dict(stage='COMMON_CRUISE_RELEASE',stamp=now,distance=decision.get('distance')))
                    if getattr(self,'auto_start',False) and all(f.stage=='ASSEMBLED' for f in self.flights.values()):
                        for i,f in self.flights.items():
                            f.transition('CLIMB',now);task=f.goto(f.climb_target,.8);task['_forward_climb']=True;self.send(i,task)
                        self.phase_events.append(dict(stage='COMMON_CLIMB_RELEASE',stamp=now,targets={i:f.climb_target.tolist() for i,f in self.flights.items()}))
                except (ValueError,KeyError,TypeError) as exc:self.fault=str(exc)
            if self.fault:
                for f in self.flights.values():
                    if f.stage not in ('LANDED','HOLD'):f.stage='FAILED'
            stages={i:f.stage for i,f in self.flights.items()};passed=all(s=='LANDED' for s in stages.values()) and not self.fault and not problem
            result=dict(control_proofs=getattr(self,'control_proofs',{}),last_tasks=self.last_tasks,last_task_sent_stamp=getattr(self,'last_task_sent_stamp',{}),climb_geometry=getattr(self,'parallel_climb_geometry',None),stages=stages,active_ids=list(self.ids),scene_count=10,passed=passed,reason=self.fault or problem or self.reason,decision=decision,stamp=now,states=self.states,odometry=self.odometry,climb_timing={i:b.status() for i,b in getattr(self,'climb_budgets',{}).items()},phase_events=getattr(self,'phase_events',[]),cruise_wait={i:dict(elapsed=now-f.started,budget_seconds=120.,reason='Awaiting measured separation and stopping reserve') for i,f in self.flights.items() if f.stage=='CRUISE_WAIT'})
            self.status.publish(String(json.dumps(result)))
            terminal=tuple(stages.values())
            if all(s in ('FAILED','LANDED','HOLD') for s in terminal) and terminal!=self.saved:
                temporary=self.output/'pair-result.json.tmp';temporary.write_text(json.dumps(result,indent=2));temporary.replace(self.output/'pair-result.json');self.saved=terminal
    def shutdown(self):
        for i,f in self.flights.items():
            if f.stage not in ('IDLE','LANDED'):self.tasks[i].publish(String(json.dumps(dict(op='hold'))))
if __name__=='__main__':PairCoordinator();rospy.spin()
