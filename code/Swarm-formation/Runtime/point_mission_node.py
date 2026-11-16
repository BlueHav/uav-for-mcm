#!/usr/bin/env python3
"""Point-configured ten-aircraft coordinator using existing per-aircraft YOPO flight control."""
import json
import os
import time
import uuid
from pathlib import Path
from portable_paths import PROJECT
from active_layout import ActiveLayout
import numpy as np
import rospy
from std_msgs.msg import String
from ten_node import Coordinator as LegacyCoordinator
from formation_core import min_separation
from point_mission_core import PointMission
from point_mission_engine import GroupFlight,leader_sample_time
from planning_mode import ModeGate
from flight_status import FlightClock,formation_modes
from sensor_inbox import LatestMeasurement

class Coordinator(LegacyCoordinator):
    def shutdown(self):
        with self.lock:
            self.shutting_down=True
            super().shutdown()
            self.log=None

    def tick(self,_):
        # RViz histories grow throughout a flight. Serialize them without
        # holding the lock needed by raw-ground and aircraft-state callbacks.
        visual=None
        with self.lock:
            if getattr(self,'shutting_down',False):return
            if self.fast_terminal_checks:
                self.terminal_tick_count+=1
                if self.planning_gate.mode!='YOPO' and self.terminal_tick_count%2:return
            try:
                self.pending_visual=None;self.step();visual=self.pending_visual
            except Exception as exc:rospy.logerr('Point coordinator %r',exc);self.fail(repr(exc))
        if visual is not None and self.terminal_tick_count%2==0:self.visual_inbox.put(visual)

    def publish_visual_snapshot(self,_):
        visual=self.visual_inbox.take()
        if visual is not None:self.visualize(*visual)

    def final_evidence_problem(self):
        r=self.report
        if r.get('errors'):return 'Final acceptance contains prior errors'
        if self.scene.get('collision_events')!=0 or self.scene.get('bounds_events')!=0:return 'Final Unity collision/bounds evidence'
        group_limit=len(self.active_groups)>1
        if r['min_body_distance']<1.0 or (group_limit and r['max_group_distance']>2000.) or r['max_altitude']>300.:return 'Final historical distance/height limit violated'
        if getattr(self.model,'high_speed_cloud',False) and not r.get('cruise_evidence',{}).get('passed',False):return 'Measured straight cruise below required minimum for one or more members'
        return None

    def init_terrain(self):
        self.model=PointMission(rospy.get_param('~point',str(PROJECT / 'config/point.json')))
        requested=rospy.get_param('~mode','')
        if requested:self.model.layout=ActiveLayout(requested)
        self.model.layout.require_runtime()
        self.active=tuple(self.model.layout.active_ids);self.active_groups=tuple(self.model.layout.group_ids)
        self.cloud_mission=os.environ.get('YOPO_CLOUD_MISSION')=='1';self.model.cloud_only=self.cloud_mission
        self.model.independent_terminal=self.cloud_mission and os.environ.get('YOPO_INDEPENDENT_TERMINAL')=='true'
        self.fast_terminal_checks=self.cloud_mission
        self.terminal_tick_count=0
        self.visual_inbox=LatestMeasurement()
        self.visual_timer=rospy.Timer(rospy.Duration(.1),self.publish_visual_snapshot)
        if self.cloud_mission:
            from cloud_config import configure_model
            configure_model(self.model)
            from cruise_evidence import CruiseEvidence
            self.cruise_evidence=CruiseEvidence()
        self.planning_gate=ModeGate('pending');self.handoff_dwell=None;self.handoff_started=None
        self.flight_clock=FlightClock()
        self.mode_pub=rospy.Publisher('/swarm10/planning_mode',String,queue_size=1,latch=True)
        self.groups=[GroupFlight(self.model,g) for g in range(2)]
        self.relay_followers=set()
        if getattr(self.model,'relay',False):
            from relay_core import followers as relay_followers
            for g in self.active_groups:self.relay_followers.update(relay_followers(g))
        self.terminal_coordinator=None
        self.execution_commit_pubs=[]
        if self.cloud_mission and not self.model.independent_terminal:
            from terminal_continuous import ContinuousCoordinator
            self.terminal_coordinator=ContinuousCoordinator(self.model,clock=self.clock)
            self.execution_commit_pubs=[rospy.Publisher('/swarm10/group_%d/execution_commit'%g,String,queue_size=1,tcp_nodelay=True) for g in (0,1)]
            rospy.on_shutdown(self.terminal_coordinator.close)
        self.terminal_enabled=True;self.terminal_goal=self.model.ends[0].copy();self.low_targets=None;self.landing_token=None
        self.preset_descent=self.model.arrival.copy();self.low_speed=self.model.speed
        self.saved_at=0.;self.guidance_pubs=[rospy.Publisher('/swarm10/group_%d/flight_guidance'%g,String,queue_size=1) for g in range(2)]
    def send(self,op,**kw):
        # Only explicitly active aircraft receive mission commands; the other
        # bodies stay parked in the same fixed ten-body Unity scene.
        message=String(json.dumps(dict(op=op,_formation_internal=True,**kw)))
        for i,pub in enumerate(self.tasks):
            if i in self.active:pub.publish(message)


    def ready(self,now):
        from mission_geometry_gate import terminal_distance_problem
        geometry_problem=terminal_distance_problem(self.model)
        if geometry_problem:return geometry_problem
        problem=super().ready(now)
        if problem:return problem
        if self.scene.get('map_sha256')!=self.model.hashes['world']:return 'Unity map fingerprint differs from point.json'
        if getattr(self.model,'high_speed_cloud',False) and self.planning_gate.mode!='YOPO':
            for i,s in self.states.items():
                # Relay followers deliberately do not plan; their map context is
                # not refreshed and must not veto the leader's flight.
                if i in getattr(self,'relay_followers',()):continue
                context=s.get('map',{})
                if context.get('context_hash')!=self.model.hashes['world'] or not -.1<=s['stamp']-(context.get('context_stamp') or 0)<=.6:return 'Member %d high-speed terrain context stale/mismatched'%i
        return None

    def square_settled(self,p,v):
        for g in self.active_groups:
            engine=self.groups[g]
            members=[i for i in self.active if i//5==g]
            slots=[i%5 for i in members]
            if engine.u<1. or not np.all(np.linalg.norm(p[members,:2]-engine.destination[slots,:2],axis=1)<.5):
                return False
            for i in members:
                s=self.states[i]
                if (not s['armed'] or s['contact'] or not abs(s.get('agl',np.inf)-self.model.assembly_agl)<.25
                        or not np.linalg.norm(v[i])<.2):
                    return False
        return True

    def command(self,msg):
        with self.lock:
            if getattr(self,'shutting_down',False):return
            try:
                data=json.loads(msg.data);op=data['op']
                if op in ('hold','cancel'):self.end_hold();return
                if op=='relay_fault':
                    if self.stage=='RUNNING' and data.get('run_id')==self.run_id:
                        self.fail('Relay: '+str(data.get('reason','unknown fault')))
                    return
                self.drain_ground();now=rospy.Time.now().to_sec()
                problem=self.ready(now)
                now=getattr(self,'ready_stamp',now)
                if problem:raise ValueError(problem)
                if op=='assemble' and self.stage=='IDLE':
                    if PointMission(self.model.path).hashes!=self.model.hashes:raise ValueError('Input changed; restart mission')
                    p=self.positions()
                    if np.max(np.linalg.norm(p-self.model.spawn(),axis=1))>.5:raise ValueError('Simulator spawn differs from configured ten slots')
                    if any(s['armed'] or not s['contact'] for s in self.states.values()):raise ValueError('All ten must be grounded/disarmed')
                    self.altitude=self.model.altitude;self.landing_center=self.model.arrival.copy()
                    self.run_id=time.strftime('%Y%m%dT%H%M%S')+'-'+uuid.uuid4().hex[:6]
                    if self.cloud_mission:self.planning_gate=ModeGate(self.run_id)
                    scene=Path(os.environ['YOPO_SCENE_ROOT'])
                    self.directory=scene/'Reports/ten-runs'/self.run_id;self.directory.mkdir(parents=True)
                    from ten_core import register_failure_directory
                    register_failure_directory(scene,self.run_id,self.directory)
                    self.report=dict(run_id=self.run_id,workflow='point-v1',hashes=self.model.hashes,stages=[],errors=[],passed=False,
                                     inference_devices=sorted({s.get('inference_device','unknown') for s in self.states.values()}),
                                     active_ids=list(self.active),active_mode=self.model.layout.mode,
                                     group_events=[[],[]],min_body_distance=1e9,max_group_distance=0.,max_altitude=0.,nominal_speed=getattr(self.model,'cruise_speed',6.),terminal_speed=6.,trail_spacing=self.model.spacing,departure_delay=self.model.group_delay,
                                     state_age_max=0.,state_use_age_max=0.)
                    if getattr(self.model,'independent_terminal',False):
                        self.report['terminal_mode']='independent_yopo'
                        self.report['terminal_peak_horizontal_mps']={str(i):0. for i in self.active}
                    self.log=(self.directory/'telemetry.jsonl').open('w',buffering=1)
                    self.send('takeoff',height_mode='AGL',height=self.model.assembly_agl,speed=2.)
                    self.set_stage('TAKEOFF')
                elif op=='land' and self.stage=='STAR_HOVER':
                    self.start_preland()
                elif op=='climb' and self.stage=='ASSEMBLED':
                    p=self.positions()
                    if getattr(self.model,'forward_departure',False):
                        v=np.array([self.states[i]['velocity'] for i in range(10)])
                        if not self.square_settled(p,v):raise ValueError('Square hover is not settled')
                    if getattr(self.model,'relay',False):
                        for g in self.active_groups:
                            group=p[5*g:5*g+5]
                            if not self.model.sweep_clear(group,np.repeat(group[2:3],5,axis=0)):
                                raise ValueError('Departure slot corridors lack terrain clearance')
                        self.relay_start=now
                    for g in self.active_groups:self.groups[g].release(p[5*g:5*g+5],now)
                    self.set_stage('RUNNING')
                else:raise ValueError('Command incompatible with stage '+self.stage)
            except (ValueError,KeyError,TypeError) as exc:self.reason='Command rejected: '+str(exc)

    def handoff_tick(self,now):
        if not getattr(self,'cloud_mission',False) or not self.run_id:return
        gate=self.planning_gate
        assembled=all(self.groups[g].phase=='DESCENT_ASSEMBLED' for g in self.active_groups)
        stable=assembled and all(abs(self.states[i].get('agl',999)-5.)<=.25 and np.linalg.norm(self.states[i]['velocity'])<.2 for i in self.active) and len(self.states)==10
        if stable:
            expected=np.concatenate([self.model.descent(g)[1] for g in self.active_groups])
            stable=bool(np.max(np.linalg.norm(self.positions()[:,:2]-expected[:,:2],axis=1))<.5)
        if gate.mode=='CLOUD':
            if stable:
                if self.handoff_dwell is None:self.handoff_dwell=now
                if now-self.handoff_dwell>=self.model.stable_seconds:
                    gate.begin_handoff([True,True]);self.handoff_started=now
                    self.report.setdefault('planning_events',[]).append(dict(stamp=now,**gate.reference()))
            else:self.handoff_dwell=None
        elif gate.mode=='YOPO_WARMUP':
            if not stable:self.fail('Descent assembly lost during YOPO warmup')
            elif any(self.states[i].get('handoff_error') for i in self.active):self.fail('YOPO warmup failed')
            elif all(self.states[i].get('handoff_ready') and self.states[i].get('planner_epoch')==1 and self.states[i].get('planning_run_id')==self.run_id for i in self.active):
                gate.release([True]*10);self.report.setdefault('planning_events',[]).append(dict(stamp=now,**gate.reference()))
            elif now-self.handoff_started>120:self.fail('YOPO warmup timeout')
        elif gate.mode=='YOPO' and assembled:
            if all(self.states[i].get('planner_mode')=='YOPO' and self.states[i].get('planner_epoch')==2 and self.states[i].get('planning_run_id')==self.run_id for i in self.active):
                for g in self.active_groups:self.groups[g].begin_terminal(now)
            elif now-self.handoff_started>120:self.fail('YOPO handoff acknowledgment timeout')
        if self.stage not in ('FAILED','HOLD','LANDED'):self.mode_pub.publish(String(json.dumps(dict(stamp=now,**gate.reference()))))

    def star_slots_settled(self,p,v):
        return all(self.model.star_settled(g,p[5*g:5*g+5],v[5*g:5*g+5]) for g in self.active_groups)

    def finish_terminal_approach(self,p,v):
        if self.stage!='RUNNING' or any(self.groups[g].phase!='FINAL' for g in self.active_groups):
            return
        if getattr(self.model,'independent_terminal',False):
            if self.star_slots_settled(p,v) and all(self.states[i]['phase']=='END_HOLD' for i in self.active):
                self.start_preland()
            return
        if all(abs(self.states[i]['agl']-self.model.end_agl)<.25
               and self.states[i].get('terminal_joint',{}).get('done')
               and self.states[i].get('execution',{}).get('tail_settled') for i in self.active):
            self.set_stage('STAR_HOVER')

    def step(self):
        self.drain_ground()
        now=rospy.Time.now().to_sec();mono=time.monotonic();dt=min(.25,max(0.,mono-self.last));self.last=mono
        terminal=self.stage in ('IDLE','HOLD','FAILED','LANDED')
        problem=self.ready(now)
        now=getattr(self,'ready_stamp',now)
        if not terminal and problem:self.fail(problem)
        p=self.positions() if len(self.states)==10 else None
        v=np.array([self.states[i]['velocity'] for i in range(10)]) if p is not None else None
        gap=float(np.linalg.norm(p[:5].mean(0)-p[5:].mean(0))) if p is not None else None
        spacing=min_separation(p) if p is not None else None
        # Independent 50 Hz state sample (Unity mirror of the same odometry).
        # It never replaces the mission snapshots used for control, but it makes
        # the body/group/height limits independent of the 10 Hz status cadence.
        live=self.live_state(now)
        # A mirror whose odometry source is older than the control period can
        # only contribute collision flags, never a separation/height decision.
        live_usable=live is not None and not live['odom_stale']
        live_p=np.asarray(live['positions'],float) if live_usable else None
        live_v=np.asarray(live['velocities'],float) if live_usable else None
        live_spacing=min_separation(live_p) if live_p is not None else None
        live_gap=float(np.linalg.norm(live_p[:5].mean(0)-live_p[5:].mean(0))) if live_p is not None else None
        # Include the current sample before any final success decision.
        if self.report is not None and p is not None:
            if live is not None:
                self.report['state_age_max']=max(self.report.get('state_age_max',0.),live['age'])
                self.report['state_use_age_max']=max(self.report.get('state_use_age_max',0.),now-live['received'])
            measured_spacing=spacing if live_spacing is None else min(spacing,live_spacing)
            measured_gap=gap if live_gap is None else max(gap,live_gap)
            measured_alt=float(p[:,2].max()) if live_p is None else max(float(p[:,2].max()),float(live_p[:,2].max()))
            self.report['min_body_distance']=min(self.report['min_body_distance'],measured_spacing)
            self.report['max_group_distance']=max(self.report['max_group_distance'],measured_gap)
            self.report['max_altitude']=max(self.report['max_altitude'],measured_alt)
            if 'terminal_peak_horizontal_mps' in self.report and self.stage=='RUNNING':
                peaks=self.report['terminal_peak_horizontal_mps']
                for i in self.active:
                    if self.groups[i//5].phase=='TERRAIN' and self.states[i]['phase']=='TERRAIN_FOLLOW':
                        peaks[str(i)]=max(peaks[str(i)],float(np.linalg.norm(v[i,:2])))
            if hasattr(self,'cruise_evidence'):
                for i in self.active:
                    s=self.states[i]
                    stamp=s.get('kinematic_stamp',s['stamp']-s.get('odom_age',0.))
                    cruising=(self.stage=='RUNNING' and self.groups[i//5].phase=='CRUISE'
                              and abs(s['position'][2]-self.model.altitude)<=.5
                              and np.linalg.norm(s['velocity'][:2])>=self.cruise_evidence.minimum)
                    self.cruise_evidence.update(i,stamp,s['velocity'],cruising)
                self.report['cruise_evidence']=self.cruise_evidence.report(self.active)
        if self.stage in ('LANDING','LANDED'):
            final_problem=self.final_evidence_problem()
            if final_problem:self.fail(final_problem)
        if p is not None and self.stage not in ('IDLE','HOLD','FAILED','LANDED'):
            limit=1.9 if self.stage in ('TAKEOFF','AIR_ASSEMBLE','ASSEMBLED') else 1.0
            if spacing<limit or (live_spacing is not None and live_spacing<limit):
                self.fail('Actual separation below %.1fm'%limit)
            if not self.model.group_safe(p,np.zeros_like(v),active_groups=self.active_groups):self.fail('Actual group centers exceed 2km')
            if live_p is not None and len(self.active_groups)>1 and not self.model.group_safe(live_p,np.zeros_like(live_v),active_groups=self.active_groups):self.fail('Actual group centers exceed 2km')
            if p[:,2].max()>300. or (live_p is not None and live_p[:,2].max()>300.):self.fail('Actual altitude exceeds 300m')
        self.preland_tick(mono)
        if self.stage not in ('FAILED','HOLD','LANDED'):self.handoff_tick(now)
        if self.stage=='TAKEOFF':
            settled=all(self.states[i]['armed'] and not self.states[i]['contact'] and abs(self.states[i].get('agl',0)-self.model.assembly_agl)<.25 and np.linalg.norm(self.states[i]['velocity'])<.2 for i in self.active)
            settled=settled and np.max(np.linalg.norm(p[:,:2]-self.model.spawn()[:,:2],axis=1))<.5
            if settled:
                if self.dwell is None:self.dwell=mono
                if mono-self.dwell>=self.model.stable_seconds:
                    if getattr(self.model,'forward_departure',False):
                        for g in self.active_groups:self.groups[g].segment('AIR_ASSEMBLE',self.model.assembly(g),p[5*g:5*g+5],now)
                        self.set_stage('AIR_ASSEMBLE')
                    else:self.set_stage('ASSEMBLED')
            else:self.dwell=None
            if mono-self.started>90:self.fail('Takeoff assembly timeout')
        if self.stage in ('RUNNING','AIR_ASSEMBLE','ASSEMBLED','STAR_HOVER'):
            if any(not self.states[i]['armed'] or self.states[i]['contact'] for i in self.active):self.fail('Unexpected contact/disarm in flight')
            else:
                refs=[]
                group_safe=self.model.group_safe(p,v,active_groups=self.active_groups)
                terminal=getattr(self,'terminal_coordinator',None)
                if group_safe and terminal is not None and self.planning_gate.mode=='YOPO':
                    # Renew currently owned packets before either group's
                    # planning work. New offers still use the usual commit gate.
                    for g in self.active_groups:
                        if self.groups[g].phase not in ('TERRAIN','FINAL'):continue
                        slots=[None if slot is None or not slot['committed'] else
                               dict(kind='commit',packet=slot['packet'],start=slot['start'],
                                    predecessor=slot['predecessor'])
                               for slot in (terminal.active[g],terminal.offers[g])]
                        command=dict(kind='stream',active=slots[0],next=slots[1])
                        self.publish_execution_commit(g,dict(terminal_joint=command),now)
                for g in self.active_groups:
                    engine=self.groups[g];sl=slice(g*5,g*5+5)
                    engine.distance_sample_stamps=[leader_sample_time(self.states[i],now) for i in range(g*5,g*5+5)]
                    engine.distance_observations_fresh=all(-.1<=now-stamp<=.3 for stamp in engine.distance_sample_stamps)
                    engine.leader_sample_stamp=engine.distance_sample_stamps[2]
                    engine.merge_entry_acceleration=np.array([self.states[i].get('control_reference',{}).get('acceleration',[0.,0.,0.]) for i in range(g*5,g*5+5)])
                    blocked=any(self.states[i].get('formation',{}).get('blocked',False) for i in range(g*5,g*5+5))
                    ahead=False
                    # Moving merge must finish its certified moving segment;
                    # retain the actual/predicted 2 km checks below and above.
                    if gap>1800 and len(self.active_groups)>1 and engine.phase!='MOVING_MERGE':
                        other=p[5*(1-g):5*(1-g)+5].mean(0);center=p[sl].mean(0)
                        ahead=np.dot(v[sl].mean(0),center-other)>0
                    terminal=getattr(self,'terminal_coordinator',None)
                    engine.terminal_waiting=bool(terminal is not None and hasattr(terminal,'waiting') and terminal.waiting(g,self.states,now))
                    ref=engine.step(p[sl],v[sl],now,dt,blocked,hold_leader=ahead)
                    if ref is not None:
                        if getattr(self,'terminal_coordinator',None) is not None and self.planning_gate.mode=='YOPO' and engine.phase in ('TERRAIN','FINAL'):
                            if engine.phase=='FINAL':ref['terminal_hover']=True
                            ref=self.terminal_coordinator.update(g,self.states,ref,now,self.run_id)
                            self.report['terminal_swarm']=self.terminal_coordinator.metrics
                            engine.note='YOPO candidates; jointly certified Swarm-Formation trajectories'
                        ref['run_id']=self.run_id
                        if getattr(self,'cloud_mission',False):ref.update(self.planning_gate.reference())
                        if not group_safe:ref['safety_pause']=True
                        self.publish_execution_commit(g,ref,now)
                        refs.append((g,ref))
                # The large polynomial/corridor messages can delay another
                # group's compact authority heartbeat under CPU contention.
                for g,ref in refs:
                    self.refs[g].publish(String(json.dumps(ref)))
                for g in self.active_groups:
                    engine=self.groups[g]
                    hint=dict(engine.hint,stamp=now,leader_id=2+5*g,phase=engine.phase,controller_connected=True)
                    self.guidance_pubs[g].publish(String(json.dumps(hint)))
                if getattr(self.model,'synchronized_climb',False):
                    blocked=any(s.get('formation',{}).get('blocked',False) for s in self.states.values())
                    if len(self.active_groups)==1:
                        # The barrier synchronises two groups. With one active
                        # group the parked bodies can never match its phase, so
                        # release that group from its own measured readiness.
                        gi=self.active_groups[0];g=self.groups[gi];sl=slice(5*gi,5*gi+5)
                        released=None
                        if getattr(self.model,'relay',False):
                            leader_ready=(np.linalg.norm(v[sl][2])<.2 and np.linalg.norm(p[sl][2]-g.reference[2])<.5)
                        else:
                            leader_ready=np.max(np.linalg.norm(v[sl],axis=1))<.2
                        if not blocked and g.phase in ('TRANSFER_READY','CLIMB_READY','CRUISE_READY') and leader_ready:
                            if g.phase=='CRUISE_READY':g.start_cruise(p[sl],now)
                            else:g.next_segment(p[sl],now)
                            released=g.phase
                    else:
                        from departure_sequence import release_groups, release_relay_groups
                        if getattr(self.model,'relay',False):
                            released=release_relay_groups([self.groups[g] for g in self.active_groups],p,v,now,blocked)
                        else:
                            released=release_groups([self.groups[g] for g in self.active_groups],p,v,now,blocked)
                    if released:self.report.setdefault('synchronized_releases',[]).append(dict(stamp=now,barrier=released,member_interval=self.model.member_departure_interval))
                if self.stage=='AIR_ASSEMBLE':
                    if self.square_settled(p,v):
                        if self.dwell is None:self.dwell=mono
                        if mono-self.dwell>=self.model.stable_seconds:
                            for g in self.active_groups:
                                self.groups[g].event('ASSEMBLED',now)
                                self.groups[g].note='waiting for second RViz command'
                            self.set_stage('ASSEMBLED')
                    else:self.dwell=None
                self.finish_terminal_approach(p,v)
                self.reason='; '.join('group%d:%s %s'%(g,e.phase,e.note) for g,e in enumerate(self.groups))
        if self.stage=='LANDING':
            self.retry_landing(now)
            if self.model.landed(self.states,self.active):
                if getattr(self,'landing_confirm',None) is None:self.landing_confirm=(mono,now,self.scene.get('frames',-1))
                since,stamp,frame=self.landing_confirm
                if mono-since>=2. and self.scene_stamp>=stamp+2. and self.scene.get('frames',-1)>frame:
                    self.set_stage('LANDED');self.report['passed']=True;self.report['final_states']=self.states;self.save()
            else:
                self.landing_confirm=None
                if all(self.states[i]['phase']=='LANDED' for i in self.active):self.fail('Touchdown outside fixed slot tolerance or not stopped')
            if self.stage=='LANDING' and mono-self.started>120:self.fail('Landing timeout')
        # Either source may be absent before the first complete mission
        # snapshot (IDLE) or before the first fresh live sample.
        combined_gap=gap if live_gap is None else (live_gap if gap is None else max(gap,live_gap))
        combined_spacing=spacing if live_spacing is None else (live_spacing if spacing is None else min(spacing,live_spacing))
        group_phases=['IDLE' if g not in self.active_groups else
                      self.stage if self.stage in ('IDLE','TAKEOFF') else e.phase
                      for g,e in enumerate(self.groups)]
        status=dict(stamp=now,workflow='point-v1',stage=self.stage,reason=self.reason,run_id=self.run_id,
                    group_phases=group_phases,center_distance=combined_gap,min_body_distance=combined_spacing,band_active=False,
                    state_age=None if live is None else dict(transport=live['age'],use=max(0.,now-live['received'])),live_collision=None if live is None else bool(live['collision']),
                    altitude_abs=self.model.altitude,center_ids=[2,7],stop_clearance=2.,hashes=self.model.hashes)
        if getattr(self,'cloud_mission',False):status.update(self.planning_gate.reference())
        if not hasattr(self,'flight_clock'):self.flight_clock=FlightClock()
        status.update(self.flight_clock.update(self.states,now,self.run_id))
        if self.stage=='RUNNING' and hasattr(self,'relay_start'):status['relay_start']=self.relay_start
        status['relay_cuts']=[getattr(e,'relay_cut',None) for e in self.groups]
        status['terminal_cohesion']=[e.cohesion for e in self.groups]
        status['group_modes']=formation_modes(self.stage,status['group_phases'],status.get('planner_mode'))
        self.status.publish(String(json.dumps(status)))
        if p is not None:self.pending_visual=(p.copy(),status.copy())
        if self.report is not None and p is not None:
            self.report['group_events']=[e.events for e in self.groups]
            self.report['terminal_cohesion']=[e.cohesion for e in self.groups]
            self.report['collision_events']=self.scene.get('collision_events');self.report['bounds_events']=self.scene.get('bounds_events')
            if self.log and (self.terminal_tick_count%2==0 or self.stage=='FAILED'):
                self.log.write(json.dumps(dict(status=status,states=self.states,scene=self.scene,targets=[None if e.reference is None else e.reference.tolist() for e in self.groups]))+'\n')
            if mono-self.saved_at>2:self.save();self.saved_at=mono
            if self.stage in ('FAILED','LANDED','HOLD') and mono-self.started>15 and self.log:self.log.close();self.log=None

    def publish_execution_commit(self,group,ref,now):
        pubs=getattr(self,'execution_commit_pubs',())
        command=ref.get('terminal_joint') or {}
        if not pubs or command.get('kind')!='stream' or ref.get('safety_pause') or ref.get('paused_members'):return
        slots=[s for s in (command.get('active'),command.get('next')) if s and s.get('kind')=='commit']
        if not slots:return
        clock=getattr(self,'clock',None)
        dispatch=now if clock is None else clock()
        if not -.05<=dispatch-now<=.15:return
        value=dict(kind='formation_commit',run_id=self.run_id,epoch=2,group=group,stamp=dispatch,
            commits=[dict(digest=s['packet']['digest'],start=s['start'],predecessor=s.get('predecessor')) for s in slots])
        pubs[group].publish(String(json.dumps(value,separators=(',',':'),allow_nan=False)))

if __name__=='__main__':Coordinator();rospy.spin()
