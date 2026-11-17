"""Per-group mission references. Physical commands remain in the existing Mission node."""
import numpy as np
from formation_core import transition, min_separation, graph_gradient
from ten_core import segment_clearance
from leader_following import LeaderTrail, group_order
from terrain_guidance import terrain_direction
from point_mission_core import STAR


def leader_sample_time(state,now):
    """Recover the timestamp paired with status.position, never receipt time.

    Mission snapshots position and odometry time under one lock, then reports
    odom_age = status.stamp - odometry_time. Both fields are mandatory here.
    Retain the coordinator's existing 100 ms clock tolerance, 1.2 s status
    window and 500 ms odometry-age gate; the trail checks total sample age.
    """
    if 'kinematic_stamp' in state:
        # Set only by the supervisor after direct executor/certificate checks.
        # The planner report retains its own timestamp and sensor ages.
        sample=state['kinematic_stamp']
        if isinstance(sample,bool) or not isinstance(sample,(float,int)) or not np.isfinite(sample) or not -.05<=now-sample<=.5:
            raise ValueError('Expired/future measured leader observation')
        return sample
    try:
        if isinstance(state['stamp'],bool) or isinstance(state['odom_age'],bool):raise ValueError()
        published=float(state['stamp']);age=float(state['odom_age']);current=float(now)
    except (KeyError,TypeError,ValueError,OverflowError):raise ValueError('Missing/invalid paired leader observation time')
    if not np.isfinite([published,age,current]).all() or not -.1<=current-published<=1.2 or not -.1<=age<=.5:
        raise ValueError('Expired/future leader status or odometry')
    sample=published-age
    if not -.1<=current-sample<=1.2:raise ValueError('Expired/future leader position sample')
    return sample


def relay_entry_stop_clear(positions, velocities, accelerations):
    from terminal_stopping import actual_stop_paths

    for member,(position,velocity,acceleration) in enumerate(zip(positions,velocities,accelerations)):
        peers=np.delete(positions,member,axis=0)
        for path in actual_stop_paths(position,velocity,acceleration,velocity):
            if np.any(np.linalg.norm(path[:,None]-peers[None,:],axis=2)<2.8):
                return False
    return True


class GroupFlight:
    def __init__(self,model,group):
        self.model=model;self.group=group;self.relay=bool(getattr(model,'relay',False));self.phase='ASSEMBLED';self.reference=None
        self.queue=[];self.events=[];self.guide=None;self.guide_stamp=-1e9;self.note='waiting for second RViz command'
        self.waypoint=0;self.last_progress=None;self.last_position=None;self.dwell=None;self.hint={}
        self.descent_braking=False
        self.distance_gate=None;self.formation_governor=None;self.cohesion=None

    def event(self,phase,now):
        self.phase=phase;self.events.append(dict(phase=phase,stamp=now));self.dwell=None

    def release(self,p,now):
        self.distance_gate=None;self.formation_governor=None;self.cohesion=None
        self.reference=p.copy();self.queue=self.model.departure(self.group,p)
        self.release_time=now+self.group*self.model.group_delay;self.event('WAIT',now)
        self.last_progress=now;self.last_position=p.copy()
        self.note='Released for departure'

    def begin_terminal(self,now):
        if self.phase!='DESCENT_ASSEMBLED':raise ValueError('Descent assembly required')
        self.event('TERRAIN',now);self.terminal_origin=self.model.assembly_anchor(self.group)
        self.terminal_agl_target=self.model.arrival_agl
        self.last_progress=now

    def segment(self,phase,destination,p,now):
        destination=np.asarray(destination,float)
        if segment_clearance(p,destination)<1.9 or not self.model.sweep_clear(p,destination):
            raise ValueError('Unsafe geometry for '+phase)
        self.origin=p.copy();self.destination=destination.copy();self.reference=p.copy();self.u=0.
        fast=phase in ('CLIMB','DESCEND','DESCENT_MERGE','AIR_ASSEMBLE');delta=destination-p
        horizontal=3. if phase=='AIR_ASSEMBLE' else 6. if phase in ('DESCEND','DESCENT_MERGE') else (getattr(self.model,'cruise_speed',20.) if (self.relay and phase=='CLIMB') else 4.) if fast else 1.
        self.duration=max(1.,float(np.linalg.norm(delta[:,:2],axis=1).max())/horizontal,float(np.abs(delta[:,2]).max())/(6. if fast else 1.))
        if phase=='CLIMB':self.member_u=np.zeros(5);self.climb_elapsed=0.
        self.event(phase,now)
        if phase=='AIR_ASSEMBLE':self.note='Tracking 2 m AGL square slots'

    def next_segment(self,p,now):
        phase,end=self.queue.pop(0);self.segment(phase,end,p,now)

    def relay_climb(self,p,v,now,blocked,hold_leader):
        if not hasattr(self,'climb_course'):
            direction=self.model.ends[self.group]-p[2,:2]
            self.climb_axis=direction/np.linalg.norm(direction)
            self.climb_origin=p[2,:2].copy()
            self.climb_course=self.climb_origin+2000.*self.climb_axis
        if abs(p[2,2]-self.model.altitude)<.3 and abs(v[2,2])<.25:
            self.start_cruise(p,now)
            return self.cruise(p,v,now,blocked,hold_leader)
        self.note='Continuous forward climb toward group endpoint'
        self.reference=p.copy()
        self.reference[2]=np.r_[self.climb_course,self.model.altitude]
        result=self.output(now,pause=blocked or hold_leader)
        result['continuous_climb']=True
        return result

    def start_cruise(self,p,now):
        order=[i%5 for i in group_order(self.group)]
        history=p[order[::-1]]
        self.trail=LeaderTrail(spacing=self.model.spacing)
        sample=self.leader_stamp(now)
        if getattr(self.model,'forward_departure',False):
            self.trail.append(p[2],sample);self.wait_anchors=p.copy();self.waiting={0,1,3,4};self.joining=set()
            self.departure_direction=self.model.routes[self.group][1]-p[2,:2]
            self.departure_direction/=np.linalg.norm(self.departure_direction)
        else:
            if not self.model.sweep_clear(history[:-1],history[1:]):raise ValueError('Initial leader trail has no terrain clearance')
            self.trail.seed(history,sample)
        self.reference=p.copy();self.event('CRUISE',now);self.guide=None;self.guide_stamp=-1e9
        self.descent_braking=False
        start,_=self.model.descent(self.group)
        self.navigation=list(self.model.routes[self.group][1:-1])+[start[2,:2]]
        if hasattr(self,'climb_course'):
            # Finish the straight climb course before taking the next forward route waypoint.
            # Keep the two endpoint bearings through high-speed cruise.
            # The former turn toward opposite entry sides crossed the two lines.
            if self.relay and getattr(self.model,'moving_merge',False):
                self.navigation=[self.climb_origin+3800.*self.climb_axis]
            else:
                self.navigation=[self.climb_course]+[q for q in self.navigation
                                 if np.dot(q-self.climb_course,self.departure_direction)>0.]
        self.waypoint=0;self.last_progress=now;self.last_position=p.copy()

    def relay_descent(self,p,v,now,blocked,hold_leader):
        from cloud_config import STAR_ENTRY_SPEED
        from moving_merge import RelayStarMerge
        anchor=self.model.assembly_anchor(self.group)
        remaining=np.linalg.norm(anchor-p[2,:2])
        if not hasattr(self,'descent_turn'):
            altitude=232. if self.group==0 else 263.
            if (np.dot(p[2,:2]-self.climb_origin,self.climb_axis)>=4150.
                    and abs(p[2,2]-altitude)<5.):
                self.descent_turn=True
                self.descent_profile=dict(origin=p[2,:2].tolist(),anchor=anchor.tolist(),
                                          from_z=float(p[2,2]),to_z=float(self.model.ground(anchor[None,:])[0]+(55. if self.group==0 else 94.)),
                                          level_after=max(1.,remaining-850.))
            else:
                self.reference=p.copy()
                self.reference[2]=np.r_[self.climb_origin+5200.*self.climb_axis,altitude]
                self.note='Descending in separate altitude lanes before turning'
                result=self.output(now,stage='CRUISE',pause=blocked or hold_leader)
                result['speed']=self.model.cruise_speed
                result['relay_descent']=self.descent_profile
                return result
        cut=getattr(self,'relay_cut',None)
        if remaining<=1100. or cut is not None:
            acceleration=getattr(self,'merge_entry_acceleration',None)
            settled=(acceleration is not None and not blocked and not hold_leader and np.max(np.linalg.norm(v,axis=1))<=STAR_ENTRY_SPEED+.25
                     and relay_entry_stop_clear(p,v,acceleration))
            if cut is None:
                # One low-speed sample during braking is insufficient for all
                # five delayed commands to hand over at a moving boundary.
                if not settled:self.relay_entry_since=None
                elif getattr(self,'relay_entry_since',None) is None:self.relay_entry_since=now
                elif now-self.relay_entry_since>=1.:self.relay_cut=now+.7
            elif now>=cut and not settled:
                if acceleration is None:raise ValueError('Relay star takeover missing fresh controller command boundary')
                raise ValueError('Relay star takeover lost measured stopping clearance')
        if cut is not None and now>=cut:
            self.merge_plan=RelayStarMerge(self.model,self.group,p,v,getattr(self,'merge_entry_acceleration',None))
            self.merge_elapsed=0.;self.reference=p.copy();self.event('MOVING_MERGE',now)
            return self.moving_descent(p,v,now,0.,blocked,hold_leader)
        if remaining<750.:raise ValueError('Insufficient distance for safe star takeover')
        self.reference=p.copy()
        self.reference[2]=np.r_[anchor,self.descent_profile['to_z']]
        self.note='Early descent on delayed leader corridor'
        result=self.output(now,stage='CRUISE',pause=blocked or hold_leader)
        result['speed']=STAR_ENTRY_SPEED if remaining<1320. else 12. if remaining<1550. else self.model.cruise_speed
        result['relay_descent']=self.descent_profile
        return result

    def output(self,now,stage=None,velocity=None,pause=False,paused=None,agl=None):
        r=dict(workflow='point-v1',stage=stage or ('TERRAIN' if self.phase in ('TERRAIN','FINAL') else 'CRUISE' if self.phase=='CRUISE' else 'DESCEND' if self.phase in ('DESCEND','DESCENT_MERGE') else 'CLIMB' if self.phase=='CLIMB' else 'ASSEMBLE' if self.phase=='AIR_ASSEMBLE' else 'REFORM'),
               stamp=now,targets=self.reference.tolist(),velocity=(np.zeros((5,3)) if velocity is None else velocity).tolist(),
               speed=getattr(self.model,'cruise_speed',self.model.speed) if (self.phase=='CRUISE' or (self.relay and self.phase=='CLIMB')) else self.model.speed,leader_id=2+5*self.group,safety_pause=bool(pause),paused_members=paused or [])
        if self.relay and self.phase=='CLIMB':r['relay_climb']=True
        if agl is not None:r.update(agl=agl,height_mode='AGL')
        if self.phase=='MOVING_MERGE':
            from moving_merge import CATCHUP_SPEED
            r.update(stage='DESCEND',moving_merge=True,speed=CATCHUP_SPEED,
                     leader_speed=16.)
        return r

    def moving_descent(self,p,v,now,dt,blocked,hold_leader):
        # A paused polynomial cannot be resumed at its old moving boundary.
        # The existing coordinator failure path commands a controlled hold.
        if blocked or hold_leader:raise ValueError('Moving merge interrupted by safety condition; controlled hold required')
        if np.max(np.linalg.norm(p-self.reference,axis=1))>5.:
            raise ValueError('Moving merge tracking error exceeds 5 m')
        self.merge_elapsed=min(self.merge_plan.duration,self.merge_elapsed+dt)
        self.reference,velocity,acceleration=self.merge_plan.sample(self.merge_elapsed)
        self.note='Moving formation catch-up; final L braking only'
        arrived=(self.merge_elapsed>=self.merge_plan.duration and
                 np.max(np.linalg.norm(p-self.reference,axis=1))<.5 and
                 np.max(np.linalg.norm(v,axis=1))<.2)
        if arrived:
            if self.dwell is None:self.dwell=now
            if now-self.dwell>=self.model.stable_seconds:self.event('DESCENT_ASSEMBLED',now)
        else:self.dwell=None
        return self.output(now,velocity=velocity)

    def step(self,p,v,now,dt,blocked,hold_leader=False):
        p=np.asarray(p,float);v=np.asarray(v,float);dt=min(.25,max(0.,dt))
        if self.phase=='ASSEMBLED':
            return None if self.reference is None else self.output(now,stage='ASSEMBLE',agl=self.model.assembly_agl)
        if self.phase in ('DESCENT_ASSEMBLED','TRANSFER_READY','CLIMB_READY','CRUISE_READY'):
            self.last_progress=now
            return self.output(now)
        if hold_leader:self.last_progress=now
        if self.last_position is None or np.max(np.linalg.norm(p-self.last_position,axis=1))>.5:
            self.last_progress=now;self.last_position=p.copy()
        protected_wait=self.phase=='TERRAIN' and getattr(self,'terminal_waiting',False)
        if self.phase=='TERRAIN' and getattr(self,'terminal_wait_was_active',False) and not protected_wait:
            # A healthy certified owner is resuming after its protected wait.
            # Start a new progress deadline without fabricating displacement.
            self.terminal_resume_stamp=now
        self.terminal_wait_was_active=protected_wait
        progress_origin=max(self.last_progress,getattr(self,'terminal_resume_stamp',-1e9) if self.phase=='TERRAIN' else -1e9)
        if self.phase not in ('WAIT','FINAL') and now-progress_origin>90 and not protected_wait:
            raise ValueError('No safe group progress: '+self.phase+' '+self.note)
        if self.phase=='WAIT':
            if now>=self.release_time:self.next_segment(p,now)
            else:return self.output(now)
        if self.phase=='CLIMB' and self.relay and now-self.release_time>=16.:
            return self.relay_climb(p,v,now,blocked,hold_leader)
        if self.phase=='RELAY_DESCENT':return self.relay_descent(p,v,now,blocked,hold_leader)
        if self.phase=='CRUISE':return self.cruise(p,v,now,blocked,hold_leader)
        if self.phase=='MOVING_MERGE':return self.moving_descent(p,v,now,dt,blocked,hold_leader)
        if self.phase in ('TERRAIN','FINAL'):return self.terminal(p,v,now,blocked,hold_leader)
        old=self.reference.copy()
        if self.relay and self.phase in ('CLIMB','DESCEND','DESCENT_MERGE'):
            # Relay followers are not tracking the group reference, so only the
            # leader can gate the member ramp.
            reference_ready=np.linalg.norm(p[2]-self.reference[2])<2.
        else:
            lag=p-self.reference
            if self.phase=='AIR_ASSEMBLE':lag=lag[:,:2]
            reference_ready=np.max(np.linalg.norm(lag,axis=1))<2.
        if not blocked and not hold_leader and reference_ready:
            length=float(np.linalg.norm(self.destination-self.origin,axis=1).max())
            if self.phase=='CLIMB' and getattr(self.model,'synchronized_climb',False):
                self.climb_elapsed+=dt
                for rank,ident in enumerate(group_order(self.group)):
                    i=ident%5
                    if self.climb_elapsed<rank*self.model.member_departure_interval:continue
                    u=self.member_u[i]
                    # Phase 2: the relay leader must vacate the square slot
                    # quickly (forward+up), so its initial ramp is faster.
                    floor=.5 if (self.relay and i==2) else .15
                    ramp=min(1.,max(floor,np.sqrt(1.2*length*min(u,1-u))/max(length/self.duration,1e-6)))
                    self.member_u[i]=min(1.,u+dt*ramp/self.duration)
                self.u=float(self.member_u.min())
                self.reference=self.origin+(self.destination-self.origin)*self.member_u[:,None]
            else:
                ramp=min(1.,max(.15,np.sqrt(1.2*length*min(self.u,1-self.u))/max(length/self.duration,1e-6)))
                self.u=min(1.,self.u+dt*ramp/self.duration)
                self.reference=self.origin+(self.destination-self.origin)*self.u
        vel=(self.reference-old)/max(dt,1e-6)
        if self.phase=='AIR_ASSEMBLE':
            # A member can still close on the frozen reference through its
            # local safety checks. The coordinator owns the fleet-wide dwell.
            return self.output(now,velocity=vel,pause=hold_leader,agl=self.model.assembly_agl)
        if getattr(self.model,'cloud_only',False) and self.phase in ('ALIGN_LIFT','ALIGN_MOVE','ALIGN_SETTLE','STAR_LIFT','STAR_MOVE','STAR_SETTLE') and not blocked and not hold_leader:
            # Swarm-Formation's normalized-Laplacian shape cost supplements
            # metric slot tracking. Layered transition corridors stay intact;
            # each aircraft still certifies its complete stop and peer motion.
            correction=-.4*graph_gradient(p,self.reference)
            correction*=np.minimum(1.,.25/np.maximum(np.linalg.norm(correction,axis=1),1e-9))[:,None]
            vel+=correction
        if self.relay and self.phase=='CLIMB':
            # Cruise altitude reached: hand over to CRUISE without stopping.
            arrived=abs(p[2][2]-self.destination[2][2])<1.0
        elif self.relay and self.phase in ('DESCEND','DESCENT_MERGE'):
            arrived=np.linalg.norm(p[2]-self.destination[2])<.5 and np.linalg.norm(v[2])<.2
        else:
            arrived=self.u==1 and np.max(np.linalg.norm(p-self.destination,axis=1))<.5 and np.max(np.linalg.norm(v,axis=1))<.2
        if arrived:
            if self.dwell is None:self.dwell=now
            if now-self.dwell>=1.:
                if self.queue:
                    if self.queue[0][0] in ('ALIGN_MOVE','CLIMB') and getattr(self.model,'synchronized_climb',False):self.event('TRANSFER_READY' if self.queue[0][0]=='ALIGN_MOVE' else 'CLIMB_READY',now)
                    else:self.next_segment(p,now)
                elif self.phase=='CLIMB':
                    if getattr(self.model,'synchronized_climb',False):self.event('CRUISE_READY',now)
                    else:self.start_cruise(p,now)
                elif self.phase=='DESCEND':
                    self.reference=p.copy()
                    if getattr(self.model,'cloud_only',False):self.event('DESCENT_ASSEMBLED',now)
                    else:self.event('TERRAIN',now);self.terminal_origin=self.model.assembly_anchor(self.group)
                vel=np.zeros((5,3))
        else:self.dwell=None
        return self.output(now,velocity=vel,pause=blocked or hold_leader)

    def leader_stamp(self,now):
        # The production coordinator sets this from its paired state before
        # every step, including the synchronized CRUISE_READY seed. Legacy
        # direct engine callers without observation metadata sample at `now`.
        sample=float(getattr(self,'leader_sample_stamp',now))
        if not np.isfinite(sample) or not -.1<=now-sample<=self.trail.max_sample_gap:
            raise ValueError('Expired/future leader position sample')
        return sample

    def cruise(self,p,v,now,blocked,hold_leader):
        self.trail.append(p[2],self.leader_stamp(now))
        forward=getattr(self.model,'forward_departure',False)
        # Two metres is the existing allowed small forward overshoot. Larger
        # prediction would conflict with the five-metre formation lag guard.
        anticipation=min(2.,.5*float(np.linalg.norm(v[2]))) if forward else 0.
        nominal={}
        if forward:
            predecessor=2
            for rank,ident in enumerate(group_order(self.group)):
                i=ident%5
                if not rank:continue
                target=self.trail.slot(rank,now)
                if i in self.waiting:
                    ahead=target is not None and np.dot(target[:2]-self.wait_anchors[i,:2],self.departure_direction)>=2.
                    clear=np.dot(p[predecessor,:2]-self.wait_anchors[i,:2],self.departure_direction)>=self.model.spacing
                    if ahead and clear and predecessor not in self.waiting:
                        self.waiting.remove(i);self.joining.add(i)
                    else:self.reference[i]=self.wait_anchors[i]
                if i not in self.waiting and target is not None:
                    nominal[i]=target
                    self.reference[i]=self.trail.slot(rank,now,anticipation)
                    if np.linalg.norm(p[i]-target)<5. and np.linalg.norm(v[i]-v[2])<1.:self.joining.discard(i)
                predecessor=i
        else:
            history=self.trail.targets(now)
            if history is None:self.reference=p.copy()
            else:
                for rank,i in enumerate(group_order(self.group)):self.reference[i%5]=history[rank]
        goal=self.navigation[self.waypoint]
        stop_gate=25.
        if self.waypoint==len(self.navigation)-1 and getattr(self.model,'high_speed_cloud',False):
            speed=float(np.linalg.norm(v[2,:2]));stop_gate=max(25.,speed*speed/4.4+2.*speed+20.)
        if self.descent_braking or np.linalg.norm(p[2,:2]-goal)<stop_gate:
            if self.waypoint<len(self.navigation)-1:
                self.waypoint+=1;goal=self.navigation[self.waypoint];self.guide=None
            else:
                if self.relay and getattr(self.model,'moving_merge',False):
                    anchor=self.model.assembly_anchor(self.group)
                    altitude=232. if self.group==0 else 263.
                    ahead=p[2,:2]+1500.*self.climb_axis
                    descent_time=max(2.,(p[2,2]-altitude)/1.6)
                    self.descent_profile=dict(origin=p[2,:2].tolist(),anchor=ahead.tolist(),from_z=float(p[2,2]),
                                              to_z=altitude,level_after=max(200.,descent_time*np.linalg.norm(v[2,:2])))
                    self.event('RELAY_DESCENT',now)
                    return self.relay_descent(p,v,now,blocked,hold_leader)
                if getattr(self.model,'moving_merge',False):
                    from moving_merge import MovingMerge
                    self.merge_plan=MovingMerge(self.model,self.group,p,v,getattr(self,'merge_entry_acceleration',None))
                    self.merge_elapsed=0.;self.reference=p.copy();self.event('MOVING_MERGE',now)
                    # The ahead hold was selected for CRUISE before this phase
                    # transition. Only that soft wait ends; real blockers stay.
                    return self.moving_descent(p,v,now,0.,blocked,False)
                self.descent_braking=True
                if np.max(np.linalg.norm(v,axis=1))>.2:
                    self.reference=p.copy();return self.output(now,pause=True)
                if getattr(self.model,'descending_merge',False):self.queue=self.model.descent_merge(self.group,p)
                else:
                    start,end=self.model.descent(self.group)
                    self.queue=list(zip(('STAR_LIFT','STAR_MOVE','STAR_SETTLE'),transition(p,start)))
                    self.queue.extend(('DESCEND',q) for q in self.model.descent_path(self.group)[1:])
                self.next_segment(p,now)
                return self.output(now)
        if self.relay and hasattr(self,'climb_course'):
            self.reference=p.copy()
            self.reference[2]=np.r_[goal,self.model.altitude]
            self.note='Continuous straight cruise toward group endpoint'
            return self.output(now,pause=blocked or hold_leader)
        if self.guide is None or now-self.guide_stamp>=2. or np.linalg.norm(p[2,:2]-self.guide[:2])<8:
            self.hint=terrain_direction(p[2],np.r_[goal,self.model.altitude],self.model.cloud(p[2]),1 if self.group==0 else -1,cruise_z=self.model.altitude,lookahead=300. if getattr(self.model,'high_speed_cloud',False) else 100.)
            self.guide=np.array(self.hint['target']);self.guide_stamp=now;self.note=self.hint['reason']
        paused=bool(blocked or hold_leader or not self.hint.get('available',self.guide is not None))
        follower_ids=[i for i in (0,1,3,4) if not forward or i not in self.waiting]
        # Compare actual lag to nominal slots, never to anticipated positions.
        if not self.relay:
            paused|=any(np.linalg.norm(p[i]-nominal.get(i,self.reference[i]))>(40. if forward and i in self.joining else 5.) for i in follower_ids)
        self.reference[2]=self.guide if self.guide is not None else p[2]
        if min_separation(self.reference)<2.:
            self.reference=p.copy();return self.output(now,pause=True)
        feed=np.zeros((5,3));directions=self.trail.directions(now);slot_directions=np.zeros((5,3))
        if forward:
            directions=np.array([np.zeros(3) if self.trail.slot_direction(rank,now,anticipation) is None else self.trail.slot_direction(rank,now,anticipation) for rank in range(5)])
        if directions is not None:
            for rank,i in enumerate(group_order(self.group)):
                slot_directions[i%5]=directions[rank]
                if rank and (not forward or i%5 not in self.waiting):feed[i%5]=directions[rank]*min(getattr(self.model,'cruise_speed',6.),np.linalg.norm(v[2]))
        feed[:,:2]*=np.minimum(1.,getattr(self.model,'cruise_speed',6.)/np.maximum(np.linalg.norm(feed[:,:2],axis=1),1e-9))[:,None]
        result=self.output(now,velocity=feed,paused=sorted((self.waiting if forward else set())|({2} if paused else set())))
        if forward:result.update(waiting_members=sorted(self.waiting),joining_members=sorted(self.joining))
        if getattr(self.model,'cloud_only',False):
            paths=[[] for _ in range(5)]
            for rank,i in enumerate(group_order(self.group)):
                if rank and (not forward or i%5 not in self.waiting):paths[i%5]=self.trail.route_to_slot(p[i%5],rank,now,anticipation)
            result['trail_paths']=paths
        # A short reversal can make the symmetric tangent exactly zero. Keep
        # its zero feed, but omit the optional overshoot hint for this frame.
        if directions is not None and np.all(np.abs(np.linalg.norm(slot_directions,axis=1)-1.)<.01):
            result['trail_directions']=slot_directions.tolist()
        return result

    def terminal(self,p,v,now,blocked,hold_leader):
        end=self.model.ends[self.group]
        remaining=np.linalg.norm(p[2,:2]-end)
        if getattr(self.model,'independent_terminal',False):
            final=self.model.landing_slots(self.group)
            remaining=np.linalg.norm(p[:,:2]-final[:,:2],axis=1).max()
            agl=self.model.terminal_agl((40.-remaining)/32.)
            self.reference=final.copy()
            self.reference[:,2]=self.model.ground(final[:,:2])+agl
            self.note='Independent YOPO to fixed star slots'
            ref=self.output(now,stage='TERRAIN',pause=hold_leader,agl=agl)
            ref.update(independent_terminal=True,fixed_endpoints=final.tolist(),moving_slots=False)
            from terminal_star_approach import StarApproach,certificates_ready,boundary_matches,settled_boundary
            approach=getattr(self,'star_approach',None)
            states=getattr(self,'member_states',[])
            start=getattr(self,'star_approach_start',None)
            if (remaining<=30. and not hasattr(self,'star_prepare_since')
                    and now>=getattr(self,'star_retry_after',-1e9)):self.star_prepare_since=now
            preparing=hasattr(self,'star_prepare_since')
            if preparing:ref['star_prepare']=True
            boundary=None
            if preparing and len(states)==5:
                try:
                    from relay_core import validated_control_reference
                    samples=[validated_control_reference(s.get('control_reference'),now) for s in states]
                    boundary=[np.array([s[key] for s in samples]) for key in ('position','velocity','acceleration')]
                except ValueError:pass
            if approach is not None and start is None:
                certificates=[s.get('formation',{}).get('star_certificate') for s in states]
                if (certificates_ready(certificates,approach.token,now) and boundary is not None
                        and boundary_matches(approach,*boundary) and boundary_matches(approach,p,v,boundary[2])
                        and settled_boundary(p,v,boundary)
                        and now<self.star_planned_start-.2):
                    self.star_approach_start=start=self.star_planned_start
                elif now-self.star_proposed>.8:self.star_approach=approach=None
            if (approach is None and preparing and remaining<=30. and boundary is not None
                    and settled_boundary(p,v,boundary) and not hold_leader
                    and now-getattr(self,'star_attempt',-1e9)>1.):
                self.star_attempt=now
                try:
                    target=final.copy();target[:,2]=self.model.ground(final[:,:2])+self.model.end_agl
                    approach=StarApproach(*boundary,target,'star-%d-%.6f'%(self.group,now))
                    self.star_approach=approach;self.star_proposed=now;self.star_planned_start=now+1.2
                except ValueError:pass
            if preparing and start is None and now-self.star_prepare_since>12.:
                # Admission failure is still YOPO ownership. Resume observation,
                # rather than keeping a group stopped indefinitely at a bad slot.
                self.star_approach=approach=None
                del self.star_prepare_since
                self.star_retry_after=now+15.
                ref.pop('star_prepare',None)
            if approach is not None:
                ref['star_approach']=dict(approach.packet(),start=start)
                if start is not None:
                    ref.update(agl=self.model.end_agl,speed=3.,targets=approach.sample(approach.duration)[0].tolist())
                    self.note='Committed formation-only star approach'
            cohesion=self.model.config.get('terminal_cohesion',{})
            if cohesion.get('enabled',False) and start is None:
                if self.distance_gate is None:
                    from tree_distance import TreeDistanceGate
                    from formation_speed_governor import FormationSpeedGovernor
                    minimum=cohesion.get('minimum_speed_mps',2.)
                    self.formation_governor=FormationSpeedGovernor(self.model.star_offsets(self.group),
                        minimum_speed=minimum,nominal_speed=float(ref['speed']),
                        coefficient_tolerance=cohesion.get('coefficient_tolerance',.02))
                    self.distance_gate=TreeDistanceGate(self.model.star_offsets(self.group),
                        trigger=cohesion.get('trigger_m',50.),resume=cohesion.get('resume_m',25.),
                        slow_factor=cohesion.get('slow_factor',.35),max_speed=float(ref['speed']),
                        lookahead=cohesion.get('lookahead_s',3.5),braking_margin=cohesion.get('braking_margin_m',22.),
                        minimum_speed=minimum)
                formation=self.formation_governor.update(p,v,now,
                    forward=self.formation_governor.initial_axis if remaining<40. else None)
                formation['culprit_global_members']=[5*self.group+i for i in formation['culprit_members']]
                self.cohesion=self.distance_gate.update(p,v,fresh=getattr(self,'distance_observations_fresh',True),formation=formation)
                self.cohesion['stamp']=now
                self.cohesion['sample_stamps']=list(getattr(self,'distance_sample_stamps',[now]*5))
                ref.update(cohesion_required=True,cohesion=self.cohesion,
                           member_speed_caps=[float(ref['speed'])]*5)
            if not self.model.star_settled(self.group,p,v):
                self.dwell=None
                if self.phase=='FINAL':self.event('TERRAIN',now)
            elif self.phase=='TERRAIN':
                if self.dwell is None:self.dwell=now
                if now-self.dwell>=2.:self.event('FINAL',now)
            return ref
        scheduled=self.model.terminal_agl(float(np.clip((40.-remaining)/32.,0.,1.)))
        agl=min(getattr(self,'terminal_agl_target',self.model.arrival_agl),scheduled)
        self.terminal_agl_target=agl
        targets=p.copy();targets[:,:2]=p[2,:2]+STAR
        # Near the endpoint all five use their fixed, prevalidated slots.
        if remaining<15:targets[:,:2]=end+STAR
        else:targets[2,:2]=end
        targets[:,2]=self.model.ground(targets[:,:2])+agl;self.reference=targets
        error=np.linalg.norm((p[:,:2]-p[2,:2])-STAR,axis=1);error[2]=0
        paused=hold_leader or bool(error.max()>2.)
        # Local blockers pause only the leader, allowing trailing slots to settle.
        paused|=bool(blocked)
        moving=np.linalg.norm(p[2,:2]-end)>=15
        feed=np.tile(v[2],(5,1)) if moving else np.zeros((5,3));feed[2]=0
        feed[:,:2]*=np.minimum(1.,6/np.maximum(np.linalg.norm(feed[:,:2],axis=1),1e-9))[:,None]
        r=self.output(now,stage='TERRAIN',velocity=feed,paused=[2] if paused else [],agl=agl)
        r.update(fixed_endpoints=self.model.landing_slots(self.group).tolist(),moving_slots=bool(moving))
        arrived=self.model.star_settled(self.group,p,v)
        if arrived:
            if self.dwell is None:self.dwell=now
            if now-self.dwell>=2:self.event('FINAL',now)
        else:self.dwell=None
        return r
