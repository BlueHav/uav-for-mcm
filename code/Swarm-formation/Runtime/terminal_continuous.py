"""Future-PVA joint replanning with retained mixed old/new reservations."""
import copy
from collections import deque
from concurrent.futures import ProcessPoolExecutor
from concurrent.futures.process import BrokenProcessPool
from multiprocessing import get_context
import numpy as np
from terminal_coordinator import stationary_states
from terminal_formation import (SAMPLE_PADDING, TRACKING_LIMIT, _corners,
                                packet_paths, make_packet, spatially_separated, tracking_reservation)
from terminal_stopping import actual_stop_paths
from terminal_shape import UNIT_STAR,COMPACT_RADIUS,shape_state
from terminal_rollout import build_rollout
from terminal_observation import nearby_trees
from static_perception import launch_observation_covers_switch,state_sensors_fresh,matching_permission
from terminal_hint import usable_hint


TERRAIN_TRAVEL_RADIUS=16.


class ReplanBudget:
    """Measured lead time; never extends a certificate or freshness deadline."""
    def __init__(self):self.build=deque([.8],maxlen=32);self.admission=deque([1.1],maxlen=32)
    def observe_build(self,seconds):
        if np.isfinite(seconds) and seconds>=0:self.build.append(float(seconds))
    def observe_admission(self,seconds):
        if np.isfinite(seconds) and seconds>=0:self.admission.append(float(seconds))
    def lead(self):return max(2.,float(max(self.build)+max(self.admission)+.35))


def observed_escape_direction(positions,hits):
    """Propose a common heading away from the members' near returns."""
    away=[]
    for member,hit in hits.items():
        delta=np.asarray(positions[member],float)[:2]-np.asarray(hit,float)[:2]
        length=np.linalg.norm(delta)
        if .1<length<=4.:
            away.append(delta/length)
    if not away:return None
    heading=np.sum(away,axis=0)
    length=np.linalg.norm(heading)
    if length<.1:return None
    heading/=length
    return heading if np.min(np.asarray(away)@heading)>.1 else None


def wall_bypass(world,positions,hits,end):
    """Use a depth-confirmed facade to keep a local bypass goal across switches."""
    if not hits:return []
    p=np.asarray(positions,float)
    center=p[2,:2]
    radius=max(float(shape_state(p)[0]),TERRAIN_TRAVEL_RADIUS)
    points=np.asarray(list(hits.values()),float)
    for obj in world.get('objects',()):
        if obj.get('kind')!='building':continue
        middle=np.array([obj['x'],obj['y']])
        half=np.array([obj['width'],obj['depth']])*.5
        on_facade=(np.abs(points[:,:2]-middle)<=half+.75).all(axis=1)
        on_facade &= (np.abs(np.abs(points[:,:2]-middle)-half)<=.75).any(axis=1)
        on_facade &= (points[:,2]>=obj.get('bottom_z',obj['base_z'])-.5)&(points[:,2]<=obj['base_z']+obj['height']+.5)
        if not np.any(on_facade):continue
        escape=observed_escape_direction(positions,hits)
        if escape is None:continue
        lower=middle-half-radius-2.
        upper=middle+half+radius+2.
        end=np.asarray(end,float)
        routes=[]
        for axis in range(2):
            if abs(escape[axis])<.2:continue
            side=0 if escape[axis]<0 else 1
            edge=(lower,upper)[side]
            first=center.copy();first[axis]=edge[axis]
            if (first[axis]-center[axis])*escape[axis]<0:continue
            # An exit bearing for the observing member can sweep a different
            # star arm through the same facade before reaching the waypoint.
            destination=p[:,:2]+first-center
            sweep_min=np.minimum(p[:,:2],destination)
            sweep_max=np.maximum(p[:,:2],destination)
            crosses=((sweep_min<=middle+half+.5)&(sweep_max>=middle-half-.5)).all(axis=1)
            if np.any(crosses):continue
            tangent=1-axis
            if (end[axis]-edge[axis])*escape[axis]>=0:
                routes.append([first])
                continue
            for finish in (lower,upper):
                corner=first.copy();corner[tangent]=finish[tangent]
                route=[first,corner]
                if (end[tangent]-finish[tangent])*(finish[tangent]-middle[tangent])<0:
                    last=corner.copy();last[axis]=(upper,lower)[side][axis]
                    route.append(last)
                routes.append(route)
        if routes:
            return min(routes,key=lambda route:sum(np.linalg.norm(b-a) for a,b in zip([center]+route,route+[end])))
    return []


def settled_peer_envelopes(slot,states,now):
    """Reserve the live brake at a clamped endpoint after all five settle."""
    packet=slot['packet']
    end=slot['start']+max(sum(durations) for durations in packet['durations'])
    if not slot['committed'] or now<end:
        return None
    for state in states:
        e=state.get('execution',{})
        odom=e.get('odom_stamp',-1e9)
        if (e.get('active')!=packet['digest'] or e.get('active_start')!=slot['start']
                or not e.get('tail_settled') or e.get('error')
                or not -.05<=now-e.get('stamp',-1e9)<=.35
                or odom<end or not -.05<=now-odom<=.5):
            return None

    zero=np.zeros(3)
    boxes=[]
    for plan in packet_paths(packet):
        # The owner still follows the same packet and checks its full map
        # certificate. Only the peer reservation loses past positions.
        # At the clamped endpoint the reference velocity and acceleration
        # are zero. The original tracking tolerances bound live drift and
        # speed, and the existing two brakes bound each axis's displacement.
        position,_,_=plan.sample(plan.duration)
        speed,_=tracking_reservation(plan)
        horizontal=actual_stop_paths(zero,[speed,0.,0.],zero,zero)
        vertical=actual_stop_paths(zero,[0.,0.,speed],zero,zero)
        xy=max(np.abs(path[:,0]).max() for path in horizontal)
        z=max(np.abs(path[:,2]).max() for path in vertical)
        padding=np.array([xy,xy,z])+TRACKING_LIMIT+SAMPLE_PADDING
        boxes.append(_corners(position-padding,position+padding))
    return boxes


def union_reservations(states,active,offers,inactive_observations=None,now=None):
    from inactive_reservations import reservation_positions
    inactive={} if inactive_observations is None else reservation_positions(inactive_observations,now)
    if set(states)&set(inactive) or set(states)|set(inactive)!=set(range(10)):raise ValueError('Reservations require distinct real active and inactive bodies')
    boxes=[np.asarray([states[i]['position'] if i in states else inactive[i]],float) for i in range(10)]
    for group in (0,1):
        for slot in (active[group],offers[group]):
            if slot is None:continue
            for member,box in enumerate(slot.get('settled_envelopes',slot['packet']['envelopes'])):
                i=5*group+member;boxes[i]=np.vstack((boxes[i],box))
    return boxes


def initial_start_matches(states,packet):
    """A retained geometry proof cannot certify a subsequently moved start."""
    if not stationary_states(states):return False
    for state,plan in zip(states,packet_paths(packet)):
        p,v,a=plan.sample(0.)
        actual=np.asarray(state.get('position',[]),float)
        reference=np.asarray(state.get('control_reference',{}).get('position',[]),float)
        if (actual.shape!=(3,) or reference.shape!=(3,) or not np.isfinite(actual).all()
                or not np.isfinite(reference).all() or np.linalg.norm(actual-p)>=.10
                or np.linalg.norm(reference-p)>=.10 or np.linalg.norm(v)>=.035 or np.linalg.norm(a)>=.06):return False
    return True


class ContinuousCoordinator:
    def __init__(self,model,pool=None,clock=None):
        self.model=model;self.clock=clock
        self.pool=pool or ProcessPoolExecutor(max_workers=2,mp_context=get_context('spawn'))
        self.active=[None,None];self.offers=[None,None];self.work=[None,None];self.serial=0
        self.budgets=[ReplanBudget(),ReplanBudget()]
        self.cursor=[0,0];self.tokens=[None,None];self.departed=[False,False];self.contracting=[False,False]
        self.retry_count=[0,0];self.avoidance_members=[(),()];self.observation_caps=[None,None];self.pool_broken=False
        self.rejected_observation_contract=set()
        self.avoidance_hits=[{},{}]
        self.wall_bypass=[[],[]]
        self.departure_goals=[model.assembly_anchor(g)+44.*model.offset(g)/np.linalg.norm(model.offset(g)) for g in (0,1)]
        self.metrics=dict(native_calls=0,committed=[0,0],map_rejections=[0,0],admission_retries=[0,0],continuous_switches=[0,0],
                          certificate_commits=[0,0],certificate_input_gap_commits=[0,0],
                          opportunistic_replans=[0,0],estimated_replan_lead=[None,None],scheduled_replan_lead=[None,None],
                          switch_speeds=[[],[]],late_rollouts=[0,0],fallback_stops=[0,0],
                          planning_failures=[0,0],last_planning_error=None,
                          minimum_envelope_distance=None,last_optimizer=None)
    def close(self):self.pool.shutdown(wait=False)

    def reserved(self,states,now=None):return union_reservations(states,self.active,self.offers,getattr(self,'inactive_observations',None),now)

    def waiting(self,group,states,now):
        slot=self.active[group]
        if slot is None:return False
        for i in range(group*5,group*5+5):
            state=states[i];e=state.get('execution',{});joint=state.get('terminal_joint',{})
            if (not -.05<=now-e.get('stamp',-1e9)<=.35 or e.get('error') or not e.get('tail_settled')
                    or e.get('active')!=slot['packet']['digest'] or joint.get('digest')!=slot['packet']['digest']
                    or not joint.get('committed')):return False
            if not state_sensors_fresh(state,now) and not matching_permission(e,slot['packet'],slot['start'],now,
                    getattr(self.model,'hashes',{}).get('world')):return False
        return True

    def reference(self,group,states,reference,run):
        reference=dict(reference,moving_slots=False,paused_members=[])
        center=np.asarray(states[group*5+2]['position'])[:2]
        other_id=7 if group==0 else 2
        other=np.asarray(states[other_id]['position'])[:2] if other_id in states else None
        gap=np.linalg.norm(center-other) if other is not None else None
        # Wide stars put the outside members into obstacles beside the route.
        # Retain the lateral departure maneuver only while the groups need it.
        if np.linalg.norm(center-self.departure_goals[group])<2. or (gap is not None and gap>2.*TERRAIN_TRAVEL_RADIUS+4.):
            self.departed[group]=True
        if self.departed[group] and np.linalg.norm(center-self.model.ends[group])<=40.:
            self.contracting[group]=True
        radius=TERRAIN_TRAVEL_RADIUS if gap is None else min(TERRAIN_TRAVEL_RADIUS,max(COMPACT_RADIUS,(gap-4.)/2.))
        if self.contracting[group]:radius=COMPACT_RADIUS
        cap=self.observation_caps[group]
        if cap is not None:
            if np.linalg.norm(center-cap[1])>=5.:
                self.observation_caps[group]=None
                self.cursor[group]=0
            else:radius=min(radius,cap[0])
        goal=self.model.ends[group] if self.departed[group] else self.departure_goals[group]
        route=self.wall_bypass[group]
        while route and np.linalg.norm(center-route[0])<1.:
            route.pop(0)
            self.cursor[group]=0
        if route:goal=route[0]
        phase='contract' if self.contracting[group] else 'travel' if self.departed[group] else 'depart'
        token='%s:%d:%s'%(run,group,phase)
        if self.tokens[group]!=token:
            self.tokens[group]=token;self.cursor[group]=0;self.retry_count[group]=0
            self.avoidance_hits[group].clear()
            self.rejected_observation_contract.discard(group)
        reference.update(terminal_intent_token=token,terminal_center_goal=np.asarray(goal).tolist(),
                         terminal_target_radius=radius,terminal_contracting=self.contracting[group],
                         terminal_departed=self.departed[group],continuous=True,
                         terminal_force_contract=self.retry_count[group]>=8,
                         terminal_skip_observation_contract=group in self.rejected_observation_contract)
        targets=self.model.landing_slots(group).copy()
        if route or not self.departed[group]:targets[:,:2]=goal+UNIT_STAR*radius
        reference['targets']=targets.tolist()
        return reference

    def completed(self,group,slot,states):
        self.retry_count[group]=0
        self.rejected_observation_contract.discard(group)
        if slot.get('stats',{}).get('action')=='contract_for_observation':
            plans=packet_paths(slot['packet'])
            final=np.array([p.sample(p.duration)[0] for p in plans])
            radius,_=shape_state(final)
            self.observation_caps[group]=(float(radius),np.array(states[5*group+2]['position'])[:2])

    def restart_cursor(self,group,slot):
        # Probe the next longer candidate after each admitted travel segment.
        # The search still falls back to shorter candidates on a refusal.
        if (self.departed[group] and not self.contracting[group] and slot.get('token')==self.tokens[group]
                and slot.get('stats',{}).get('action')=='translate'):
            stats=slot['stats']
            cursor=stats.get('restart_cursor',1)
            if (self.observation_caps[group] is None and cursor>=7
                    and stats.get('formation_radius',0.)+.5<stats.get('target_radius',0.)):
                return 0
            return max(1,cursor-1)
        return 0

    def submit(self,group,states,reference,now,run):
        if self.pool_broken:
            if any(x is not None for x in self.work):return
            self.pool.shutdown(wait=False)
            self.pool=ProcessPoolExecutor(max_workers=2,mp_context=get_context('spawn'));self.pool_broken=False
        own=[states[i] for i in range(group*5,group*5+5)]
        if not all(state_sensors_fresh(s,now) for s in own):return
        if not all(-.05<=now-s.get('kinematic_stamp',s['stamp'])<=.3 for s in states.values()):return
        if not all(s.get('terminal_joint',{}).get('scanned') for s in own):return
        intents=[s.get('terminal_joint',{}).get('intent') for s in own]
        if any(not usable_hint(x,s['position'],now,reference['terminal_intent_token'],run,
                getattr(self.model,'hashes',{}).get('world'),
                static=bool(s.get('execution',{}).get('perception_enabled',False)))
                for x,s in zip(intents,own)):return
        active=self.active[group];snapshot=copy.deepcopy(states);ref=copy.deepcopy(reference)
        ref['terminal_avoidance']=self.retry_count[group]>0
        ref['avoidance_members']=self.avoidance_members[group]
        hits=self.avoidance_hits[group]
        for member,hit in tuple(hits.items()):
            if np.linalg.norm(hit[:2]-np.asarray(own[member]['position'])[:2])>4.:
                del hits[member]
        escape=observed_escape_direction([s['position'] for s in own],hits)
        if self.wall_bypass[group]:
            toward=np.asarray(ref['terminal_center_goal'])-np.asarray(own[2]['position'])[:2]
            length=np.linalg.norm(toward)
            if length>.1:
                ref['terminal_wall_escape']=toward/length
        elif escape is not None:
            ref['terminal_wall_escape']=escape
        ref['predecessor_flexible']=bool(active and active['packet'].get('flexible'))
        ref['observed_positions']=[state['position'] for state in states.values()]
        start=None;predecessor=None;opportunistic=False
        if active:
            if not all(s.get('terminal_joint',{}).get('digest')==active['packet']['digest'] and s.get('terminal_joint',{}).get('committed') for s in own):return
            plans=packet_paths(active['packet']);end=active['start']+plans[0].duration
            nominal=active['packet'].get('nominal_duration',plans[0].duration)
            start=max(active['start']+min(3.,nominal*.5),now+self.budgets[group].lead())
            latest_nominal=active['start']+nominal-.6
            if now+2.<=latest_nominal<start:
                # A pessimistic latency estimate is not a prohibition on
                # trying. Prepare within the still-certified nominal motion;
                # the existing result/admission deadlines discard a late job
                # and the original finite backup remains the owner.
                start=latest_nominal;opportunistic=True
            if start>end-.6:
                # A completed finite packet remains the owner of this hold.
                # Resume from its clamped endpoint, with the same predecessor.
                if not (now>=end and stationary_states(own) and all(
                        s.get('terminal_joint',{}).get('done') and s.get('execution',{}).get('tail_settled') for s in own)):return
                start=now+self.budgets[group].lead()
            predecessor=active['packet']['digest']
            for i,plan in enumerate(plans):
                p,v,a=plan.sample(min(plan.duration,start-active['start']));state=snapshot[5*group+i]
                state.update(position=p.tolist(),velocity=v.tolist(),
                             control_reference=dict(position=p.tolist(),velocity=v.tolist(),acceleration=a.tolist()))
            ref['continuous_old_boxes']=active.get('settled_envelopes',active['packet']['envelopes'])
        elif not stationary_states(own):return
        ref['planning_start']=start if start is not None else now+self.budgets[group].lead()
        if self.active[1-group] is not None:
            ref['peer_prediction']=dict(packet=self.active[1-group]['packet'],start=self.active[1-group]['start'])
        self.serial+=1
        job=dict(group=group,states=snapshot,reference=ref,run=run,serial=self.serial,stamp=now,
                 reserved=self.reserved(states,now),cursor=self.cursor[group],ends=np.asarray(self.model.ends),
                 trees=nearby_trees(getattr(self.model,'world',{}),[s['position'] for s in own]),
                 landing_slots=np.array([self.model.landing_slots(g) for g in (0,1)]))
        try:
            self.work[group]=dict(future=self.pool.submit(build_rollout,job),start=start,predecessor=predecessor,
                                  token=reference['terminal_intent_token'],submitted=now)
            self.metrics['estimated_replan_lead'][group]=self.budgets[group].lead()
            self.metrics['scheduled_replan_lead'][group]=None if start is None else start-now
            if opportunistic:self.metrics['opportunistic_replans'][group]+=1
        except BrokenProcessPool as exc:
            self.pool_broken=True;self.metrics['planning_failures'][group]+=1;self.metrics['last_planning_error']=str(exc)

    def update(self,group,states,reference,now,run):
        ref=self.reference(group,states,reference,run)
        own=[states[i] for i in range(group*5,group*5+5)]
        active=self.active[group];offer=self.offers[group]
        if offer and offer['committed'] and active:
            digest=offer['packet']['digest']
            switched=all(s.get('terminal_joint',{}).get('digest')==digest and s.get('execution',{}).get('active')==digest
                         and s.get('execution',{}).get('odom_stamp',-1e9)>=offer['start'] for s in own)
            if switched:
                self.completed(group,active,states)
                self.active[group]=active=offer;self.offers[group]=offer=None
                self.cursor[group]=self.restart_cursor(group,active);self.metrics['continuous_switches'][group]+=1
                self.metrics['switch_speeds'][group].append(float(np.linalg.norm(packet_paths(active['packet'])[2].sample(0.)[1])))
            elif now>offer['start']+.35:
                raise ValueError('Partial continuous switch; retain mixed reservations and stop')
        if active and not (offer and offer['committed']):
            joint=[s.get('terminal_joint',{}) for s in own]
            if now>active['start']+.35 and not all(
                    s.get('terminal_joint',{}).get('digest')==active['packet']['digest'] and
                    s.get('terminal_joint',{}).get('committed') and
                    s.get('execution',{}).get('active')==active['packet']['digest'] for s in own):
                raise ValueError('Partial initial/active packet activation; stop within retained reservations')
            settled=all(s.get('digest')==active['packet']['digest'] and s.get('done') for s in joint)
            if settled and stationary_states(own):
                if 'settled_envelopes' not in active and self.waiting(group,states,now):
                    boxes=settled_peer_envelopes(active,own,now)
                    if boxes is not None:active['settled_envelopes']=boxes
                # Keep ownership regardless of sensor freshness: neither the
                # mission nor another group may recycle this waiting space.
                if offer is None and not active.get('settlement_counted'):
                    self.completed(group,active,states)
                    active['settlement_counted']=True;self.metrics['fallback_stops'][group]+=1
                    self.cursor[group]=self.restart_cursor(group,active)
            elif now>active['start']+max(sum(x) for x in active['packet']['durations'])+5.:
                holds=all(any(c.get('digest')==active['packet']['digest'] and c.get('start')==active['start']
                    and c.get('mode')=='hold' and c.get('allowed') for c in s.get('execution',{}).get('perception_permissions',[]))
                    and matching_permission(s.get('execution',{}),active['packet'],active['start'],now) for s in own)
                if not holds:raise ValueError('Continuous finite tail did not settle')
        work=self.work[group]
        if work and work['future'].done():
            self.work[group]=None
            try:packet,stats=work['future'].result()
            except (TimeoutError,RuntimeError,OSError) as exc:
                self.metrics['planning_failures'][group]+=1;self.metrics['last_planning_error']=repr(exc)
                if isinstance(exc,BrokenProcessPool):self.pool_broken=True
                packet=None;stats=dict(next_cursor=self.cursor[group],error=repr(exc))
            self.metrics['native_calls']+=1;self.metrics['last_optimizer']=stats
            if work.get('submitted') is not None:self.budgets[group].observe_build(now-work['submitted'])
            obsolete=(work['token']!=ref['terminal_intent_token'] or
                      work['predecessor']!=(None if active is None else active['packet']['digest']))
            late=work['start'] is not None and now>work['start']-.8
            if late:self.metrics['late_rollouts'][group]+=1
            if packet and not obsolete and not late:
                packet=make_packet(run,2,packet['serial'],group,packet_paths(packet),now,envelopes=packet['envelopes'],flexible=packet.get('flexible',False))
                offer=dict(packet=packet,start=work['start'],predecessor=work['predecessor'],committed=False,
                           requested=now,token=work['token'],stats=stats)
                if offer['start'] is None and all(s.get('execution',{}).get('perception_enabled') for s in own):offer['start']=now+2.
                proposed=list(self.offers);proposed[group]=offer
                safe,distance=spatially_separated(union_reservations(states,self.active,proposed,getattr(self,'inactive_observations',None),now))
                if safe:
                    self.offers[group]=offer
                    previous=self.metrics['minimum_envelope_distance']
                    self.metrics['minimum_envelope_distance']=distance if previous is None else min(previous,distance)
                else:offer=None;self.cursor[group]+=1
            else:
                offer=None
                self.cursor[group]=0 if obsolete else (stats or {}).get('next_cursor',self.cursor[group]+1)
        offer=self.offers[group]
        if offer and not offer['committed']:
            digest=offer['packet']['digest']
            acks=[s.get('terminal_joint',{}).get('candidate') if active else s.get('terminal_joint',{}) for s in own]
            positive=all(a and a.get('digest')==digest and a.get('accepted')
                         and (a.get('checks') or {}).get('clear') is not False for a in acks)
            blockers=tuple(i for i,a in enumerate(acks)
                           if a and a.get('digest')==digest and (a.get('checks') or {}).get('clear') is False)
            geometry_refused=bool(blockers)
            static=all(s.get('execution',{}).get('perception_enabled') for s in own)
            world=getattr(self.model,'hashes',{}).get('world')
            # Fresh maps are required to MINT each exact prepared certificate.
            # Once minted, the closed static scene guard retains it until
            # revocation. The executor is the current admission authority;
            # a delayed parent ACK/image update cannot undo that admission.
            # An explicit newly observed conflict still vetoes the commit.
            cached=bool(static and isinstance(world,str) and world and all(
                matching_permission(s.get('execution',{}),offer['packet'],offer['start'],now,world,committed=False)
                and s.get('execution',{}).get('prepared')==digest
                and -.1<=now-s.get('execution',{}).get('odom_stamp',-1e9)<=.5 for s in own))
            sensors_fresh=all(state_sensors_fresh(s,now) for s in own)
            if any(s.get('execution',{}).get('perception_enabled') for s in own):
                ready=cached and not geometry_refused
            else:
                ready=positive and sensors_fresh and all(a and -.05<=now-a.get('check_stamp',-1e9)<=.20 for a in acks)
            unity=any(s.get('execution',{}).get('perception_source')=='unity' for s in own)
            if unity and self.clock is not None:now=max(now,self.clock())
            commit_lead=.13 if unity else .20
            if unity:
                # The exact executor proof below carries the latest map-check
                # time. Parent ACKs still identify and accept this candidate.
                ready=ready and positive and sensors_fresh
            if active:
                ready=ready and all(s.get('execution',{}).get('prepared')==digest for s in own)
            else:ready=ready and (initial_start_matches(own,offer['packet']) if cached else stationary_states(own))
            # Count preparation latency before waiting for the late switch
            # window. Otherwise each late commit increases the next lead.
            if ready and 'ready_at' not in offer:offer['ready_at']=now
            if ready and unity:
                ready=all(any(cert.get('digest')==digest and cert.get('start')==offer['start']
                              and launch_observation_covers_switch(offer['packet']['created'],cert,now,offer['start'])
                              for cert in state['execution'].get('perception_permissions',()))
                          for state in own)
            if ready and (offer['start'] is None or commit_lead<=offer['start']-now<=.9):
                if not spatially_separated(self.reserved(states))[0]:raise ValueError('Mixed continuous reservation conflict')
                if offer['start'] is None:offer['start']=now+.7
                offer['committed']=True;offer['committed_at']=now;self.metrics['committed'][group]+=1
                self.avoidance_members[group]=()
                if cached:
                    self.metrics['certificate_commits'][group]+=1
                    if not sensors_fresh:self.metrics['certificate_input_gap_commits'][group]+=1
                self.budgets[group].observe_admission(offer['ready_at']-offer['requested'])
                if active is None:self.active[group]=active=offer;self.offers[group]=offer=None
            elif (offer['start'] is None and now-offer['requested']>1.5) or (offer['start'] is not None and now>offer['start']-commit_lead):
                # A missed preparation window is a measured lower bound on
                # admission time, not a successful zero-cost sample. Do not
                # learn geometric refusals or intentional wait-until-start.
                if 'ready_at' not in offer and not geometry_refused and not unity:
                    self.budgets[group].observe_admission(now-offer['requested'])
                self.offers[group]=None
                if geometry_refused:
                    self.metrics['map_rejections'][group]+=1;self.retry_count[group]+=1
                    previous_blockers=self.avoidance_members[group]
                    # Keep the first observer's priority for this search.
                    # An alternating subset must not replay the first heading.
                    self.avoidance_members[group]=tuple(dict.fromkeys((*previous_blockers,*blockers)))
                    new_hit=False
                    for member in blockers:
                        detail=(acks[member].get('checks') or {}).get('map_refusal') or {}
                        hit=np.asarray(detail.get('position',[]),float)
                        if (detail.get('kind')=='occupied' and member not in self.avoidance_hits[group]
                                and hit.shape==(3,) and np.isfinite(hit).all()
                                and .1<np.linalg.norm(hit[:2]-np.asarray(own[member]['position'])[:2])<=2.):
                            self.avoidance_hits[group][member]=hit
                            new_hit=True
                    if new_hit and self.departed[group] and not self.contracting[group]:
                        route=wall_bypass(getattr(self.model,'world',{}),[s['position'] for s in own],
                                          self.avoidance_hits[group],self.model.ends[group])
                        if route:
                            self.wall_bypass[group]=route
                            ref=self.reference(group,states,reference,run)
                    if offer['stats'].get('action')=='contract_for_observation':
                        # Retrying an identical blocked contraction masks the
                        # next direction in the obstacle avoidance search.
                        self.rejected_observation_contract.add(group)
                        self.cursor[group]=1
                    else:
                        self.cursor[group]=(0 if not previous_blockers or new_hit else
                                            offer['stats'].get('next_cursor',self.cursor[group]+1))
                    ref['terminal_skip_observation_contract']=group in self.rejected_observation_contract
                else:self.metrics['admission_retries'][group]+=1
                offer=None
        def command(slot):
            if slot is None:return {'kind':'wait'}
            value=dict(kind='commit' if slot['committed'] else 'request',packet=slot['packet'])
            if slot['start'] is not None:value['start']=slot['start']
            if slot['predecessor'] is not None:value['predecessor']=slot['predecessor']
            return value
        if active:
            ref['terminal_joint']=dict(kind='stream',active=command(active),next=None if offer is None else command(offer))
        else:ref['terminal_joint']=command(offer)
        if not ref.get('terminal_hover') and self.offers[group] is None and self.work[group] is None:
            self.submit(group,states,ref,now,run)
        return ref
