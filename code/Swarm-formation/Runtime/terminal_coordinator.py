"""YOPO-directed five-body native packets with per-map two-phase acceptance."""
import copy
import math
import time
from concurrent.futures import ThreadPoolExecutor
import numpy as np
from point_mission_core import STAR
from terminal_native import TerminalNative
from terminal_formation import make_packet,packet_paths,stop_clouds,spatially_separated,PacketGate
from terminal_shape import UNIT_STAR,COMPACT_RADIUS,TRAVEL_RADIUS,AVOIDANCE_DEVIATION,shape_state,formation_kept,desired_radius
from terminal_backup import append_backup
from terminal_observation import curve_reference,optimizer_obstacles,peer_predictions,prioritized_directions,tree_canopies,tree_corridors_clear
from terminal_corridor import outer_box

def terminal_ground(intent,xy):
    try:
        resolution=float(intent.get('ground_resolution',.5))
        if resolution not in (.5,2.):return float('nan')
        q=(np.asarray(xy)-intent['ground_origin'])/resolution;ij=np.floor(q).astype(int)
        grid=np.asarray(intent['ground'],float)
        if grid.shape not in ((13,13),(31,31)) or (ij<0).any() or (ij>=np.array(grid.shape)-1).any():return float('nan')
        values=grid[ij[0]:ij[0]+2,ij[1]:ij[1]+2]
        if not np.isfinite(values).all() or (values>1e8).any():return float('nan')
        u=q-ij;return float(np.array([1-u[0],u[0]])@values@np.array([1-u[1],u[1]]))
    except (KeyError,ValueError,TypeError):return float('nan')

def stationary_states(states):
    for state in states:
        ref=state.get('control_reference',{})
        if 'acceleration' not in ref:return False
        if max(np.linalg.norm(state['velocity']),np.linalg.norm(ref['velocity']))>=.035 or np.linalg.norm(ref['acceleration'])>=.06:return False
    return True

class TerminalCoordinator:
    def __init__(self,model):
        # Certificate construction is Python-heavy. Two threads contend on the
        # GIL (~2.8s measured vs ~1.9s serial for both before batch sampling).
        # Each group still owns an independent packet and reservation.
        self.model=model;self.pool=ThreadPoolExecutor(max_workers=1)
        self.work=[None,None];self.pending=[None,None];self.started=[None,None]
        self.pending_token=[None,None]
        self.pending_next=[1,1];self.pending_action=[None,None]
        self.observation_caps=[None,None]
        self.cursor_tokens=[None,None];self.map_retry_count=[0,0]
        self.requested=[0.,0.];self.cursor=[0,0];self.serial=0
        self.departed=[False,False];self.contracting=[False,False]
        self.departure_goals=[model.assembly_anchor(g)+44.*model.offset(g)/np.linalg.norm(model.offset(g)) for g in (0,1)]
        self.metrics=dict(native_calls=0,committed=[0,0],map_rejections=[0,0],minimum_envelope_distance=None,last_optimizer=None)
    def close(self):self.pool.shutdown(wait=False)
    def reserved(self,states):
        result=[np.array([states[i]['position']],float) for i in range(10)]
        for g,packet in enumerate(self.pending):
            if packet is not None:result[g*5:g*5+5]=[np.asarray(x) for x in packet['envelopes']]
        return result
    def build(self,group,states,reference,run,serial,now,reserved,cursor):
        began=time.monotonic()
        own=[states[i] for i in range(group*5,group*5+5)];p=np.array([s['position'] for s in own])
        intents=[s['terminal_joint']['intent'] for s in own]
        observed=reference.get('observed_positions')
        obstacles=(optimizer_obstacles(intents,observed) if observed is not None else
                   [point for intent in intents for point in intent.get('obstacles',[])][:2560])
        trees=getattr(self,'trees',np.empty((0,5)))
        tree_hints=np.c_[tree_canopies(trees),np.maximum(trees[:,4],trees[:,3]*.30)+.35].tolist()
        obstacles=(tree_hints+obstacles)[:2560]
        terrain=[dict(origin=i['ground_origin'],resolution=i.get('ground_resolution',.5),values=i['ground']) for i in intents]
        if any(np.asarray(i['values']).shape!=(31,31) for i in terrain):terrain=[]
        token=reference.get('terminal_intent_token')
        if not isinstance(token,str) or any(not isinstance(i,dict) or i.get('token')!=token for i in intents):return None,None
        endpoints=self.model.landing_slots(group)
        radius,error=shape_state(p)
        restoring=bool(reference.get('predecessor_flexible') and error>.25)
        if error>(AVOIDANCE_DEVIATION if restoring else .6):
            raise ValueError('Measured terminal star shape lost')
        center_goal=np.asarray(reference.get('terminal_center_goal',self.model.ends[group]))
        remaining=float(np.linalg.norm(center_goal-p[2,:2]))
        target_radius=float(reference.get('terminal_target_radius',COMPACT_RADIUS))
        near=bool(reference.get('terminal_contracting') and radius<3.55 and remaining<3.)
        tree_detour=bool(len(trees) and reference.get('continuous') and not reference.get('terminal_contracting'))
        choices=prioritized_directions(intents,center_goal-p[2,:2],reference.get('avoidance_members',()),
                                      tree_detour=tree_detour,escape=bool(reference.get('terminal_force_contract')))
        wall_escape=reference.get('terminal_wall_escape') if not reference.get('terminal_contracting') else None
        escape_choices=[]
        if wall_escape is not None:
            # Independently certify two nearby bearings as well as the
            # measured escape. A corner may hide one of the three corridors.
            yaw=math.atan2(wall_escape[1],wall_escape[0])
            for offset in (0.,math.pi/4.,-math.pi/4.):
                direction=np.array([math.cos(yaw+offset),math.sin(yaw+offset)])
                escape_choices.append((direction,None,None))
            choices=escape_choices+choices
        if not choices and near:choices=[(np.array([1.,0.]),2,0)]
        directions=[choice[0] for choice in choices]
        stats=None
        if not directions and not reference.get('terminal_contracting') and not reference.get('terminal_force_contract'):
            # A turning/unfinished leader search is not an observed geometric
            # obstruction. Retain the current owner and search cursor.
            return None,dict(intent_token=token,next_cursor=cursor,reason='awaiting_leader_direction')
        lengths=(26.,20.,12.,8.,4.,2.,1.,.5,.25)
        # 0 is the optional scale proposal. Then visit every length for one
        # YOPO direction before moving to the next direction. A map refusal
        # resumes AFTER the actual peer-safe candidate, not at the same size.
        direction_index,length_index=divmod(max(0,cursor-1),len(lengths))
        direction=None if direction_index>=len(directions) else np.array(directions[direction_index],float)
        if direction is not None and (direction.shape!=(2,) or not np.isfinite(direction).all() or abs(np.linalg.norm(direction)-1.)>.01):return None,None
        escaping=direction_index<len(escape_choices)
        shape_error=p[2,:2]+UNIT_STAR*radius-p[:,:2]
        candidates=[]
        scale_change=float(np.clip(target_radius-radius,-2.,2.))
        if not escaping and abs(scale_change)>.15 and (scale_change<0 or cursor==0):
            q=p.copy();q[:,:2]=p[2,:2]+UNIT_STAR*(radius+scale_change)
            if reference.get('continuous') and direction is not None:
                # Scale the common star while its center advances. The full
                # old/new stopping union below still decides admissibility.
                reach=min(26.,remaining,max(0.,min(radius,radius+scale_change)-2.6-(.12 if radius<6. else 2.5)))
                for advance in (reach,reach*.5):
                    if advance>.15:
                        moving=q.copy();moving[:,:2]+=direction*advance
                        candidates.append((moving,'contract' if scale_change<0 else 'expand',radius<6.,1))
            candidates.append((q,'contract' if scale_change<0 else 'expand',True,1))
        # Retain the complete expanded stopping box inside the 60 m rolling
        # window, including its 1 s velocity lead and map interpolation edge.
        for index,length in enumerate(lengths):
            if index<length_index:continue
            if direction is None or escaping and length>2.:continue
            if reference.get('terminal_contracting') and radius>3.55:continue
            if length>max(.5,radius-2.6) or length>remaining+.1:continue
            goals=p.copy()
            if near:
                delta=endpoints[:,:2]-p[:,:2]
                delta*=np.minimum(1.,length/np.maximum(np.linalg.norm(delta,axis=1),1e-9))[:,None]
                goals[:,:2]+=delta
            else:goals[:,:2]+=direction*length+shape_error
            candidates.append((goals,'final' if near else 'translate',escaping or radius<6.,2+direction_index*len(lengths)+index))
        can_try_contraction=radius>3.5 and not reference.get('terminal_skip_observation_contract',False)
        if can_try_contraction and not escaping and (direction_index>=len(directions) or reference.get('terminal_force_contract',False)):
            q=p.copy();q[:,:2]=p[2,:2]+UNIT_STAR*max(COMPACT_RADIUS,radius-2.)
            candidates.insert(0,(q,'contract_for_observation',True,cursor+1))
        with TerminalNative() as native:
            for goals,action,slow,next_cursor in candidates:
                goals[:,2]=[terminal_ground(intent,q[:2])+reference['agl'] for intent,q in zip(intents,goals)]
                if not np.isfinite(goals).all():continue
                distance=float(np.linalg.norm(goals-p,axis=1).max())
                if distance<.015 or distance>30.:continue
                old_boxes=reference.get('continuous_old_boxes')
                if old_boxes is not None:
                    # Necessary endpoint-only rejection avoids spending full
                    # stop integration time on an already intersecting union.
                    subset=list(reserved)
                    subset[group*5:group*5+5]=[np.vstack((p[i],goals[i],old_boxes[i])) for i in range(5)]
                    if not spatially_separated(subset)[0]:continue
                duration=max(6. if reference.get('continuous') else 2.,math.sqrt((12./1.1 if slow else 5.8/2.1)*distance),1.875*distance/6.)
                faster=(reference.get('continuous') and reference.get('terminal_departed')
                        and not reference.get('terminal_contracting') and not reference.get('terminal_avoidance')
                        and not restoring and not slow and action=='translate' and radius>=12.)
                durations=(max(4., distance/3.),duration) if faster and 8.<=distance<=14. else (duration,)
                for duration in durations:
                    if duration>15.:continue
                    v=np.array([s['control_reference']['velocity'] for s in own]);a=np.array([s['control_reference']['acceleration'] for s in own])
                    travel=bool(reference.get('continuous') and not near and action in ('translate','expand','contract'))
                    avoidance=bool(travel and (reference.get('terminal_avoidance') or restoring))
                    end_velocity=None
                    if travel:
                        advance=goals[2,:2]-p[2,:2];length=np.linalg.norm(advance)
                        if length>=.20:
                            end_velocity=np.zeros((5,3))
                            end_velocity[:,:2]=advance/length*min(6.,1.5*length/duration)
                    variants=[]
                    if end_velocity is not None:
                        if avoidance and not escaping:variants.append((end_velocity,True))
                        if not restoring:variants.append((end_velocity,False))
                        elif escaping:variants.append((end_velocity,True))
                        if not avoidance:variants.append((end_velocity,True))
                    if end_velocity is None and not avoidance and not restoring:variants.append((None,False))
                    for tail_v,flexible in variants:
                        if direction_index<len(choices) and choices[direction_index][1] is not None:
                            _,member,curve_index=choices[direction_index]
                            curve=curve_reference(intents[member],curve_index,p[2],goals[2])
                        else:
                            curve=np.empty((0,3))
                        peers=peer_predictions(reference,states,group,duration)
                        plans,stats=native.plan(p,v,a,goals,duration,end_velocity=tail_v,flexible=flexible,
                                               obstacles=obstacles,reference=curve,peers=peers,terrain=terrain)
                        if plans is not None and tail_v is not None:
                            try:plans=append_backup(plans)
                            except ValueError:continue
                        if plans is None or not formation_kept(plans,flexible=flexible):continue
                        corridors=stop_clouds(plans,corridors=True)
                        if not tree_corridors_clear(trees,corridors):continue
                        boxes=[outer_box(c) for c in corridors];all_boxes=list(reserved)
                        all_boxes[group*5:group*5+5]=boxes if old_boxes is None else [np.vstack((old,box)) for old,box in zip(old_boxes,boxes)]
                        safe,distance=spatially_separated(all_boxes)
                        if safe:break
                    else:continue
                    stats=dict(stats,minimum_envelope_distance=distance,build_seconds=time.monotonic()-began,source_stamp=now,
                               action=action,wall_escape=escaping,formation_radius=float(radius),target_radius=target_radius,flexible=flexible,
                               nominal_duration=getattr(plans[0],'nominal_duration',plans[0].duration),
                               backup_duration=plans[0].duration-getattr(plans[0],'nominal_duration',plans[0].duration))
                    # Warm-start the next travel solve at this admitted length,
                    # but use its current leader's first direction, not this old
                    # direction index. Departure/scale/landing keep their search.
                    stats.update(intent_token=token,next_cursor=next_cursor,
                                 restart_cursor=1+(next_cursor-2)%len(lengths) if action=='translate' else 0)
                    result=(make_packet(run,2,serial,group,plans,now,envelopes=boxes,corridors=corridors,flexible=flexible),stats)
                    return result
        # A failed contraction at the end of the direction list must not
        # strand future searches beyond every available direction.
        exhausted=direction_index>=len(directions)
        return None,dict(stats or {},intent_token=token,next_cursor=0 if exhausted else 1+(direction_index+1)*len(lengths))
    def update(self,group,states,reference,now,run):
        # All members receive fixed final destinations for YOPO intent. The
        # executing targets are exclusively the jointly accepted polynomials.
        reference=dict(reference,moving_slots=False,paused_members=[])
        reference['targets']=self.model.landing_slots(group).tolist()
        command={'kind':'wait'};own=[states[i] for i in range(group*5,group*5+5)]
        positions=np.array([s['position'] for s in own]);center=positions[2,:2]
        if np.linalg.norm(center-self.departure_goals[group])<2.:self.departed[group]=True
        remaining=np.linalg.norm(center-self.model.ends[group])
        if self.departed[group] and remaining<=40.:self.contracting[group]=True
        other=np.asarray(states[7 if group==0 else 2]['position'])[:2]
        target_radius=COMPACT_RADIUS if self.contracting[group] else min(TRAVEL_RADIUS,max(COMPACT_RADIUS,(np.linalg.norm(center-other)-4.)/2.))
        cap=self.observation_caps[group]
        if cap is not None:
            if np.linalg.norm(center-cap[1])>=5.:self.observation_caps[group]=None
            else:target_radius=min(target_radius,cap[0])
        center_goal=self.model.ends[group] if self.departed[group] else self.departure_goals[group]
        reference.update(terminal_center_goal=np.asarray(center_goal).tolist(),terminal_target_radius=float(target_radius),
                         terminal_contracting=self.contracting[group],terminal_departed=self.departed[group],
                         terminal_force_contract=self.map_retry_count[group]>=8)
        phase='contract' if self.contracting[group] else 'travel' if self.departed[group] else 'depart'
        reference['terminal_intent_token']='%s:%d:%s'%(run,group,phase)
        if self.cursor_tokens[group]!=reference['terminal_intent_token']:
            self.cursor_tokens[group]=reference['terminal_intent_token'];self.cursor[group]=0;self.map_retry_count[group]=0
        reference['terminal_force_contract']=self.map_retry_count[group]>=8
        if not self.departed[group]:
            for i in range(5):reference['targets'][i][:2]=(center_goal+UNIT_STAR[i]*target_radius).tolist()
        packet=self.pending[group]
        if packet is not None and self.started[group] is None and self.pending_token[group]!=reference['terminal_intent_token']:
            self.pending[group]=None;packet=None
        if packet is not None:
            joint=[s.get('terminal_joint',{}) for s in own]
            if self.started[group] is not None:
                command=dict(kind='commit',packet=packet,start=self.started[group])
                if all(s.get('digest')==packet['digest'] and s.get('done') for s in joint) and stationary_states(own):
                    if self.pending_action[group]=='contract_for_observation':
                        self.observation_caps[group]=(shape_state(positions)[0],center.copy())
                        target_radius=min(target_radius,self.observation_caps[group][0])
                        reference['terminal_target_radius']=float(target_radius)
                        if not self.departed[group]:
                            for i in range(5):reference['targets'][i][:2]=(center_goal+UNIT_STAR[i]*target_radius).tolist()
                    self.map_retry_count[group]=0;reference['terminal_force_contract']=False
                    self.pending[group]=None;self.started[group]=None;self.cursor[group]=0
                    command={'kind':'wait'}
                elif now>self.started[group]+.4 and not all(s.get('digest')==packet['digest'] and s.get('committed') for s in joint):
                    raise ValueError('Terminal partial/expired commit; stop within certified boxes')
                elif now>self.started[group]+max(sum(x) for x in packet['durations'])+5.:
                    raise ValueError('Terminal packet did not settle')
            else:
                gate=PacketGate(packet)
                for i,s in enumerate(joint):gate.ack(i,s.get('digest'),s.get('accepted',False),s.get('check_stamp',-1e9))
                # A valid ACK is not necessarily young enough to start the
                # commit round. Leave 150ms inside its existing 350ms lifetime
                # for delivery and the client's final current-map validation.
                # Otherwise continue requesting real checks of the same packet.
                ack_budget=all(now-s.get('check_stamp',-1e9)<=.20 for s in joint)
                if gate.ready(now) and ack_budget and stationary_states(own):
                    safe,distance=spatially_separated(self.reserved(states))
                    if not safe:raise ValueError('Cross-group terminal reservation conflict')
                    # The commit is consumed after the member's current image
                    # fusion and map check. A measured 360ms end-to-end delay
                    # exhausted the old 400ms lead minus the 50ms start margin.
                    # Schedule farther ahead inside the existing 1s window;
                    # client ACK/sensor/map/stop deadlines remain unchanged.
                    self.started[group]=now+.7;self.metrics['committed'][group]+=1
                    command=dict(kind='commit',packet=packet,start=self.started[group])
                elif now-self.requested[group]>2.:
                    self.pending[group]=None;self.cursor[group]=self.pending_next[group];self.metrics['map_rejections'][group]+=1
                    self.map_retry_count[group]+=1
                    reference['terminal_force_contract']=self.map_retry_count[group]>=8
                else:command=dict(kind='request',packet=packet)
        if self.pending[group] is None:
            future=self.work[group]
            if future is not None and future.done():
                self.work[group]=None
                packet,stats=future.result();self.metrics['native_calls']+=1;self.metrics['last_optimizer']=stats
                stale_intent=bool(stats is not None and stats.get('intent_token')!=reference['terminal_intent_token'])
                if stale_intent:packet=None
                if packet is not None:
                    # Another group may reserve space while this worker runs.
                    reserved=self.reserved(states);reserved[group*5:group*5+5]=[np.asarray(x) for x in packet['envelopes']]
                    safe,distance=spatially_separated(reserved)
                    if safe:
                        # Creation is publication time. Source time stays in
                        # diagnostics; every member must still match the old
                        # start PVA and ACK its CURRENT observed map.
                        packet=make_packet(run,2,packet['serial'],group,packet_paths(packet),now,envelopes=packet['envelopes'])
                        self.pending[group]=packet;self.pending_token[group]=reference['terminal_intent_token']
                        self.pending_next[group]=stats['next_cursor'];self.pending_action[group]=stats['action']
                        self.requested[group]=now;command=dict(kind='request',packet=packet)
                        old=self.metrics['minimum_envelope_distance'];self.metrics['minimum_envelope_distance']=distance if old is None else min(old,distance)
                    else:self.cursor[group]+=1
                else:self.cursor[group]=0 if stale_intent else (stats or {}).get('next_cursor',self.cursor[group]+1)
            if self.pending[group] is None and self.work[group] is None:
                ready=stationary_states(own) and shape_state(positions)[1]<=.25 and all(s.get('terminal_joint',{}).get('scanned') for s in own)
                ready=ready and all(-.05<=now-s['stamp']<=.3 for s in states.values())
                ready=ready and all(s.get('terminal_joint',{}).get('intent') is not None and -.05<=now-s['terminal_joint']['intent']['stamp']<=.7 for s in own)
                ready=ready and all(s['terminal_joint']['intent'].get('token')==reference['terminal_intent_token'] for s in own)
                if ready:
                    self.serial+=1
                    self.work[group]=self.pool.submit(self.build,group,copy.deepcopy(states),copy.deepcopy(reference),run,self.serial,now,self.reserved(states),self.cursor[group])
        reference['terminal_joint']=command
        return reference
