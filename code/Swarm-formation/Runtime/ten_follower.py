"""Ten-mode adapter: five-member references, ten-body safety, center-only band."""
import os
import json
import math
from contextlib import nullcontext
import numpy as np
import rospy
from std_msgs.msg import String
from formation_follower import FormationFollower
from formation_core import validate_reference,peer_safe
from ten_core import center_band,predicted_band
from mission_core import smooth_velocity
from local_approach import plan_approach

class TenFollower(FormationFollower):
 def __init__(self,mission):
  self.m=mission;self.reference=None;self.peers={};self.active=False
  self.run_id=None;self.blocked=False;self.note='waiting for two-group start';self.applied=0;self.interrupted=False;self.group_status=None;self.suspended=False;self.safety_paused=False
  self.group_id=mission.id//5;self.member_id=mission.id%5
  rospy.Subscriber('/swarm10/group_%d/reference'%self.group_id,String,self.receive,queue_size=1)
  rospy.Subscriber('/swarm10/status',String,self.group,queue_size=1)
  for i in range(10):
   if i!=mission.id:
    suffix='/execution/peer_state' if getattr(mission,'cloud_mission',False) else '/mission/status'
    rospy.Subscriber('/drone_%d'%i+suffix,String,self.peer,callback_args=i,queue_size=1,tcp_nodelay=True)
 def validate(self,ref,now):
  if getattr(getattr(self,'m',None),'cloud_mission',False):
   m=self.m
   # The planning-mode channel establishes ownership before the first
   # formation reference arrives. It also carries subsequent epoch changes.
   run=m.handoff.run_id
   if not run or ref.get('planner_mode')!=m.planning_mode or ref.get('planner_epoch')!=m.planner_epoch or ref.get('run_id')!=run:raise ValueError('Stale planning ownership')
  point=ref.get('workflow')=='point-v1'
  from cloud_config import reference_speed
  limit=reference_speed(ref) if point else 5.
  if ref.get('moving_merge'):
   if not point or ref.get('planner_mode')!='CLOUD' or ref.get('stage')!='DESCEND' or ref.get('leader_speed')!=16.:
    raise ValueError('Invalid moving merge ownership or leader speed')
  points=validate_reference(ref,now,clearance=1.9 if ref.get('stage') in ('ASSEMBLE','REFORM') else 2.,max_speed=limit)
  velocity=np.asarray(ref.get('velocity',np.zeros((5,3))),float)
  if velocity.shape!=(5,3) or not np.isfinite(velocity).all() or np.max(np.linalg.norm(velocity[:,:2],axis=1))>(limit+.01 if point else 4.01) or np.max(np.abs(velocity[:,2]))>6.01:raise ValueError('Invalid reference velocity')
  if point and 'trail_directions' in ref:
   directions=np.asarray(ref['trail_directions'],float)
   if directions.shape!=(5,3) or not np.isfinite(directions).all() or not np.all(np.abs(np.linalg.norm(directions,axis=1)-1.)<.01):raise ValueError('Invalid trail directions')
  if 'trail_paths' in ref:
   paths=ref['trail_paths']
   if len(paths)!=5:raise ValueError('Invalid trail paths')
   for path in paths:
    a=np.asarray(path,float)
    if len(a) and (a.ndim!=2 or a.shape[1]!=3 or len(a)>2000 or not np.isfinite(a).all()):raise ValueError('Invalid measured trail path')
  terminal=np.asarray(ref.get('terminal_z',points[:,2]),float)
  if terminal.shape!=(5,) or not np.isfinite(terminal).all():raise ValueError('Invalid terminal altitude')
  if point and ref.get('stage')=='ASSEMBLE' and 'agl' in ref:
   agl=float(ref['agl'])
   if not np.isfinite(agl) or agl<=0.:raise ValueError('Invalid square assembly AGL')
  if ref.get('stage')=='TERRAIN':
   agl=float(ref.get('agl',float('nan')))
   if not (2.<=agl<=5. if point else agl==2.):raise ValueError('Terrain reference AGL outside mission range')
  return points
 def receive(self,msg):
  self.receive_locked(msg)
 def receive_locked(self,msg):
  try:ref=json.loads(msg.data)
  except (ValueError,TypeError):return
  if ref.get('op')=='stop':
   # Stop changes the reference and phase together with the planner's lock.
   with getattr(self.m,'maplock',nullcontext()):
    was_suspended=getattr(self,'suspended',False);self.suspended=True
    self.active=False;self.reference=None
    if not was_suspended and self.m.armed and not self.m.failure and self.m.phase not in ('LAND','LANDED'):
     self.m.task(String(json.dumps(dict(op='hold',_formation_internal=True,_reference_stop=True))))
   return
  if getattr(self,'suspended',False) or (self.group_status or {}).get('stage') in ('PRELAND','LANDING','LANDED','HOLD','FAILED'):return
  previous=self.reference
  super().receive(msg)
  client=getattr(self.m,'terminal_client',None)
  if self.reference is not previous and client is not None and hasattr(client,'note_reference'):
   client.note_reference(self.reference.get('terminal_joint'),self.reference['stamp'],rospy.Time.now().to_sec())
 def prepare(self):
  m=self.m
  ref=self.reference
  if not self.active or ref is None or getattr(self,'suspended',False):return
  try:points=self.validate(ref,rospy.Time.now().to_sec())
  except (ValueError,KeyError,TypeError) as exc:self.active=False;m.fail('Ten-agent reference timeout: '+str(exc));return
  if m.failure or not self.active or self.reference is not ref or (self.group_status or {}).get('stage') in ('PRELAND','LANDING','LANDED','HOLD','FAILED'):return
  cruise_peers=None
  if ref.get('workflow')=='point-v1' and (ref.get('stage')=='CRUISE' or ref.get('moving_merge')):
   now=rospy.Time.now().to_sec()
   snapshot={i:self.peers.get(i) for i in range(10) if i!=m.id}
   missing=[i for i,s in snapshot.items() if s is None or not -.1<=now-s.get('stamp',0)<=.6]
   if missing:
    # Never drop a delayed peer from the stop corridor while continuing flight.
    self.active=False;self.blocked=True;m.braking=True;m.desired[:]=0.
    m.fail('Cruise peer state missing/stale: '+','.join(map(str,missing)));return
   cruise_peers=np.array([s['position'] for s in snapshot.values()],float).reshape(-1,3)
  if ref.get('stage')=='TERRAIN':
   m.goal=points[self.member_id].copy();m.mode='AGL';m.height=float(ref.get('agl',2.));m.speed=float(ref['speed'])
   self.applied_terminal_reference=ref
   endpoint=np.asarray(ref.get('fixed_endpoints',points),float)[self.member_id]
   terminal_near=np.linalg.norm(endpoint[:2]-m.p[:2])<15.
   if terminal_near:m.speed=min(m.speed,.6)
   now=rospy.Time.now().to_sec()
   m.map.dynamic_peers=np.array([s['position'] for s in self.peers.values() if -.1<=now-s.get('stamp',0)<=.6],float).reshape(-1,3)
   m.map.dynamic_clearance=2.2 if terminal_near else 2.8
   resume=m.phase=='END_HOLD' and ref.get('workflow')=='point-v1' and (np.linalg.norm(m.goal[:2]-m.p[:2])>.5 or abs(m.p[2]-m.ground-m.height)>.25)
   if m.phase not in ('TERRAIN_FOLLOW','END_HOLD') or resume:
    m.detour=None;m.last_detour=0.;m.desired[:]=0.;m.set_phase('TERRAIN_FOLLOW','Terrain reference; local obstacle avoidance')
   if m.phase=='END_HOLD':self.blocked=False
   if ref.get('terminal_joint') and getattr(m,'terminal_client',None) is not None:
    m.terminal_client.receive(ref['terminal_joint'],now)
   self.applied+=1;return
  if ref.get('stage')=='APPROACH' or (ref.get('workflow')=='point-v1' and (ref.get('stage')=='CRUISE' or ref.get('moving_merge'))):
   now=rospy.Time.now().to_sec();m.map.dynamic_peers=cruise_peers if cruise_peers is not None else np.array([s['position'] for s in self.peers.values() if -.1<=now-s.get('stamp',0)<=.6],float).reshape(-1,3)
   # Cruise spacing must retain enough room for the full delayed, jerk-limited
   # stop checked by final_sweep, including a preceding aircraft that stops.
   m.map.dynamic_clearance=2.8
  else:m.map.dynamic_peers=np.empty((0,3))
  m.goal=points[self.member_id].copy();m.mode='ABS';m.height=float(m.goal[2]);m.speed=float(ref['speed'])
  if ref.get('workflow')=='point-v1' and ref.get('stage')=='ASSEMBLE' and 'agl' in ref:
   m.mode='AGL';m.height=float(ref['agl']);m.goal[2]=m.ground+m.height
  if ref.get('moving_merge') and self.member_id==2:m.speed=float(ref['leader_speed'])
  if ref.get('stage')=='APPROACH':
   token=ref.get('approach_token')
   if token!=getattr(self,'approach_ack',None):self.approach_path=None;self.approach_visited=[];self.approach_candidate=0
   self.approach_ack=token;self.approach_check_stamp=rospy.Time.now().to_sec()
   self.approach_goal_clear=bool(m.map.points_clear([m.goal],radius=.5,check_peers=False)[0])
  if m.phase!='FORMATION':m.hold=m.p.copy();m.set_phase('FORMATION','Group %d %s'%(self.group_id,ref['stage']))
  self.applied+=1
 def plan(self,p,v):
  if (self.reference or {}).get('stage')=='APPROACH':plan_approach(self,p,v);return
  m=self.m;self.blocked=False;self.safety_paused=False;delta=m.goal-p;tolerated_overshoot=False
  if (self.reference or {}).get('safety_pause',False) or self.member_id in (self.reference or {}).get('paused_members',[]):
   self.safety_paused=True;m.desired=np.zeros(3);return
  point=(self.reference or {}).get('workflow')=='point-v1'
  feed=np.asarray((self.reference or {}).get('velocity',np.zeros((5,3))),float)[self.member_id]
  desired=feed+.65*delta-.8*(v-feed)
  if point and self.member_id!=2 and self.reference.get('stage')=='CRUISE' and 'trail_directions' in self.reference:
   tangent=np.asarray(self.reference['trail_directions'],float)[self.member_id,:2]
   norm=np.linalg.norm(tangent)
   if norm>.1:
    tangent=tangent/norm;along=float(np.dot(delta[:2],tangent));lateral=delta[:2]-along*tangent
    if -2.<=along<0. and np.linalg.norm(lateral)<=2. and abs(delta[2])<.5:
     tolerated_overshoot=True
     # A small overshoot of a moving slot is tolerated: retain the forward
     # feed, without position/velocity damping asking for a slowdown or turn.
     # YOPO, final_sweep and the peer guards still control the actual command.
     desired[:2]+=tangent*(np.dot(feed[:2],tangent)-np.dot(desired[:2],tangent))
     if np.linalg.norm(feed[:2])<.02:desired[:2]=0.
  if getattr(m,'planning_mode','YOPO')=='CLOUD' and self.member_id!=2 and self.reference.get('stage')=='CRUISE' and not tolerated_overshoot:
   from leader_following import trail_waypoint
   waypoint=trail_waypoint(self.reference.get('trail_paths',[[]]*5)[self.member_id],p)
   if waypoint is not None:
    target,bend,tangent=waypoint;offset=target-p;distance=np.linalg.norm(offset[:2])
    # Feed follows the measured trail, independent of cross-track position
    # error. A nearby moving slot must not rotate cruise feed sideways.
    local_feed=tangent[:2]*np.linalg.norm(feed[:2])
    desired[:2]=local_feed+.65*offset[:2]-.8*(v[:2]-local_feed)
    if bend:desired[:2]*=min(1.,max(.2,np.sqrt(2*1.2*distance))/max(np.linalg.norm(desired[:2]),1e-9))
  fast=(self.reference or {}).get('stage') in ('CLIMB','CRUISE','DESCEND','ASSEMBLE')
  cruise_limit=float(getattr(m,'speed',6.)) if getattr(m,'planning_mode','YOPO')=='CLOUD' and (self.reference.get('stage')=='CRUISE' or self.reference.get('moving_merge')) else 6.
  if point and self.reference.get('moving_merge') and getattr(m,'planning_mode',None)=='CLOUD':
   # Position feedback must not accelerate the trailing aircraft far past
   # its moving slot's feed speed immediately after the delayed relay cut.
   cruise_limit=min(cruise_limit,max(.8,float(np.linalg.norm(feed[:2]))+.35))
  desired[:2]*=min(1.,((cruise_limit if point else 5.) if fast else 1.4)/max(np.linalg.norm(desired[:2]),1e-9))
  desired[2]=np.clip(desired[2],-7. if fast else -1.8,7. if fast else 1.8)
  if np.linalg.norm(desired[:2])>.05:m.heading=float(np.arctan2(desired[1],desired[0]))
  if (self.reference or {}).get('stage')=='DESCEND' and 'terminal_z' in self.reference:
   remaining=max(0.,float(p[2]-self.reference['terminal_z'][self.member_id]))
   cap=min(7.,.9+.35*max(0.,remaining-3.));desired[2]=np.clip(desired[2],-cap,cap)
   horizontal=min(5.,.6+.15*max(0.,remaining-3.));desired[:2]*=min(1.,horizontal/max(np.linalg.norm(desired[:2]),1e-9))
  # Feed-forward is usable only through observed free space. The existing final
  # swept-volume/controller and peer braking checks still run after this planner.
  samples=p+np.linspace(.1,.8,8)[:,None]*desired
  if point:self.point_plan(p,v,desired)
  elif np.all(m.map.points_clear(samples,radius=.35)):
   m.desired=desired;m.chosen=samples;m.reason='Continuous observed formation corridor'
  else:
   super().plan(p,v)
  self.blocked=bool(np.linalg.norm(delta)>.7 and np.linalg.norm(m.desired)<.05)
  if getattr(m,'cloud_planner',None) is not None and m.planning_mode=='CLOUD' and self.reference.get('stage')=='CRUISE':self.blocked=False
  # When the leader temporarily stops, a follower inside the accepted tolerance
  # may hold safely. Echoing that hold as an obstacle would freeze both forever.
  if tolerated_overshoot and getattr(m,'reason','') in ('Vertical local corridor verified','Point-cloud formation corridor verified'):self.blocked=False
  # A nearby trailing aircraft may turn or settle while its moving slot advances.
  # Freezing the leader for that alone freezes the slot too and creates a
  # stop/reverse/turn loop. Its own zero command and every safety check remain;
  # the coordinator separately stops the leader for tracking error above 5 m.
  if point and self.member_id!=2 and self.reference.get('stage')=='CRUISE' and np.linalg.norm(delta)<=5. and getattr(m,'reason','') in ('Turning sensor toward local route','Braking to settle moving formation slot'):
   self.blocked=False
  # A safety pause brakes everyone; unlike a tracking-lag freeze it must not
  # chase the frozen slots. Do not echo this shared pause as a local blocker.
  if (self.reference or {}).get('safety_pause',False) or self.member_id in (self.reference or {}).get('paused_members',[]):self.safety_paused=True;m.desired[:]=0.
 def point_plan(self,p,v,desired):
  """Give tracking intent to YOPO; never replace its stop or obstacle direction."""
  m=self.m;speed=float(np.linalg.norm(desired[:2]));goal=m.goal.copy()
  if getattr(m,'planning_mode','YOPO') in ('CLOUD','YOPO_WARMUP'):
   planner=getattr(m,'cloud_planner',None)
   climbing=self.reference.get('continuous_climb') is True
   if planner is not None and (climbing or self.reference.get('stage')=='CRUISE') and m.planning_mode=='CLOUD':
    now=rospy.Time.now().to_sec()
    if climbing:planner.update(m,now,climbing=True)
    elif self.reference.get('relay_descent') is not None:planner.update(m,now,descent=self.reference['relay_descent'])
    else:planner.update(m,now)
    reference=planner.reference(rospy.Time.now().to_sec()+.1)
    if reference is not None or not climbing:
     m.desired=np.zeros(3) if reference is None else reference[1].copy()
     m.reason=planner.reason
     if reference is not None:m.chosen=np.array([reference[0]])
     return
    # Maintain a checked forward/upward command while the native worker
    # prepares its first continuation. Keep a valid moving entry boundary.
    desired[:2]*=min(1.,1.5/max(np.linalg.norm(desired[:2]),1e-9))
    desired[2]=np.clip(desired[2],0.,.6)
   samples=p+np.linspace(.05,.8,16)[:,None]*desired
   if np.all(m.map.points_clear(samples,radius=.35)):
    m.desired=desired;m.chosen=samples;m.reason='Point-cloud formation corridor verified'
   else:m.desired=np.zeros(3);m.reason='Point-cloud formation corridor blocked'
   return
  # Negative velocity feedback while the slot is still ahead means braking,
  # not a request to turn the depth camera around and fly a reverse route.
  if np.linalg.norm(v[:2])>.2 and np.dot(desired[:2],v[:2])<0 and np.dot(goal[:2]-p[:2],v[:2])>=0:
   m.heading=m.yaw;m.desired=np.zeros(3);m.reason='Braking to settle moving formation slot';return
  if speed<.02:
   m.hold=goal.copy();m.hold[:2]=p[:2];m.vertical_plan(p,v);m.desired[:2]=0.;return
  # Moving slots may be close even at cruise speed. Supply a look-ahead
  # direction to the local planner, then cap its result by tracking demand.
  m.goal=goal.copy();m.goal[:2]=p[:2]+desired[:2]/speed*max(20.,np.linalg.norm(goal[:2]-p[:2]))
  try:m.horizontal_plan(p,v)
  finally:m.goal=goal
  m.desired[:2]*=min(1.,speed/max(np.linalg.norm(m.desired[:2]),1e-9))
 def plan_terrain(self,p,v):
  m=self.m;self.blocked=False;self.safety_paused=False
  ref=self.reference or {}
  if getattr(m,'terminal_client',None) is not None and getattr(m,'planning_mode',None)=='YOPO' and (not ref.get('terminal_joint') or not self.active or getattr(self,'suspended',False)):
   self.safety_paused=True;m.desired[:]=0.
   return
  if ref.get('terminal_joint') and getattr(m,'terminal_client',None) is not None:
   now=rospy.Time.now().to_sec();m.desired[:]=0.
   if ref.get('safety_pause') or self.member_id in ref.get('paused_members',[]):
    self.safety_paused=True
    if m.terminal_client.committed:m.terminal_client.abort('Shared terminal safety pause')
    return
   if m.terminal_client.executor_stopping:m.terminal_client.monitor(now)
   client=m.terminal_client
   client.propose(p,v,now)
   candidate=client.candidate or {}
   has_local_route=bool((client.intent or {}).get('directions'))
   if client.scanned and 'coverage_heading' in candidate and not candidate.get('committed'):
    m.heading=candidate['coverage_heading']
   elif client.scanned and has_local_route and 'observe_heading' in candidate and not candidate.get('committed'):
    m.heading=candidate['observe_heading']
   elif (client.scanned and has_local_route and client.done and ref.get('terminal_departed')
       and not ref.get('terminal_contracting') and not (client.candidate or {}).get('accepted')):
    # A stationary member may aim its camera at the common continuation
    # while waiting for observations. YOPO and the independent map still
    # determine whether the trajectory may be admitted.
    ahead=np.asarray(ref['terminal_center_goal'],float)-p[:2]
    m.heading=math.atan2(ahead[1],ahead[0])
   self.safety_paused=not client.committed
   return
  if ref.get('safety_pause',False) or self.member_id in ref.get('paused_members',[]):
   self.safety_paused=True;m.desired=np.zeros(3);return
  if ref.get('workflow')=='point-v1' and ref.get('moving_slots',False) and self.member_id!=2:
   feed=np.asarray(ref.get('velocity',np.zeros((5,3))),float)[self.member_id]
   desired=feed+.65*(m.goal-p)-.8*(v-feed)
   desired[:2]*=min(1.,m.speed/max(np.linalg.norm(desired[:2]),1e-9))
   self.point_plan(p,v,desired)
  else:m.horizontal_plan(p,v)
  self.blocked=bool(np.linalg.norm(m.goal[:2]-p[:2])>.5 and np.linalg.norm(m.desired)<.05)
  if (self.reference or {}).get('safety_pause',False) or self.member_id in (self.reference or {}).get('paused_members',[]):self.safety_paused=True;m.desired[:]=0.
 def safety(self,p,v):
  m=self.m
  if not m.armed:return
  s=self.group_status;now=rospy.Time.now().to_sec()
  if self.group_running() and (not s or not -.1<=now-s.get('stamp',0)<=1.2):
   self.active=False;m.desired[:]=0;m.fail('Ten-agent coordinator heartbeat timeout');return
  points=[];velocities=[]
  reachable,_=smooth_velocity(v,getattr(m,'refa',np.zeros(3)),m.desired,.3,vertical_speed=7.,vertical_acc=2.5,vertical_jerk=8.)
  initial_assembly=(s or {}).get('workflow')=='point-v1' and (s or {}).get('stage') in ('TAKEOFF','AIR_ASSEMBLE','ASSEMBLED')
  clearance=1.9 if initial_assembly else 1.0
  low=((self.reference or {}) if (s or {}).get('workflow')=='point-v1' else (s or {})).get('stage') in ('APPROACH','TERRAIN')
  independent_low=(not self.group_running() and m.mode=='AGL' and m.phase in ('TERRAIN_FOLLOW','END_HOLD'))
  low=low or independent_low
  warning=2.2 if low and (independent_low or (s or {}).get('stage')=='TERRAIN') and np.linalg.norm(m.goal[:2]-p[:2])<15. else 2.8
  if (s or {}).get('stage') in ('HOLD','FAILED'):clearance=float(s.get('stop_clearance',clearance))
  env=os.environ.get('YOPO_ACTIVE_IDS','')
  active=tuple(int(v) for v in env.split(',') if v.strip()) or tuple(range(10))
  stage=(s or {}).get('stage')
  for i in active:
   if i==m.id:points.append(p);velocities.append(v);continue
   peer=self.peers.get(i)
   # Callbacks can publish a newer measurement while this loop yields during
   # numerical work. Read the clock AFTER taking the peer reference; an entry
   # clock can falsely label a fresh peer as more than100ms in the future.
   checked=rospy.Time.now().to_sec()
   age=None if peer is None else checked-peer.get('stamp',0)
   if peer is None or not -.1<=age<=1.2:
    self.peer_check_failure=dict(id=i,checked=checked,stamp=None if peer is None else peer.get('stamp'),
                                 age=age,loop_started=now)
    m.desired[:]=0;self.blocked=True;self.note='Peer %d stale'%i
    if self.group_running():m.fail(self.note)
    return
   self.peer_stale_since=None
   other=np.asarray(peer.get('position',[]),float);vel=np.asarray(peer.get('velocity',[]),float)
   if other.shape!=(3,) or vel.shape!=(3,) or not np.isfinite(np.r_[other,vel]).all():m.desired[:]=0;m.fail('Invalid ten-peer state');return
   if peer.get('failure') or peer.get('controller_fault'):m.desired[:]=0;m.fail('Ten-peer %d fault'%i);return
   points.append(other);velocities.append(vel)
   if np.linalg.norm(p-other)<clearance:
    m.desired[:]=0;m.fail('Ten-agent body safety separation breached');return
   if getattr(m,'terminal_client',None) is not None and m.terminal_client.committed and (self.reference or {}).get('terminal_joint'):
    # The joint spatial certificate includes each member not executing or
    # braking at any point. Do not replace it with a frozen-peer prediction.
    continue
   def predictive_clear(a,av,b,bv):
    # Inside the extra warning margin permit only non-closing motion. The
    # actual body limit still applies, including to reference combinations.
    margin=max(clearance,min(warning,float(np.linalg.norm(a-b)))) if low else clearance
    return peer_safe(a,av,b,bv,clearance=margin-1e-8)
   # Phase 2 relay takeoff: the group leader must depart forward+up while the
   # followers fly to its vacated slot. Their deliberate motion may close the
   # projected corridor, so the leader keeps only the actual body limit; the
   # global 2 m check still aborts any real overlap.
   relay_leader=not initial_assembly and bool(getattr(m,'relay_enabled',False)) and self.member_id==2
   safe=(True if relay_leader else (predictive_clear(p,reachable,other,vel) and predictive_clear(p,v,other,vel)))
   if low:
    reference=peer.get('control_reference',{})
    other_p=np.asarray(reference.get('position',other),float);other_v=np.asarray(reference.get('velocity',vel),float)
    own_p=np.asarray(getattr(m,'refp',p),float);own_v=np.asarray(getattr(m,'refv',v),float)
    reference_next,_=smooth_velocity(own_v,getattr(m,'refa',np.zeros(3)),m.desired,.3,vertical_speed=7.,vertical_acc=2.5,vertical_jerk=8.)
    safe=safe and all(predictive_clear(a,av,b,bv) for a,av in ((p,v),(own_p,own_v),(own_p,reference_next)) for b,bv in ((other,vel),(other_p,other_v)))
   if not safe:
    # Do not resume merely because measured velocity temporarily looks safe:
    # the controller may still be executing an accelerating reference.
    if low:m.braking=True
    m.desired[:]=0;self.blocked=True;self.note='Predicted body conflict; braking';return
  if s and s.get('workflow')=='point-v1':
   a=np.asarray(points);b=np.asarray(velocities)
   if np.linalg.norm(a[:5].mean(0)-a[5:].mean(0))>2000.:m.desired[:]=0;m.fail('Actual group centers exceed 2km');return
   future=a+3.*b
   if np.linalg.norm(future[:5].mean(0)-future[5:].mean(0))>2000.:m.desired[:]=0;self.blocked=True;self.note='Predicted group centers exceed 2km; braking';return
  elif s and s.get('band_active'):
   if not center_band(points):m.desired[:]=0;m.fail('Group centers outside 30-80m');return
   if not predicted_band(points,velocities):m.desired[:]=0;self.blocked=True;self.note='Group center prediction outside 35-75m; braking';return
  self.note='Ten bodies checked; distance band uses group centers only'
 def status(self):
  s=super().status();s.update(group_id=self.group_id,member_id=self.member_id)
  if getattr(self,'peer_check_failure',None) is not None:s['peer_check_failure']=self.peer_check_failure
  # Final swept-volume checks run after plan(), so their brake must also
  # reach the shared coordinator before it advances another reference.
  s['blocked']=bool(s['blocked'] or getattr(self.m,'braking',False));return s
