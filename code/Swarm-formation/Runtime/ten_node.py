#!/usr/bin/env python3
"""Two five-agent phase engines; center-band supervisor, no control publisher."""
import portable_paths
import json,time,uuid,threading
from pathlib import Path
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point,PoseStamped
from ten_core import *
from formation_core import min_separation,graph_gradient
from ten_cloud import xyz
from ten_terrain import TerrainStages
from planning_mode import input_problem
from sensor_inbox import LatestMeasurement
from swarm_state import fresh as state_fresh, parse_swarm_state
from sensor_channels import group_cloud_topic, group_members

# Matches mission_node: the ten-drone ground stream runs at ~5 Hz.
GROUND_AGE_LIMIT=max(.6,3.5*.2)
ROOT=Path((str(portable_paths.SCENE)))
BAND={'LINE','CRUISE','CRUISED'}
TERMINAL={'IDLE','FAILED','HOLD','LANDED','STAR_HOVER'}
class Coordinator(TerrainStages):
 def __init__(self):
  rospy.init_node('ten_coordinator');self.lock=threading.RLock();self.states={};self.scene={};self.scene_stamp=0.;self.ground={}
  self.clock=lambda:rospy.Time.now().to_sec()
  self.state_inboxes=[LatestMeasurement() for _ in range(10)]
  self.ground_inboxes=[LatestMeasurement() for _ in range(10)]
  self.ascent_speed=float(rospy.get_param('~ascent_speed',6.));self.descent_speed=float(rospy.get_param('~descent_speed',6.))
  phase_duration(np.zeros((10,3)),np.zeros((10,3)),'CLIMB',self.ascent_speed,self.descent_speed)
  self.stage='IDLE';self.reason='Ready for ten-group command';self.targets=None;self.start_points=None;self.end_points=None
  self.u=np.zeros(2);self.duration=np.ones(2);self.dwell=None;self.auto=False;self.altitude=150.;self.cruise_distance=30.
  self.landing_center=None;self.center_tracks=[[],[]];self.nav_stamp=None;self.stop_clearance=2.
  self.init_terrain()
  self.live=None;self.live_errors=0;self.live_received=0.
  self.executions={};self.execution_inboxes=[LatestMeasurement() for _ in range(10)]
  self.coordination={};self.coordination_inboxes=[LatestMeasurement() for _ in range(10)]
  self.run_id='';self.report=None;self.log=None;self.directory=None;self.last=time.monotonic();self.started=self.last;self.progress=self.last
  self.refs=[rospy.Publisher('/swarm10/group_%d/reference'%g,String,queue_size=1) for g in range(2)]
  self.tasks=[rospy.Publisher('/drone_%d/task'%i,String,queue_size=1) for i in range(10)]
  self.status=rospy.Publisher('/swarm10/status',String,queue_size=1,latch=True);self.markers=rospy.Publisher('/swarm10/markers',MarkerArray,queue_size=1,latch=True)
  for i in range(10):
   rospy.Subscriber('/drone_%d/mission/status'%i,String,self.state,callback_args=i,queue_size=1)
   rospy.Subscriber('/drone_%d/local_ground'%i,PointCloud2,self.ground_cb,callback_args=i,queue_size=1,buff_size=1000000)
   if getattr(self,'cloud_mission',False):
    rospy.Subscriber('/drone_%d/execution/status'%i,String,self.execution_cb,callback_args=i,queue_size=1,tcp_nodelay=True)
    rospy.Subscriber('/drone_%d/terminal/coordination'%i,String,self.coordination_cb,callback_args=i,queue_size=1,tcp_nodelay=True)
  for group in range(2):
   # The group-shared leader cloud feeds every member's freshness/landing
   # ground check; per-aircraft clouds remain for the two leaders.
   rospy.Subscriber(group_cloud_topic(group),PointCloud2,self.group_ground_cb,callback_args=group,queue_size=1,buff_size=1000000)
  rospy.Subscriber('/scene/status',String,self.scene_cb,queue_size=1);rospy.Subscriber('/swarm10/command',String,self.command,queue_size=1)
  # Compact 50 Hz state channel (position/velocity/collision/bounds). It is
  # independent of the 5 Hz evidence message and of the depth/cloud streams.
  rospy.Subscriber('/swarm10/state',String,self.live_state_cb,queue_size=1,tcp_nodelay=True)
  rospy.Subscriber('/move_base_simple/goal',PoseStamped,self.nav_goal,queue_size=1)
  rospy.Timer(rospy.Duration(.05 if getattr(self,'fast_terminal_checks',False) else .1),self.tick);rospy.on_shutdown(self.shutdown)
 def state(self,msg,i):
  inboxes=getattr(self,'state_inboxes',None)
  if inboxes is not None:inboxes[i].put(msg);return
  with self.lock:self.accept_state(msg,i)
 def accept_state(self,msg,i):
  try:
   value=json.loads(msg.data);execution=getattr(self,'executions',{}).get(i)
   if execution and execution['stamp']>=value.get('execution',{}).get('stamp',-1e9):value['execution']=execution
   self.merge_coordination(value,getattr(self,'coordination',{}).get(i))
   self.states[i]=value
  except ValueError:pass
 def drain_states(self):
  for i,inbox in enumerate(getattr(self,'state_inboxes',())):
   msg=inbox.take()
   if msg is not None:self.accept_state(msg,i)
 def execution_cb(self,msg,i):
  # Never queue behind the planning/report lock. Safety reads the newest
  # independent executor ACK at its next gate, without re-dating it.
  self.execution_inboxes[i].put(msg)
 def coordination_cb(self,msg,i):self.coordination_inboxes[i].put(msg)
 @staticmethod
 def merge_coordination(state,value):
  if (value and value.get('run_id')==state.get('planning_run_id') and value.get('epoch')==2
      and value['stamp']>=state.get('terminal_coordination_stamp',state['stamp'])):
   state['terminal_joint']=dict(state.get('terminal_joint',{}),**value['joint'])
   state['terminal_coordination_stamp']=value['stamp']
   if value.get('execution',{}).get('stamp',-1e9)>=state.get('execution',{}).get('stamp',0.):
    state['execution']=value['execution']
 def drain_coordination(self):
  for i,inbox in enumerate(getattr(self,'coordination_inboxes',())):
   msg=inbox.take()
   if msg is None:continue
   try:
    value=json.loads(msg.data);state=self.states.get(i)
    if (state is None or value.get('run_id')!=state.get('planning_run_id') or value.get('epoch')!=2
        or not np.isfinite(value['stamp']) or value['stamp']<self.coordination.get(i,{}).get('stamp',-1e9)):continue
    self.coordination[i]=value;self.merge_coordination(state,value)
   except (ValueError,KeyError,TypeError):pass
 def drain_execution(self):
  for i,inbox in enumerate(getattr(self,'execution_inboxes',())):
   msg=inbox.take()
   if msg is None:continue
   try:
    value=json.loads(msg.data);stamp=value['stamp']
    if not np.isfinite(stamp) or stamp<self.executions.get(i,{}).get('stamp',-1e9):continue
    self.executions[i]=value
    state=self.states.get(i)
    if state is not None and stamp>=state.get('execution',{}).get('stamp',-1e9):state['execution']=value
   except (ValueError,KeyError,TypeError):pass
 def live_state_cb(self,msg):
  try:self.live=parse_swarm_state(msg.data,rospy.Time.now().to_sec())
  except (ValueError,TypeError):self.live_errors+=1
 def live_state(self,now):
  """Newest compact state sample, or None when absent/expired."""
  live=getattr(self,'live',None)
  return live if live is not None and state_fresh(live,now) else None
 def scene_cb(self,msg):
  try:
   with self.lock:self.scene=json.loads(msg.data);self.scene_stamp=rospy.Time.now().to_sec()
  except ValueError:pass
 def ground_cb(self,msg,i):
  # A callback must keep draining TCP while the coordinator builds/serializes
  # a reference. Decode only the newest immutable message at the next gate.
  self.ground_inboxes[i].put(msg)
 def group_ground_cb(self,msg,group):
  for i in group_members(group):self.ground_inboxes[i].put(msg)
 def drain_ground(self):
  for i,inbox in enumerate(getattr(self,'ground_inboxes',())):
   msg=inbox.take()
   if msg is None:continue
   try:
    stamp=msg.header.stamp.to_sec()
    if msg.header.frame_id!='world' or not np.isfinite(stamp):continue
    if stamp<self.ground.get(i,(-float('inf'),None))[0]:continue
    if msg.height<1 or msg.width<1 or msg.point_step<12 or msg.row_step<msg.width*msg.point_step:continue
    if len(msg.data)<(msg.height-1)*msg.row_step+msg.width*msg.point_step:continue
    p=xyz(msg)
    self.ground[i]=(stamp,p)
   except (ValueError,TypeError):pass
 def send(self,op,**kw):
  for pub in self.tasks:pub.publish(String(json.dumps(dict(op=op,_formation_internal=True,**kw))))
 def stop_refs(self):
  for pub in self.refs:pub.publish(String('{"op":"stop"}'))
 def shutdown(self):
  if self.stage!='IDLE':self.stop_refs()
  if self.log:self.log.close()
 def ready(self,now):
  from depth_source import depth_source
  self.drain_states()
  self.drain_execution()
  self.drain_coordination()
  # Mailbox parsing may yield while newer executor reports arrive. The
  # complete immutable snapshot must be checked against a later real clock,
  # not the timer-entry date. Never use message dates to manufacture time.
  if getattr(self,'clock',None) is not None:now=float(self.clock())
  self.ready_stamp=now
  if self.scene.get('depth_source','unity')!=depth_source():return 'Unity/ROS depth source mismatch'
  if self.scene.get('drone_count')!=10 or now-self.scene_stamp>1.2:return 'Unity ten-agent scene absent/stale'
  if self.scene.get('collision_events',0) or self.scene.get('bounds_events',0):return 'Unity collision/bounds event'
  live=self.live_state(now)
  if live is not None and (live['collision'] or live['bounds']):return 'Unity live collision/bounds event'
  for i in (tuple(getattr(self,'active',())) or tuple(range(10))):  # inactive bodies in a fixed ten-body scene cannot age out
   s=self.states.get(i)
   if s is None:return 'Member %d stale'%i
   s.pop('kinematic_stamp',None)
   if s.get('failure') or s.get('controller_fault') or s.get('formation',{}).get('interrupted'):return 'Member %d fault/interruption'%i
   if i in getattr(self,'relay_followers',()) and self.stage not in ('IDLE','TAKEOFF','AIR_ASSEMBLE','ASSEMBLED'):
    # Phase 2 followers replay delayed leader commands; their planner, ground
    # and terminal checks are intentionally idle until their cut.
    if not -.1<=now-s.get('stamp',0)<=1.2:return 'Member %d stale'%i
    continue
   terminal=getattr(self,'terminal_coordinator',None)
   commits=() if terminal is None else (terminal.active[i//5],terminal.offers[i//5])
   world=getattr(getattr(self,'model',None),'hashes',{}).get('world')
   measured=None
   if getattr(self,'cloud_mission',False) and self.planning_gate.mode=='YOPO':
    from supervisor_observation import measured_execution
    measured=measured_execution(s,i,now,world,commits)
    if measured is not None:self.states[i]=s=measured
   if not -.1<=now-s.get('stamp',0)<=1.2 and measured is None:return 'Member %d stale'%i
   execution=getattr(self,'executions',{}).get(i)
   if execution and -.05<=now-execution['stamp']<=.35:
    if execution.get('error'):return 'Member %d executor fault: %s'%(i,execution['error'])
    if not -.1<=now-execution.get('odom_stamp',-1e9)<=.5:return 'Member %d executor odometry stale'%i
   if not -.1<=s.get('odom_age',999)<=.5:return 'Member %d odometry stale'%i
   for key in ('position','velocity'):
    value=np.asarray(s.get(key,[]))
    if value.shape!=(3,) or not np.isfinite(value).all():return 'Invalid member state'
   # Check current raw input directly. Integrated map time is paired with the
   # status timestamp that observed it, not aged twice through status transport.
   raw_stamp=self.ground.get(i,(0,None))[0]
   map_stamp=s.get('map',{}).get('ground_stamp') or 0.
   from static_perception import state_perception_allowed
   protected=measured is not None or state_perception_allowed(s,now,world,commits)
   if not 0<=s.get('plan_age',999)<=.8 and not protected:return 'Member %d planner stale'%i
   # Compare the integrated map stamp with the current clock, exactly like the
   # raw ground above. Using the status stamp here double-counted status
   # transport delay and made a healthy 5 Hz ground stream look 0.6 s stale.
   ground_stale=(not -.1<=now-raw_stamp<=GROUND_AGE_LIMIT or not -.1<=now-map_stamp<=GROUND_AGE_LIMIT)
   if ground_stale and not protected:
    # A short ground gap holds the formation; only a persistent loss fails
    # the run. The aircraft zero their own commands on stale data, so this
    # grace never keeps a body moving on old measurements.
    if self.stage=='RUNNING':
     if getattr(self,'ground_grace',None) is None:self.ground_grace=[now,i]
     if now-self.ground_grace[0]<=5.:continue
    return 'Member %d local ground stale (raw %.3fs, integrated %.3fs)'%(i,now-raw_stamp,now-map_stamp)
   self.ground_grace=None
   if getattr(self,'cloud_mission',False):
    mode=self.planning_gate.mode
    if mode=='CLOUD' and (s.get('planner_mode')!='CLOUD' or any(s.get(k)!=0 for k in ('image_callbacks','network_inferences'))):return 'Member %d unexpected image planning/model activity in cloud mode'%i
    problem=input_problem(s,raw_stamp,now,mode)
    if problem and not (protected and mode=='YOPO' and problem in ('local ground stale','depth missing/stale','planner stale') and s.get('yopo_ready',False)):return 'Member %d %s'%(i,problem)
   else:
    ages=s.get('frame_ages',{})
    if set(ages)!=set(('front','up','down')) or not all(np.isfinite(v) and -.1<=v<=.6 for v in ages.values()):return 'Member %d sensor missing/stale'%i
  return None
 def positions(self):return np.array([self.states[i]['position'] for i in range(10)])
 def set_stage(self,stage):
  self.stage=stage;self.reason=stage;self.started=time.monotonic();self.progress=self.started;self.dwell=None
  if self.report is not None:self.report['stages'].append(dict(stage=stage,stamp=rospy.Time.now().to_sec()));self.save()
  rospy.loginfo('SWARM10_STAGE %s',stage)
 def save(self):
  if self.report is not None:
   p=self.directory/'report.json';q=p.with_suffix('.tmp');q.write_text(json.dumps(self.report,indent=2));q.replace(p)
 def fail(self,reason):
  if self.stage=='FAILED':return
  if self.stage not in ('HOLD','STAR_HOVER'):
   self.stop_clearance=1.5 if self.stage in ('TAKEOFF','ASSEMBLE','REFORM') else 2.
  self.stop_refs();self.send('hold',_phase_token='fault:'+uuid.uuid4().hex);self.targets=None;self.set_stage('FAILED');self.reason=reason
  if self.report is not None:self.report['errors'].append(reason);self.report['passed']=False;self.save()
 def nav_goal(self,msg):
  with self.lock:
   # RViz is a two-command gate; its click position never replaces point.json.
   if self.stage=='ASSEMBLED':
    self.auto=True
    self.command(String('{"op":"climb"}'));return
   # An active click always brakes, even if the new coordinates are unusable.
   if self.stage not in ('IDLE','HOLD','FAILED','LANDED','STAR_HOVER'):
    self.command(String('{"op":"hold"}'));return
   if self.stage!='IDLE':
    self.reason='Task ended; reset simulation before starting again';return
   self.command(String('{"op":"assemble"}'))
 def begin(self,stage,dest):
  actual=self.positions();dest=np.asarray(dest,float)
  if dest.shape!=(10,3) or not np.isfinite(dest).all():raise ValueError('Invalid ten targets')
  if not np.all((dest[:,0]>-8469.625)&(dest[:,0]<1528.375)&(dest[:,1]>-6572.4375)&(dest[:,1]<3425.5625)&(dest[:,2]>-32.850982666015625)&(dest[:,2]<859.8286743164062)):raise ValueError('Target outside task bounds')
  origin=self.targets.copy() if self.targets is not None else actual.copy()
  clearance=1.9 if stage in ('ASSEMBLE','REFORM') else 2.
  if min(segment_clearance(actual,dest),segment_clearance(origin,dest))<clearance-1e-6:raise ValueError('Formation swept paths too close')
  self.start_points=origin;self.end_points=dest;self.targets=origin.copy();self.u=np.zeros(2)
  # Constant reference slope across modules; controller acceleration/jerk limits remain active.
  seconds=phase_duration(origin,dest,stage,getattr(self,'ascent_speed',6.),getattr(self,'descent_speed',6.))
  self.duration=np.full(2,seconds);self.set_stage(stage)
 def low_square(self):
  actual=self.positions();center=actual.mean(0);center[0]+=4.
  dest=mission_shape(actual,'square',float(np.mean([s['ground'] for s in self.states.values()]))+2.,9.,center[:2])
  return self.local_heights(dest,2.)
 def local_heights(self,dest,agl):
  dest=dest.copy();now=rospy.Time.now().to_sec()
  for i in range(10):
   stamp,p=self.ground.get(i,(0,np.empty((0,3))))
   if not -.1<=now-stamp<=.6 or not len(p):raise ValueError('Local landing ground stale/missing')
   distances=np.linalg.norm(p[:,:2]-dest[i,:2],axis=1);j=int(np.argmin(distances))
   if distances[j]>.75:
    # Sharing current local measurements does not introduce a global map.
    fresh=[q for stamp,q in self.ground.values() if -.1<=now-stamp<=.6 and len(q)]
    if fresh:
     p=np.concatenate(fresh);distances=np.linalg.norm(p[:,:2]-dest[i,:2],axis=1);j=int(np.argmin(distances))
   if distances[j]>.75 or not np.isfinite(p[j,2]):raise ValueError('Landing slot ground not measured')
   dest[i,2]=p[j,2]+agl
  return dest
 def climb(self):
  # Finish square -> straight line at low altitude before the line ascends.
  self.route_climb,self.route_cruise,self.route_end=route_waypoints(self.targets[:,:2].mean(0),self.landing_center,self.cruise_distance)
  if self.report is not None:
   self.report['route']=dict(climb_end=self.route_climb.tolist(),cruise_end=self.route_cruise.tolist(),landing_center=self.route_end.tolist());self.save()
  self.begin('REFORM',mission_shape(self.targets,'line',float(self.targets[:,2].mean()),9.))
 def start_ascent(self):
  self.begin('CLIMB',mission_shape(self.targets,'line',self.altitude,50.,self.route_climb,line_spacing=20. if getattr(self,'terminal_enabled',False) else 10.))
 def cruise(self):
  dest=self.targets.copy()
  dest[:,:2]+=self.route_cruise-dest[:,:2].mean(0)
  self.begin('CRUISE',dest)
 def end_hold(self,stage='HOLD'):
  if self.stage not in ('HOLD','FAILED','STAR_HOVER'):
   self.stop_clearance=1.5 if self.stage in ('TAKEOFF','ASSEMBLE','REFORM') else 2.
  self.stop_refs();self.send('hold',_phase_token='stop:'+uuid.uuid4().hex);self.targets=None;self.auto=False;self.set_stage(stage)
 def shape_target(self,name,z=None):
  actual=self.positions();out=[]
  for g in range(2):
   old=actual[g*5:g*5+5];center=old.mean(0)
   if z is not None:center[2]=z
   out.append(assign(old,formation(name)+center))
  return np.concatenate(out)
 def descend(self):
  # Forward descent and line-to-star deformation share the same progress.
  dest=mission_shape(self.targets,'star',float(self.targets[:,2].mean()),9.8,self.route_end)
  dest=self.local_heights(dest,2.)
  if np.any(dest[:,2]>=self.targets[:,2]-.1):raise ValueError('Landing terrain requires a higher cruise altitude')
  self.begin('DESCEND',dest)
 def command(self,msg):
  with self.lock:
   try:
    data=json.loads(msg.data);op=data['op']
    if op in ('hold','cancel'):
     self.end_hold();return
    self.drain_ground();now=rospy.Time.now().to_sec()
    problem=self.ready(now)
    now=getattr(self,'ready_stamp',now)
    if problem:raise ValueError(problem)
    if op in ('start','assemble'):
     if self.stage!='IDLE':raise ValueError('Restart simulation before another run')
     if any(s['armed'] or not s['contact'] for s in self.states.values()):raise ValueError('All ten must be grounded/disarmed')
     altitude=float(data.get('altitude',150));distance=float(data.get('distance',30))
     if not np.isfinite([altitude,distance]).all() or not 12<=altitude<=800 or not 5<=distance<=1500:raise ValueError('Altitude 12..800 ABS; distance 5..1500')
     self.landing_center=self.preset_descent.copy() if getattr(self,'terminal_enabled',False) else landing_goal('world',data['x'],data['y']) if 'x' in data and 'y' in data else None
     if not getattr(self,'diagnostic_start_low',False):route_waypoints(self.positions()[:,:2].mean(0)+[4.,0.],self.landing_center,distance)
     self.altitude=altitude;self.cruise_distance=distance;self.auto=op=='start';self.run_id=time.strftime('%Y%m%dT%H%M%S')+'-'+uuid.uuid4().hex[:6]
     self.directory=ROOT/'Reports/ten-runs'/self.run_id;self.directory.mkdir(parents=True);(ROOT/'Runtime/ten-last-run').write_text(str(self.directory))
     self.report=dict(run_id=self.run_id,scope='fixed mission through AGL2 transit and ground landing' if getattr(self,'terminal_enabled',False) else 'continuous mission; completion requires STAR_HOVER',altitude_abs=altitude,cruise_distance=distance,band='assembly <=10 m; climb expands to 50 m; cruise 30..80 m',stages=[],errors=[],passed=False)
     if getattr(self,'diagnostic_start_low',False):self.report['scope']='DIAGNOSTIC ONLY: local low transit, excludes full-route acceptance'
     self.log=open(self.directory/'telemetry.jsonl','w',buffering=1);self.send('takeoff',height_mode='AGL',height=2,speed=2.);self.set_stage('TAKEOFF')
    elif op=='climb' and self.stage=='ASSEMBLED':self.climb()
    elif op=='cruise' and self.stage=='LINE':self.cruise()
    elif op=='descend' and self.stage in ('LINE','CRUISED'):self.descend()
    elif op=='land' and self.stage=='FINAL':self.start_preland()
    else:raise ValueError('Command incompatible with stage '+self.stage)
   except (ValueError,KeyError,TypeError) as exc:self.reason='Command rejected: '+str(exc)
 def tick(self,_):
  with self.lock:
   try:self.step()
   except Exception as exc:rospy.logerr('Ten coordinator %r',exc);self.fail(repr(exc))
 def step(self):
  self.drain_ground()
  now=rospy.Time.now().to_sec();mono=time.monotonic();dt=min(.25,max(0.,mono-self.last));self.last=mono
  active=self.stage not in TERMINAL
  problem=self.ready(now)
  now=getattr(self,'ready_stamp',now)
  if active and problem:self.fail(problem)
  actual=self.positions() if len(self.states)==10 else None
  velocity=np.array([self.states[i]['velocity'] for i in range(10)]) if actual is not None else None
  spacing=min_separation(actual) if actual is not None else None
  centers=center_distance(actual) if actual is not None else None
  if self.stage not in TERMINAL and spacing is not None:
   if spacing<(1.5 if self.stage in ('TAKEOFF','ASSEMBLE','REFORM') else 2.):self.fail('Actual body separation breached')
   if self.stage in ('ASSEMBLE','REFORM') and centers>10.01:self.fail('Assembly centers exceed 10m')
   if self.stage in BAND and not center_band(actual):self.fail('Group center distance outside 30..80m')
  self.preland_tick(mono)
  if self.stage=='TAKEOFF':
   diagnostic=getattr(self,'diagnostic_start_low',False)
   if all(s['armed'] and not s['contact'] and s.get('agl',0)>.75 and (not diagnostic or np.linalg.norm(s['velocity'])<.2) for s in self.states.values()):
    if getattr(self,'diagnostic_start_low',False):
     self.end_points=self.local_heights(mission_shape(actual,'star',2.,9.8,self.preset_descent),2.);self.begin_approach()
    else:self.begin('ASSEMBLE',self.low_square())
   elif mono-self.started>30:self.fail('Takeoff timeout')
  errors=None;blocked=[]
  if self.targets is not None and self.stage not in TERMINAL:
   if any(not s['armed'] or s['contact'] for s in self.states.values()):self.fail('Unexpected contact/disarm in formation')
   else:
    errors=np.linalg.norm(actual-self.targets,axis=1)
    blocks=[any(self.states[i].get('formation',{}).get('blocked') for i in range(5*g,5*g+5)) for g in range(2)]
    if self.stage in BAND and not predicted_band(actual,velocity):blocks=[True,True]
    # Close formations share one progress clock; do not advance one group through its neighbor.
    rate=terminal_progress_scale(self.stage,self.start_points,self.end_points,float(self.u.min()),float(self.duration.max()))
    horizon=min(1.,float(self.u.min())+min(dt,.25)/float(self.duration.max()))
    tangent=mission_tangent(self.stage,self.start_points,self.end_points,horizon)/float(self.duration.max())
    rate=min(rate,4./max(np.linalg.norm(tangent[:,:2],axis=1).max(),1e-9))
    # High-rate commands tolerate bounded tracking lag; body separation is checked independently.
    nominal=np.linalg.norm((self.end_points-self.start_points)/float(self.duration.max()),axis=1).max()
    tracking_limit=min(3.,max(.25,.4*(min_separation(self.targets)-2.))) if self.stage in ('CLIMB','CRUISE','DESCEND') else 1.5
    tracking_error=formation_tracking_error(actual,self.targets,tracking_limit) if self.stage in ('CLIMB','CRUISE','DESCEND') else float(errors.max())
    new=advance(float(self.u.min()),dt*rate,float(self.duration.max()),tracking_error,any(blocks),max_error=tracking_limit)
    candidate=mission_reference(self.stage,self.start_points,self.end_points,new)
    if segment_clearance(self.targets,candidate)<(1.9 if self.stage in ('ASSEMBLE','REFORM') else 2.)-1e-6:new=float(self.u.min());blocks=[True,True]
    if self.stage in BAND and not center_band(candidate,35,75):new=float(self.u.min());blocks=[True,True]
    moving=new>self.u.min()
    if moving:self.progress=mono
    old_targets=self.targets.copy();old_stage=self.stage
    self.u[:]=new;self.targets=mission_reference(self.stage,self.start_points,self.end_points,new);blocked=blocks
    step_velocity=(self.targets-old_targets)/max(min(dt,.25),1e-6)
    arrived=bool(new>=1 and np.max(np.linalg.norm(actual-self.end_points,axis=1))<.65)
    descent_settled=arrived and np.max(np.linalg.norm(velocity,axis=1))<.2 and np.max(np.linalg.norm(actual-self.end_points,axis=1))<.25 and all(abs(s.get('agl',0)-2)<.2 for s in self.states.values())
    if self.stage=='DESCEND' and getattr(self,'terminal_enabled',False) and not descent_settled and all(s.get('agl',999)<15 for s in self.states.values()):self.begin_approach()
    if arrived:
     if self.stage=='ASSEMBLE':
      self.set_stage('ASSEMBLED')
      if self.auto:self.climb()
     elif self.stage=='REFORM':self.start_ascent()
     elif self.stage=='CLIMB':
      self.set_stage('LINE')
      if self.auto:self.cruise()
     elif self.stage=='CRUISE':
      self.set_stage('CRUISED')
      if self.auto:self.descend()
     elif self.stage=='DESCEND' and np.max(np.linalg.norm(velocity,axis=1))<.2 and np.max(np.linalg.norm(actual-self.end_points,axis=1))<.25 and all(abs(s.get('agl',0)-2)<.2 for s in self.states.values()):
      if getattr(self,'terminal_enabled',False):self.begin_terrain()
      else:
       self.end_hold('STAR_HOVER')
       if self.report is not None:self.report['passed']=True;self.save()
    if self.stage in ('ASSEMBLE','REFORM','CLIMB','CRUISE','DESCEND','STAR') and (mono-self.progress>60 or mono-self.started>max(self.duration)*4+120):self.fail('No safe formation progress')
    if self.targets is not None:
     for g,pub in enumerate(self.refs):
      sl=slice(g*5,g*5+5);targets=self.targets[sl].copy()
      ref_velocity=step_velocity[sl] if self.stage==old_stage and self.u[g]<1 and moving and not any(blocked) else np.zeros((5,3))
      pub.publish(String(json.dumps(dict(run_id=self.run_id,stage=self.stage,stamp=now,targets=targets.tolist(),velocity=ref_velocity.tolist(),terminal_z=self.end_points[sl,2].tolist(),speed=5.,safety_pause=any(blocked)))))
  self.approach_tick(now);self.terrain_tick(now)
  if self.stage=='LANDING':
   self.retry_landing(now)
   if all(s['phase']=='LANDED' and s['contact'] and not s['armed'] for s in self.states.values()):
    self.set_stage('LANDED');self.report['passed']=True;self.save()
   elif mono-self.started>120:self.fail('Landing timeout; local clearance prevents completion')
  status=dict(stamp=now,stage=self.stage,reason=self.reason,run_id=self.run_id,band_active=self.stage in BAND,center_distance=centers,min_body_distance=spacing,
              group_progress=self.u.tolist(),group_blocked=blocked,max_slot_error=None if errors is None else float(errors.max()),altitude_abs=self.altitude,cruise_distance=self.cruise_distance,center_ids=list(CENTER_IDS),stop_clearance=getattr(self,'stop_clearance',2.),landing_center=None if self.landing_center is None else self.landing_center.tolist(),terminal_center=getattr(self,'terminal_goal',np.zeros(2)).tolist(),continue_to_terminal=getattr(self,'terminal_enabled',False))
  self.status.publish(String(json.dumps(status)));self.visualize(actual,status)
  if self.log:
   self.log.write(json.dumps(dict(status=status,states=self.states,targets=None if self.targets is None else self.targets.tolist()))+'\n')
   if self.stage in TERMINAL and mono-self.started>15:self.log.close();self.log=None
 def visualize(self,actual,status):
  if actual is None:return
  run=status.get('run_id');now=time.monotonic()
  if run!=getattr(self,'tracks_run',None):self.center_tracks=[[],[]];self.tracks_run=run;self.last_track_publish=-1e9
  # Full flight history is retained, but serialized only once per second.
  # Avoid delaying terrain input callbacks with ever-growing 10 Hz markers.
  for g,i in enumerate(CENTER_IDS):
   history=self.center_tracks[g]
   if not history or np.linalg.norm(actual[i]-history[-1])>.1:history.append(actual[i].copy())
  if now-getattr(self,'last_track_publish',-1e9)<1.:return
  self.last_track_publish=now
  out=MarkerArray()
  for g in range(2):
   p=actual[g*5:g*5+5];m=Marker();m.header.frame_id='world';m.header.stamp=rospy.Time.now();m.ns='ten_groups';m.id=g;m.type=Marker.LINE_LIST;m.action=Marker.ADD;m.pose.orientation.w=1;m.scale.x=.08;m.color.a=.8;m.color.g=1 if g==0 else .3;m.color.b=1 if g==1 else .2
   for i in range(5):
    for j in range(i):m.points.extend([Point(*map(float,p[i])),Point(*map(float,p[j]))])
   out.markers.append(m)
  for g,i in enumerate(CENTER_IDS):
   history=self.center_tracks[g]
   trail=Marker();trail.header=m.header;trail.ns='center_aircraft_paths';trail.id=g;trail.type=Marker.LINE_STRIP;trail.action=Marker.ADD;trail.pose.orientation.w=1;trail.scale.x=.22
   trail.color.r=1. if g==0 else .1;trail.color.g=.45 if g==0 else .65;trail.color.b=.1 if g==0 else 1.;trail.color.a=1.
   trail.points=[Point(*map(float,p)) for p in history];out.markers.append(trail)
  t=Marker();t.header=m.header;t.ns='ten_groups';t.id=2;t.type=Marker.TEXT_VIEW_FACING;t.action=Marker.ADD;t.pose.orientation.w=1;t.pose.position=Point(*map(float,actual.mean(0)+[0,0,7]));t.scale.z=1.5;t.color.r=t.color.g=t.color.b=t.color.a=1
  t.text='2 x 5 | %s\ncenters %.2f m | band %s\nABS target %.0f m'%(self.stage,status['center_distance'],status['band_active'],self.altitude);out.markers.append(t);self.markers.publish(out)
if __name__=='__main__':Coordinator();rospy.spin()
