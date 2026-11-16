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
from geometry_msgs.msg import Point
from ten_core import *
from formation_core import min_separation,graph_gradient
from ten_cloud import xyz
ROOT=Path((str(portable_paths.SCENE)))
BAND={'CLIMB','LINE','CRUISE','CRUISED'}
TERMINAL={'IDLE','FAILED','HOLD','LANDED'}
class Coordinator:
 def __init__(self):
  rospy.init_node('ten_coordinator');self.lock=threading.RLock();self.states={};self.scene={};self.scene_stamp=0.;self.ground={}
  self.stage='IDLE';self.reason='Ready for ten-group command';self.targets=None;self.start_points=None;self.end_points=None
  self.u=np.zeros(2);self.duration=np.ones(2);self.dwell=None;self.auto=False;self.altitude=150.;self.cruise_distance=30.
  self.run_id='';self.report=None;self.log=None;self.directory=None;self.last=time.monotonic();self.started=self.last;self.progress=self.last
  self.refs=[rospy.Publisher('/swarm10/group_%d/reference'%g,String,queue_size=1) for g in range(2)]
  self.tasks=[rospy.Publisher('/drone_%d/task'%i,String,queue_size=1) for i in range(10)]
  self.status=rospy.Publisher('/swarm10/status',String,queue_size=1,latch=True);self.markers=rospy.Publisher('/swarm10/markers',MarkerArray,queue_size=1)
  for i in range(10):
   rospy.Subscriber('/drone_%d/mission/status'%i,String,self.state,callback_args=i,queue_size=1)
   rospy.Subscriber('/drone_%d/local_ground'%i,PointCloud2,self.ground_cb,callback_args=i,queue_size=1,buff_size=1000000)
  rospy.Subscriber('/scene/status',String,self.scene_cb,queue_size=1);rospy.Subscriber('/swarm10/command',String,self.command,queue_size=1)
  rospy.Timer(rospy.Duration(.2),self.tick);rospy.on_shutdown(self.shutdown)
 def state(self,msg,i):
  try:
   with self.lock:self.states[i]=json.loads(msg.data)
  except ValueError:pass
 def scene_cb(self,msg):
  try:
   with self.lock:self.scene=json.loads(msg.data);self.scene_stamp=rospy.Time.now().to_sec()
  except ValueError:pass
 def ground_cb(self,msg,i):
  try:
   p=xyz(msg)
   with self.lock:self.ground[i]=(msg.header.stamp.to_sec(),p)
  except ValueError:pass
 def send(self,op,**kw):
  for pub in self.tasks:pub.publish(String(json.dumps(dict(op=op,_formation_internal=True,**kw))))
 def stop_refs(self):
  for pub in self.refs:pub.publish(String('{"op":"stop"}'))
 def shutdown(self):
  if self.stage!='IDLE':self.stop_refs()
  if self.log:self.log.close()
 def ready(self,now):
  if self.scene.get('drone_count')!=10 or now-self.scene_stamp>1.2:return 'Unity ten-agent scene absent/stale'
  if self.scene.get('collision_events',0) or self.scene.get('bounds_events',0):return 'Unity collision/bounds event'
  for i in range(10):
   s=self.states.get(i)
   if s is None or not -.1<=now-s.get('stamp',0)<=1.2:return 'Member %d stale'%i
   if s.get('failure') or s.get('controller_fault') or s.get('formation',{}).get('interrupted'):return 'Member %d fault/interruption'%i
   if not -.1<=s.get('odom_age',999)<=.5 or not 0<=s.get('plan_age',999)<=.8:return 'Member %d odometry/planner stale'%i
   for key in ('position','velocity'):
    value=np.asarray(s.get(key,[]))
    if value.shape!=(3,) or not np.isfinite(value).all():return 'Invalid member state'
   if not -.1<=now-s.get('map',{}).get('ground_stamp',0)<=.6:return 'Member %d local ground stale'%i
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
  self.stop_refs();self.targets=None;self.set_stage('FAILED');self.reason=reason
  if self.report is not None:self.report['errors'].append(reason);self.report['passed']=False;self.save()
 def begin(self,stage,dest):
  actual=self.positions();dest=np.asarray(dest,float)
  if dest.shape!=(10,3) or not np.isfinite(dest).all():raise ValueError('Invalid ten targets')
  if not np.all((dest[:,0]>1)&(dest[:,0]<1999)&(dest[:,1]>-499)&(dest[:,1]<499)&(dest[:,2]>.2)&(dest[:,2]<175)):raise ValueError('Target outside task bounds')
  clearance=1.9 if stage=='ASSEMBLE' else 2.
  if segment_clearance(actual,dest)<clearance-1e-6:raise ValueError('Formation swept paths too close')
  self.start_points=actual;self.end_points=dest;self.targets=actual.copy();self.u=np.zeros(2)
  self.duration=np.array([max(4.,1.5*np.linalg.norm((dest-actual)[5*g:5*g+5,:2],axis=1).max()/.5,1.5*np.abs((dest-actual)[5*g:5*g+5,2]).max()/.4) for g in range(2)])
  self.set_stage(stage)
 def shape_target(self,name,z=None):
  actual=self.positions();out=[]
  for g in range(2):
   old=actual[g*5:g*5+5];center=old.mean(0)
   if z is not None:center[2]=z
   out.append(assign(old,formation(name)+center))
  return np.concatenate(out)
 def descend(self):
  dest=self.shape_target('final');now=rospy.Time.now().to_sec()
  for i in range(10):
   stamp,p=self.ground.get(i,(0,np.empty((0,3))))
   if not -.1<=now-stamp<=1.2 or not len(p):raise ValueError('Local landing ground stale/missing')
   distances=np.linalg.norm(p[:,:2]-dest[i,:2],axis=1);j=int(np.argmin(distances))
   if distances[j]>.75 or not np.isfinite(p[j,2]):raise ValueError('Landing slot ground not measured')
   dest[i,2]=p[j,2]+3.
  self.begin('DESCEND',dest)
 def command(self,msg):
  with self.lock:
   try:
    data=json.loads(msg.data);op=data['op'];now=rospy.Time.now().to_sec()
    if op in ('hold','cancel'):
     self.stop_refs();self.targets=None;self.auto=False;self.set_stage('HOLD');return
    problem=self.ready(now)
    if problem:raise ValueError(problem)
    if op in ('start','assemble'):
     if self.stage!='IDLE':raise ValueError('Restart simulation before another run')
     if any(s['armed'] or not s['contact'] for s in self.states.values()):raise ValueError('All ten must be grounded/disarmed')
     altitude=float(data.get('altitude',150));distance=float(data.get('distance',30))
     if not np.isfinite([altitude,distance]).all() or not 12<=altitude<=170 or not 5<=distance<=1500:raise ValueError('Altitude 12..170 ABS; distance 5..1500')
     self.altitude=altitude;self.cruise_distance=distance;self.auto=op=='start';self.run_id=time.strftime('%Y%m%dT%H%M%S')+'-'+uuid.uuid4().hex[:6]
     self.directory=ROOT/'Reports/ten-runs'/self.run_id;self.directory.mkdir(parents=True);(ROOT/'Runtime/ten-last-run').write_text(str(self.directory))
     self.report=dict(run_id=self.run_id,scope='module tests; full mission unverified until LANDED',altitude_abs=altitude,cruise_distance=distance,band='group centroids 30..80 m',stages=[],errors=[],passed=False)
     self.log=open(self.directory/'telemetry.jsonl','w',buffering=1);self.send('takeoff',height_mode='ABS',height=12,speed=.8);self.set_stage('TAKEOFF')
    elif op=='climb' and self.stage=='ASSEMBLED':self.begin('CLIMB',self.shape_target('line',self.altitude))
    elif op=='cruise' and self.stage=='LINE':self.begin('CRUISE',self.targets+[self.cruise_distance,0,0])
    elif op=='descend' and self.stage in ('LINE','CRUISED'):self.descend()
    elif op=='land' and self.stage=='FINAL':self.stop_refs();self.targets=None;self.stop_stamp=now;self.set_stage('PRELAND')
    else:raise ValueError('Command incompatible with stage '+self.stage)
   except (ValueError,KeyError,TypeError) as exc:self.reason='Command rejected: '+str(exc)
 def tick(self,_):
  with self.lock:
   try:self.step()
   except Exception as exc:rospy.logerr('Ten coordinator %r',exc);self.fail(repr(exc))
 def step(self):
  now=rospy.Time.now().to_sec();mono=time.monotonic();dt=mono-self.last;self.last=mono
  active=self.stage not in TERMINAL
  problem=self.ready(now)
  if active and problem:self.fail(problem)
  actual=self.positions() if len(self.states)==10 else None
  velocity=np.array([self.states[i]['velocity'] for i in range(10)]) if actual is not None else None
  spacing=min_separation(actual) if actual is not None else None
  centers=center_distance(actual) if actual is not None else None
  if self.stage not in TERMINAL and spacing is not None:
   if spacing<(1.5 if self.stage in ('TAKEOFF','ASSEMBLE') else 2.):self.fail('Actual body separation breached')
   if self.stage in BAND and not center_band(actual):self.fail('Group center distance outside 30..80m')
  if self.stage=='PRELAND':
   acknowledged=all(s['stamp']>=self.stop_stamp and s['phase']=='HOLD' and not s.get('formation',{}).get('active',True) and np.linalg.norm(s['velocity'])<.15 for s in self.states.values())
   if mono-self.started>.8 and acknowledged:self.send('land');self.set_stage('LANDING')
   elif mono-self.started>15:self.fail('Prelanding stop acknowledgement timed out')
  if self.stage=='TAKEOFF':
   if all(s['phase']=='HOLD' and s['armed'] for s in self.states.values()):
    dest=np.r_[assign(actual[:5],formation('assembly')+[45,-75,12],1.9),assign(actual[5:],formation('assembly')+[45,-25,12],1.9)]
    self.begin('ASSEMBLE',dest)
   elif mono-self.started>100:self.fail('Takeoff timeout')
  errors=None;blocked=[]
  if self.targets is not None and self.stage not in TERMINAL:
   if any(not s['armed'] or s['contact'] for s in self.states.values()):self.fail('Unexpected contact/disarm in formation')
   else:
    errors=np.linalg.norm(actual-self.targets,axis=1)
    for g in range(2):
     sl=slice(5*g,5*g+5);block=any(self.states[i].get('formation',{}).get('blocked') for i in range(5*g,5*g+5))
     if self.stage in BAND and not predicted_band(actual,velocity):block=True
     new=advance(self.u[g],dt,self.duration[g],float(errors[sl].max()),block)
     candidate=self.targets.copy();candidate[sl]=interpolate(self.start_points[sl],self.end_points[sl],new)
     if self.stage in BAND and not center_band(candidate,35,75):new=self.u[g];block=True
     if new>self.u[g]:self.progress=mono
     self.u[g]=new;self.targets[sl]=interpolate(self.start_points[sl],self.end_points[sl],new);blocked.append(bool(block))
    settled=bool(np.all(self.u>=1)&(np.max(np.linalg.norm(actual-self.end_points,axis=1))<.65)&(np.max(np.linalg.norm(velocity,axis=1))<.25))
    if settled:
     if self.dwell is None:self.dwell=mono
     if mono-self.dwell>=2:
      mapping={'ASSEMBLE':'ASSEMBLED','CLIMB':'LINE','CRUISE':'CRUISED','DESCEND':'FINAL'}
      if self.stage in mapping:
       self.set_stage(mapping[self.stage])
       if self.auto:
        op={'ASSEMBLED':'climb','LINE':'cruise','CRUISED':'descend','FINAL':'land'}[self.stage]
        self.command(String(json.dumps(dict(op=op))))
        if self.reason.startswith('Command rejected'):self.fail(self.reason)
    else:self.dwell=None
    if self.stage in ('ASSEMBLE','CLIMB','CRUISE','DESCEND') and (mono-self.progress>60 or mono-self.started>max(self.duration)*4+120):self.fail('No safe formation progress')
    if self.targets is not None:
     for g,pub in enumerate(self.refs):
      sl=slice(g*5,g*5+5);targets=self.targets[sl].copy()
      if self.stage!='ASSEMBLE':
       correction=-2*graph_gradient(actual[sl],targets);correction-=correction.mean(0)
       correction*=min(1.,.08/max(np.linalg.norm(correction,axis=1).max(),1e-9));targets+=correction
      pub.publish(String(json.dumps(dict(run_id=self.run_id,stage=self.stage,stamp=now,targets=targets.tolist(),speed=.8))))
  if self.stage=='LANDING':
   if all(s['phase']=='LANDED' and s['contact'] and not s['armed'] for s in self.states.values()):
    self.set_stage('LANDED');self.report['passed']=True;self.save()
   elif mono-self.started>120:self.fail('Landing timeout; local clearance prevents completion')
  status=dict(stamp=now,stage=self.stage,reason=self.reason,run_id=self.run_id,band_active=self.stage in BAND,center_distance=centers,min_body_distance=spacing,
              group_progress=self.u.tolist(),group_blocked=blocked,max_slot_error=None if errors is None else float(errors.max()),altitude_abs=self.altitude,cruise_distance=self.cruise_distance)
  self.status.publish(String(json.dumps(status)));self.visualize(actual,status)
  if self.log:
   self.log.write(json.dumps(dict(status=status,states=self.states,targets=None if self.targets is None else self.targets.tolist()))+'\n')
   if self.stage in TERMINAL and mono-self.started>15:self.log.close();self.log=None
 def visualize(self,actual,status):
  if actual is None:return
  out=MarkerArray()
  for g in range(2):
   p=actual[g*5:g*5+5];m=Marker();m.header.frame_id='world';m.header.stamp=rospy.Time.now();m.ns='ten_groups';m.id=g;m.type=Marker.LINE_LIST;m.action=Marker.ADD;m.pose.orientation.w=1;m.scale.x=.08;m.color.a=.8;m.color.g=1 if g==0 else .3;m.color.b=1 if g==1 else .2
   for i in range(5):
    for j in range(i):m.points.extend([Point(*map(float,p[i])),Point(*map(float,p[j]))])
   out.markers.append(m)
  t=Marker();t.header=m.header;t.ns='ten_groups';t.id=2;t.type=Marker.TEXT_VIEW_FACING;t.action=Marker.ADD;t.pose.orientation.w=1;t.pose.position=Point(*map(float,actual.mean(0)+[0,0,7]));t.scale.z=1.5;t.color.r=t.color.g=t.color.b=t.color.a=1
  t.text='2 x 5 | %s\ncenters %.2f m | band %s\nABS target %.0f m'%(self.stage,status['center_distance'],status['band_active'],self.altitude);out.markers.append(t);self.markers.publish(out)
if __name__=='__main__':Coordinator();rospy.spin()
