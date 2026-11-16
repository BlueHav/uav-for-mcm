#!/usr/bin/env python3
import json,time,threading,contextlib,rospy,numpy as np
from std_msgs.msg import String
from single_executor import SingleMission,audit
from shared_policy import CFG
class SharedMission(SingleMission):
 def __init__(self):
  self.central={};self.central_seq=-1;self.central_fault=None;self.last_gate_audit=0.;self.last_gate=None
  self.shape_context=threading.local();self.shaping_control_calls=0;self.shaping_sweep_calls=0
  self.cap_task_generation=0;self.cap_plan_generation=0;self.cap_certificate=None
  super().__init__()
  rospy.Subscriber('/shared_pair/decision',String,self.central_command,queue_size=1,tcp_nodelay=True)
 def central_command(self,msg):
  try:
   d=json.loads(msg.data)
   if int(d['seq'])<=self.central_seq:return
   if not np.isfinite(d['stamp']) or not -.05<=rospy.Time.now().to_sec()-d['stamp']<=CFG['lease']:return
   cap=float(d['speed_cap_3d'][str(self.id)])
   if not np.isfinite(cap) or not 0<=cap<=15:return
   self.central_seq=int(d['seq']);self.central=d
  except (ValueError,TypeError,KeyError,OverflowError):pass
 def permission(self,decision=None):
  d=self.central if decision is None else decision;age=rospy.Time.now().to_sec()-d.get('stamp',-1e10)
  hard=d.get('hard_fault') or ('central_lease_expired' if age>CFG['lease'] else None)
  if d.get('allow',{}).get(str(self.id),False):
   try:
    cap=float(d['speed_cap_3d'][str(self.id)])
    if not np.isfinite(cap) or not 0<=cap<=15:raise ValueError('invalid cap')
   except (ValueError,TypeError,KeyError,OverflowError):hard=hard or 'central_speed_cap_invalid'
  if hard and self.armed and not self.central_fault:self.central_fault=hard
  return bool(not self.central_fault and not hard and d.get('allow',{}).get(str(self.id),False)),self.central_fault or hard or d.get('reason','awaiting_central'),age
 def cap_snapshot(self):
  d=self.central;allowed,reason,age=self.permission(d)
  return dict(cap=float(d['speed_cap_3d'][str(self.id)]) if allowed else 0.,seq=d.get('seq'),stamp=d.get('stamp'),cap_generation=d.get('cap_generation',{}).get(str(self.id)),allowed=allowed,reason=reason,age=age)
 @contextlib.contextmanager
 def shaping_scope(self,source):
  previous=getattr(self.shape_context,'snapshot',None)
  snapshot=self.cap_snapshot();snapshot['source']=source;self.shape_context.snapshot=snapshot
  snapshot['task_generation']=self.cap_task_generation
  if source=='control':snapshot['certificate']=self.cap_certificate
  try:yield snapshot
  finally:self.shape_context.snapshot=previous
 def shape_desired(self,target):
  snapshot=getattr(self.shape_context,'snapshot',None)
  if snapshot is None:
   self.central_fault=self.central_fault or 'missing_frozen_speed_cap';return np.zeros(3)
  # Recheck the original permit timestamp at actual use, after any intervening lock wait.
  if snapshot['source']=='control' and rospy.Time.now().to_sec()-(snapshot['stamp'] or -1e10)>CFG['lease']:
   self.central_fault=self.central_fault or 'central_lease_expired';return np.zeros(3)
  value=np.asarray(target,float).copy();norm=float(np.linalg.norm(value))
  raw_target=value.copy();raw_norm=norm
  if not np.isfinite(value).all():
   self.central_fault=self.central_fault or 'nonfinite_desired_command';return np.zeros(3)
  effective_cap=snapshot['cap'];waiting=None
  if snapshot['source']=='control':
   if snapshot['task_generation']!=self.cap_task_generation:waiting='task_changed_during_control'
   elif self.failure or self.stop_pending or self.braking or not snapshot['allowed'] or time.monotonic()-self.last_plan>.6 or rospy.Time.now().to_sec()-self.odom_stamp>.5:value[:]=0.
   elif getattr(self,'terminal_client',None) is not None and getattr(self,'planning_mode','YOPO')=='YOPO' and self.phase in ('TERRAIN_FOLLOW','END_HOLD'):pass
   elif getattr(self,'cloud_planner',None) is not None and getattr(self,'planning_mode','YOPO')=='CLOUD' and (self.five.reference or {}).get('stage')=='CRUISE':pass
   elif self.count==10 and getattr(self.five,'active',False) and getattr(self.five,'safety_paused',False):value[:]=0.
   elif self.phase in ('CRUISE','TERRAIN_FOLLOW','GOTO','END_APPROACH','FORMATION'):
    certificate=snapshot['certificate']
    if certificate is None or certificate['task_generation']!=self.cap_task_generation or time.monotonic()-certificate['monotonic']>.6:
     waiting='awaiting_fresh_cap_sweep'
    else:
     # Continue the last completely checked candidate, never a half-built next plan.
     candidate=np.asarray(certificate['desired'],float).copy()
     if self.mode=='AGL' and self.phase in ('TERRAIN_FOLLOW','END_APPROACH'):candidate[2]=value[2]
     if snapshot['cap']<certificate['cap']-1e-9 and np.linalg.norm(candidate)>snapshot['cap']+1e-9:
      waiting='lower_cap_requires_new_sweep'
     else:
      value=candidate;effective_cap=min(snapshot['cap'],certificate['cap'])
      snapshot=dict(snapshot,certified_plan_generation=certificate['plan_generation'],certified_cap=certificate['cap'],certificate_stamp=certificate['stamp'])
   elif norm>snapshot['cap']+1e-9 and snapshot['allowed']:
    waiting='binding_cap_requires_phase_certificate'
  if waiting:value[:]=0.;self.cap_waiting_reason=waiting
  else:self.cap_waiting_reason=None
  norm=float(np.linalg.norm(value));shaped=value*min(1.,effective_cap/max(norm,1e-12))
  source=snapshot['source'];key='shaping_'+source+'_calls';setattr(self,key,getattr(self,key,0)+1)
  proof=dict(snapshot,effective_cap=effective_cap,waiting=waiting,raw_target=raw_target.tolist(),raw_norm=raw_norm,desired_before=value.tolist(),desired_after=shaped.tolist(),norm_before=norm,norm_after=float(np.linalg.norm(shaped)))
  setattr(self,'last_'+source+'_shaping',proof)
  return shaped
 def final_sweep(self,p,v):
  with self.shaping_scope('sweep') as snapshot:
   desired=self.desired.copy();generation=self.cap_plan_generation;task_generation=self.cap_task_generation
   try:result=super().final_sweep(p,v)
   except BaseException:
    self.cap_certificate=None;raise
   if result is True and task_generation==self.cap_task_generation and not self.failure:
    self.cap_certificate=dict(cap=snapshot['cap'],cap_generation=snapshot['cap_generation'],task_generation=task_generation,plan_generation=generation,desired=desired.tolist(),stamp=rospy.Time.now().to_sec(),monotonic=time.monotonic())
   else:self.cap_certificate=None
   return result
 def task(self,msg):
  with self.maplock,self.lock:
   self.cap_task_generation+=1;self.cap_certificate=None
   return super().task(msg)
 def fail(self,reason):
  super().fail(reason)
  if self.failure:self.cap_certificate=None
 def horizontal_plan(self,p,v):
  self.cap_plan_generation+=1
  allowed,reason,age=self.permission()
  if not allowed:
   self.desired[:]=0;self.reason='Central scheduling brake: '+str(reason);self.progress_stamp=time.monotonic();self.progress_p=p.copy();return
  snapshot=self.cap_snapshot();old_speed=self.speed;self.speed=min(old_speed,snapshot['cap']);self.last_planner_cap_snapshot=dict(snapshot,requested_task_speed=old_speed,applied_planner_speed=self.speed)
  try:super().horizontal_plan(p,v)
  finally:self.speed=old_speed
 def vertical_plan(self,p,v):
  allowed,reason,age=self.permission()
  if not allowed:self.desired[:]=0;self.reason='Central scheduling brake: '+str(reason);return
  super().vertical_plan(p,v)
 def control(self,event):
  allowed,reason,age=self.permission()
  if self.central_fault and self.armed and not self.failure:
   # Set the hard latch without synchronous map/NPZ I/O in the command timer.
   self.failure='Central safety: '+self.central_fault;self.auto=False;self.hold=self.p.copy();self.set_phase('FAULT_HOLD',self.failure)
  # Count1 baseline does not own a separate persistent stop_pending latch.
  # This original final branch zeroes all axes after AGL feed-forward as well.
  self.stop_pending=not allowed
  with self.shaping_scope('control'):super().control(event)
  now=time.monotonic();gate=(allowed,reason,self.central.get('cap_generation',{}).get(str(self.id)),getattr(self,'cap_waiting_reason',None))
  if self.p is not None and (gate!=self.last_gate or now-self.last_gate_audit>.5):
   self.last_gate=gate;self.last_gate_audit=now;audit('central_gate_actual_control',allowed=allowed,reason=reason,lease_age=age,seq=self.central_seq,position=self.p.tolist(),velocity=self.v.tolist(),reference_velocity=self.refv.tolist(),reference_acceleration=self.refa.tolist(),failure=self.failure,speed_cap_shaping=getattr(self,'last_control_shaping',None),planner_speed_cap=getattr(self,'last_planner_cap_snapshot',None),control_hook_calls=self.shaping_control_calls,sweep_hook_calls=self.shaping_sweep_calls)
SharedMission();rospy.spin()
