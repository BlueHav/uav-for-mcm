#!/usr/bin/env python3
import json,time,runpy,threading,hashlib
from pathlib import Path
import numpy as np,rospy
B=Path(__file__).parent;log=None;loglock=threading.Lock()
def audit(event,**kw):
 global log
 with loglock:
  if log is None:log=(B/('pair-audit-%d.jsonl'%rospy.get_param('~drone_id',-1))).open('w',buffering=1)
  entry=dict(stamp=rospy.Time.now().to_sec(),event=event);entry.update(kw)
  log.write(json.dumps(entry,allow_nan=True)+'\n')
RUNTIME_OVERRIDE=(B/'mission_runtime_override.py').resolve()
RUNTIME_OVERRIDE_SHA256=hashlib.sha256(RUNTIME_OVERRIDE.read_bytes()).hexdigest()
ns=runpy.run_path(str(RUNTIME_OVERRIDE),run_name='parallel_single_runtime');Original=ns['Mission']
class SingleMission(Original):
 def __init__(self):
  self.profile=False;self.forward_descent=False;self.last_evidence=0;self.last_plan_audit=0;self.group_stage='INIT'
  super().__init__()
  audit('runtime_override_loaded',path=str(RUNTIME_OVERRIDE),sha256=RUNTIME_OVERRIDE_SHA256,hook_present=hasattr(Original,'shape_desired'))
 def task(self,msg):
  raw=json.loads(msg.data)
  with self.maplock,self.lock:
   super().task(msg)
   if raw.get('_forward_descent') and not self.failure:
    self.forward_descent=True;self.descent_entry=np.asarray(raw['_descent_entry'],float);self.descent_end=np.array([raw['x'],raw['y']],float);self.descent_low_z=float(raw['_descent_final_z']);self.goal[2]=self.descent_low_z
    audit('forward_descent_enable',entry=self.descent_entry.tolist(),end=self.descent_end.tolist(),final_z=self.descent_low_z)
   if raw.get('_terminal_profile') and not self.failure:
    self.forward_descent=False;self.profile=True;self.profile_start=np.array(raw['_profile_start'],float);self.profile_end=np.array([raw['x'],raw['y']],float)
    audit('terminal_profile_enable',start=self.profile_start.tolist(),end=self.profile_end.tolist())
   if raw.get('_stage'):self.group_stage=raw['_stage']
 def fail(self,reason):
  observed_wait=any(word in self.reason.lower() for word in ('unknown','unobserved','no observed safe','turning sensor'))
  stopped=np.linalg.norm(self.v)<.15 and np.linalg.norm(self.refv)<.15 and np.linalg.norm(self.refa)<.2
  certified_own_hold=stopped and self.sensor_ready() is None and self.map.points_clear([self.p],radius=.35)[0]
  if reason=='No safe local progress' and observed_wait and certified_own_hold:
   if not hasattr(self,'wait_anchor') or np.linalg.norm(self.p-self.wait_anchor)>.5:self.wait_anchor=self.p.copy();self.wait_started=time.monotonic()
   if time.monotonic()-self.wait_started<120:self.reason='Stopped in observed free body volume; waiting for local corridor';self.progress_stamp=time.monotonic();return
  super().fail(reason)
 def update_mission(self,p,v):
  if self.forward_descent and self.phase=='GOTO' and np.linalg.norm(p[:2]-self.descent_end)<1 and abs(p[2]-self.ground-5)>.15:return
  # Group completion owns terminal touchdown; keep each measured endpoint hover.
  if self.profile and self.phase=='END_HOLD':return
  super().update_mission(p,v)
 def vertical_plan(self,p,v):
  super().vertical_plan(p,v)
  target=self.height if self.mode=='ABS' else self.ground+self.height
  error=target-p[2]
  if self.phase!='LAND':self.desired[2]=np.sign(error)*min(abs(self.desired[2]),.35*abs(error))
 def horizontal_plan(self,p,v):
  if self.forward_descent:
   from assembly_region import single_descent_height
   self.mode,self.height=single_descent_height(self.descent_entry,self.descent_end,self.descent_low_z,p[:2],self.ground)
  if self.profile and self.phase in ('TERRAIN_FOLLOW','END_APPROACH'):
   delta=self.profile_end-self.profile_start;progress=float(np.clip(np.dot(p[:2]-self.profile_start,delta)/np.dot(delta,delta),0,1));self.height=5.-3.*progress
  requested_speed=self.speed
  if self.mode=='ABS':self.speed=min(requested_speed,np.linalg.norm(v[:2])+1.)
  super().horizontal_plan(p,v)
  self.speed=requested_speed
  if self.forward_descent and np.linalg.norm(p[:2]-self.descent_end)<.1:self.desired[2]=np.clip(1.2*(self.ground+5-p[2]),-1.8,1.8)
  now=time.monotonic()
  if now-self.last_plan_audit>.5:
   self.last_plan_audit=now
   audit('local_plan',position=p.tolist(),velocity=v.tolist(),phase=self.phase,stage=self.group_stage,height_mode=self.mode,target_height=self.height,goal=self.goal.tolist(),reason=self.reason,desired=self.desired.tolist(),detour=None if self.detour is None else self.detour.tolist(),chosen=self.chosen[::max(1,len(self.chosen)//30)].tolist(),rejections=dict(self.rejects),network_inferences=self.inferences,frame_stamps={k:frame[0] for k,frame in self.frames.items()})
  terminal_snapshot=self.profile and np.linalg.norm(p[:2]-self.profile_end)<200
  early_snapshot=self.group_stage=='CRUISE_SPLIT_DETOURS' and self.inferences>20 and not getattr(self,'early_evidence_done',False)
  if (terminal_snapshot or early_snapshot) and now-self.last_evidence>2:
   if early_snapshot:self.early_evidence_done=True
   self.last_evidence=now;stamp=rospy.Time.now().to_sec();path=B/'perception'/('drone%d-%.3f.npz'%(self.id,stamp))
   payload={k:frame[1].copy() for k,frame in self.frames.items()};payload.update(position=p.copy(),goal=self.goal.copy(),desired=self.desired.copy(),chosen=self.chosen.copy(),occupied=(np.argwhere(self.map.state==2)+self.map.origin+.5)*self.map.resolution)
   metadata=dict(stamp=stamp,path=str(path),position=p.tolist(),reason=self.reason,detour=None if self.detour is None else self.detour.tolist(),map_stats=self.map.stats(),frame_poses={k:dict(stamp=frame[0],position=np.asarray(frame[2][0]).tolist(),rotation=np.asarray(frame[2][1]).tolist()) for k,frame in self.frames.items()})
   audit('perception_snapshot',**metadata)
   threading.Thread(target=lambda data=payload,target=path:np.savez_compressed(target,**data),daemon=True).start()
