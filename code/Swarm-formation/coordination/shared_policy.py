# Central direct-state scheduling; no peer trajectory or scene oracle.
import math
import numpy as np
CFG=dict(odom_max_age=.5,stamp_skew=.1,lease=.2,period=.05,min_distance=2.,max_distance=2000.,tracking_margin=1.5,latency=.85,brake_acc=1.2,max_initial_acc=3.,jerk=3.,release_margin=5.,conflict_radius=8.,upper_pause=1800.,upper_resume=1600.,command_speed=10.5)
def stop_bound(speed,settled=False):
 speed=float(speed)
 if speed<.15 and settled:return .5
 ramp=(CFG['max_initial_acc']+CFG['brake_acc'])/CFG['jerk'];peak=speed+CFG['max_initial_acc']*ramp
 return speed*(CFG['latency']+ramp)+.5*CFG['max_initial_acc']*ramp*ramp+peak*peak/(2*CFG['brake_acc'])
def segment_distance(p,a,b):
 p,a,b=map(lambda x:np.asarray(x,float),(p,a,b));delta=b-a;t=np.clip(np.dot(p-a,delta)/max(1e-12,np.dot(delta,delta)),0,1);return float(np.linalg.norm(p-a-t*delta))
class Policy:
 def __init__(self):self.yielding=False;self.conflict=None;self.declared_conflict=None;self.upper=None;self.previous={};self.previous_state={};self.stopped_since={};self.serial=0
 def decide(self,states,goals,now):
  self.serial+=1;d=dict(seq=self.serial,stamp=float(now),allow={0:False,1:False},hard_fault=None,reason='awaiting_global_state',yielding=self.yielding)
  try:
   if set(states)!={0,1}:return d
   if states[0].get('frame')!=states[1].get('frame'):raise ValueError('global_frame_mismatch')
   for i in (0,1):
    s=states[i];p=np.asarray(s['p'],float);v=np.asarray(s['v'],float);stamp=float(s['stamp'])
    if p.shape!=(3,) or v.shape!=(3,) or not np.isfinite(np.r_[p,v,stamp]).all() or s.get('frame') not in ('world','map'):raise ValueError('invalid_global_state')
    if not -.05<=now-stamp<=CFG['odom_max_age']:raise ValueError('global_state_stale')
    if stamp<self.previous.get(i,-math.inf):raise ValueError('global_time_reversed')
    if np.linalg.norm(v)>15.001:raise ValueError('actual_speed_limit')
    previous=self.previous_state.get(i)
    if previous is not None:
     dt=stamp-previous['stamp'];expected=(np.asarray(previous['v'])+v)*.5*dt
     if np.linalg.norm(p-np.asarray(previous['p'])-expected)>.2+1.5*dt*dt:raise ValueError('global_position_jump')
    if np.linalg.norm(v)<.15:self.stopped_since.setdefault(i,now)
    else:self.stopped_since.pop(i,None)
    self.previous[i]=stamp;self.previous_state[i]=dict(stamp=stamp,p=p.copy(),v=v.copy())
   if abs(states[0]['stamp']-states[1]['stamp'])>CFG['stamp_skew']:raise ValueError('global_time_skew')
   p=[np.array(states[i]['p'],float) for i in (0,1)];v=[np.array(states[i]['v'],float) for i in (0,1)];rel=p[0]-p[1];dv=v[0]-v[1];distance=float(np.linalg.norm(rel));speeds=[float(np.linalg.norm(x)) for x in v];bounds=[stop_bound(speeds[i],now-self.stopped_since.get(i,now)>=.5) for i in (0,1)];guard=CFG['min_distance']+CFG['tracking_margin']+sum(bounds)
   if not CFG['min_distance']<=distance<=CFG['max_distance']:raise ValueError('actual_distance_outside_2_2000')
   horizon=max(8.,max(speeds)/CFG['brake_acc']+2.);tcpa=float(np.clip(-np.dot(rel,dv)/max(1e-12,np.dot(dv,dv)),0,horizon));closest=float(np.linalg.norm(rel+dv*tcpa))
   near=distance<guard or tcpa>.05 and closest<guard
   if near and not self.yielding:self.yielding=True;self.conflict=np.asarray(self.declared_conflict,float).copy() if self.declared_conflict is not None else (p[0]+v[0]*tcpa+p[1]+v[1]*tcpa)/2
   winner_out=self.conflict is not None and np.linalg.norm(p[0]-self.conflict)>CFG['conflict_radius'] and np.dot(p[0]-self.conflict,np.asarray(goals.get(0,p[0]))-self.conflict)>=0
   if self.yielding and not near and winner_out and distance>guard+CFG['release_margin'] and speeds[1]<.15:self.yielding=False
   lane_distance=None;lane_threshold=CFG['min_distance']+CFG['tracking_margin']+bounds[1]
   allow={0:True,1:not self.yielding};reason='clear'
   if self.yielding:
    reason='fixed_priority_1_yield';goal0=np.asarray(goals.get(0,p[0]),float);lane_distance=segment_distance(p[1],p[0],goal0)
    if lane_distance<lane_threshold:allow[0]=False;reason='both_brake_until_loser_stopped_outside_winner_lane'
   upper_trigger=min(CFG['upper_pause'],CFG['max_distance']-sum(bounds)-10.)
   if distance>=upper_trigger and self.upper is None:
    outward=[float(np.dot(v[0],rel)),float(np.dot(v[1],-rel))];self.upper=int(np.argmax(outward))
   if self.upper is not None:
    if distance<CFG['upper_resume']:self.upper=None
    else:allow[self.upper]=False;reason='upper_distance_leader_yield'
   d.update(allow=allow,reason=reason,yielding=self.yielding,near=near,winner_out=bool(winner_out),lane_distance=lane_distance,lane_threshold=lane_threshold,settled_ages={i:now-self.stopped_since.get(i,now) for i in (0,1)},distance=distance,predicted_closest=closest,time_to_closest=tcpa,stop_bounds=bounds,near_trigger=guard,upper_trigger=upper_trigger,upper_paused=self.upper,conflict=None if self.conflict is None else self.conflict.tolist(),source_stamps={i:states[i]['stamp'] for i in (0,1)},positions=[x.tolist() for x in p],velocities=[x.tolist() for x in v])
  except (ValueError,TypeError,KeyError,OverflowError) as exc:d.update(hard_fault=str(exc),reason=str(exc),allow={0:False,1:False})
  return d

class LandingResource:
 def __init__(self):self.owner=None;self.next_owner=0
 def acquire(self,i):
  if self.owner is not None or i!=self.next_owner:raise ValueError('L_resource_not_available')
  self.owner=i
 def release(self,i,position,L):
  if self.owner!=i or np.linalg.norm(np.asarray(position)[:2]-np.asarray(L))<=25:raise ValueError('L_owner_not_clear')
  self.owner=None;self.next_owner=i+1
