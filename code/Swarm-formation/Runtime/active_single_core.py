"""Single physical aircraft mission stages; no formation or fabricated peers."""
import math
import numpy as np


def scene_problem(scene,active_id,position,velocity,active_ids=None):
    if scene.get('drone_count')!=10:return 'Expected fixed ten-aircraft scene'
    if any(not isinstance(scene.get(k),int) or scene[k]!=0 for k in ('collision_events','bounds_events')):return 'Scene collision/bounds evidence missing or unsafe'
    drones=scene.get('drones',[])
    if len(drones)!=10 or {d.get('id') for d in drones}!=set(range(10)):return 'Incomplete physical scene observations'
    p=np.asarray(position,float);v=np.asarray(velocity,float)
    if p.shape!=(3,) or v.shape!=(3,) or not np.isfinite(np.r_[p,v]).all():return 'Invalid active kinematics'
    # Conservative stopping reserve protects the actual inactive aircraft bodies.
    speed=float(np.linalg.norm(v));reserve=2.+speed*.85+speed*speed/(2*1.2)
    for d in drones:
        if d['id'] in ((active_id,) if active_ids is None else active_ids):continue
        q=np.array([d.get(k,float('nan')) for k in ('x','y','z')]);s=d.get('speed',float('nan'))
        if not np.isfinite(np.r_[q,s]).all():return 'Invalid inactive aircraft observation'
        if not 0<=s<=.15:return 'Inactive aircraft motion invalid or unsafe'
        if np.linalg.norm(p-q)<reserve:return 'Inactive aircraft stopping reserve'
    return None


class SingleFlight:
    def __init__(self,spawn,route,entry,handoff,end,climb_target=None):
        self.spawn=np.asarray(spawn,float);self.route=[np.asarray(p,float) for p in route]
        self.entry=np.asarray(entry,float);self.handoff=np.asarray(handoff,float);self.end=np.asarray(end,float)
        if not self.route or not np.isfinite(np.concatenate([self.spawn,self.entry,self.handoff,self.end]+self.route)).all():raise ValueError('Invalid route')
        self.climb_target=np.asarray(self.route[0] if climb_target is None else climb_target,float)
        self.stage='IDLE';self.started=0.;self.dwell=None;self.index=0

    def transition(self,stage,now):self.stage=stage;self.started=now;self.dwell=None
    def stable(self,condition,now):
        if not condition:self.dwell=None;return False
        if self.dwell is None:self.dwell=now
        return now-self.dwell>=2.
    def start(self,now,auto=True):
        if self.stage!='IDLE':raise ValueError('Mission already started')
        self.auto=auto
        self.transition('TAKEOFF',now)
        return dict(op='takeoff',height_mode='AGL',height=2.,speed=.5,_stage=self.stage)
    def pause(self,now):
        if self.stage not in ('TAKEOFF','ASSEMBLED','CLIMB','CRUISE','DESCENT','TERMINAL'):raise ValueError('Phase cannot be paused')
        self.paused_stage=self.stage;self.pause_started=now;self.stage='PAUSED';self.dwell=None
    def resume(self,now):
        if self.stage!='PAUSED':raise ValueError('Only a user pause can resume')
        self.stage=self.paused_stage;self.started+=now-self.pause_started;self.dwell=None
    def goto(self,p,speed):return dict(op='goto',height_mode='ABS',height=float(p[2]),x=float(p[0]),y=float(p[1]),speed=speed,_stage=self.stage)
    def step(self,s,now,*,climb_elapsed=None,climb_limit=180.):
        if self.stage in ('IDLE','HOLD','FAILED','LANDED','ASSEMBLED','PAUSED'):return None
        elapsed=now-self.started
        if climb_elapsed is not None:
            if self.stage!='CLIMB' or not np.isfinite(climb_elapsed) or climb_elapsed<0:raise ValueError('Invalid coordinated climb elapsed time')
            elapsed=float(climb_elapsed)
        if not np.isfinite(climb_limit) or not 0<climb_limit<=600:raise ValueError('Invalid bounded climb limit')
        if elapsed>dict(TAKEOFF=90,CLIMB=climb_limit,CRUISE=3000,DESCENT=600,TERMINAL=1800,LAND=120)[self.stage]:raise ValueError('Phase timeout: '+self.stage)
        p=np.asarray(s['position'],float);v=np.asarray(s['velocity'],float);speed=float(np.linalg.norm(v))
        if p.shape!=(3,) or v.shape!=(3,) or not np.isfinite(np.r_[p,v,s.get('agl',float('nan'))]).all():raise ValueError('Invalid mission state')
        if self.stage!='LAND' and (not s.get('armed') or s.get('contact')):
            if self.stage!='TAKEOFF':raise ValueError('Unexpected contact/disarm')
        settled=s.get('phase')=='HOLD' and speed<.2
        if self.stage=='TAKEOFF' and self.stable(settled and abs(s['agl']-2)<.25 and s.get('armed') and not s.get('contact'),now):
            if not self.auto:self.transition('ASSEMBLED',now);return None
            self.transition('CLIMB',now);task=self.goto(self.climb_target,.8);task['_forward_climb']=True;return task
        if self.stage=='CLIMB' and self.stable(settled and abs(p[2]-270)<.3,now):
            self.transition('CRUISE',now);return self.goto(self.route[0],10.5)
        if self.stage=='CRUISE' and np.linalg.norm(p-self.route[self.index])<.6 and speed<.3:
            self.index+=1
            if self.index<len(self.route):return self.goto(self.route[self.index],10.5)
            self.transition('DESCENT',now)
            task=self.goto([*self.handoff[:2],270.],5.)
            task.update(_forward_descent=True,_descent_entry=self.entry[:2].tolist(),_descent_final_z=float(self.handoff[2]));return task
        if self.stage=='DESCENT' and self.stable(settled and np.linalg.norm(p[:2]-self.handoff[:2])<.5 and abs(s['agl']-5)<.25,now):
            self.transition('TERMINAL',now)
            return dict(op='terrain_follow',height_mode='AGL',height=5.,x=float(self.end[0]),y=float(self.end[1]),speed=2.5,_terminal_profile=True,_profile_start=self.handoff[:2].tolist(),_stage=self.stage)
        if self.stage=='TERMINAL' and self.stable(s.get('phase')=='END_HOLD' and speed<.2 and np.linalg.norm(p[:2]-self.end[:2])<.5 and abs(s['agl']-2)<.25,now):
            self.transition('LAND',now);return dict(op='land',_stage=self.stage)
        if self.stage=='LAND' and self.stable(s.get('phase')=='LANDED' and s.get('contact') and not s.get('armed',True) and speed<.15 and np.linalg.norm(p[:2]-self.end[:2])<.5,now):self.transition('LANDED',now)
        return None

class SceneProgress:
    """Freshness of real Unity main-thread scene frames, independent of sensors."""
    def __init__(self):self.frame=-1;self.changed=None;self.invalid=False
    def observe(self,frame,now):
        if type(frame) is not int or frame<0 or frame<self.frame:self.invalid=True;return
        if frame>self.frame:self.frame=frame;self.changed=now
    def observe_scene(self,scene,now):
        self.observe(scene.get('scene_frame'),now) # Missing field fails closed; never substitute sensor frames.
    def fresh(self,now):return not self.invalid and self.changed is not None and 0<=now-self.changed<=.6


def forward_climb_target(model,spawn,first_waypoint):
    start=np.asarray(spawn,float).copy();start[2]=float(model.ground(start[None,:2])[0])+2.
    delta=np.asarray(first_waypoint,float)[:2]-start[:2];distance=float(np.linalg.norm(delta))
    if not np.isfinite(distance) or distance<1.:raise ValueError('Forward climb needs a distinct waypoint')
    target=np.r_[start[:2]+delta*min(100.,distance)/distance,model.altitude]
    if not model.sweep_clear(start[None,:],target[None,:],minimum_agl=1.5):raise ValueError('Forward climb path lacks terrain/building clearance')
    return target


def inactive_speed_cap(scene,active_id,position,active_ids=None):
    """Invert the existing spherical stopping reserve with 0.5m unused margin."""
    p=np.asarray(position,float);active=(active_id,) if active_ids is None else active_ids
    positions=[np.array([d[k] for k in ('x','y','z')],float) for d in scene['drones'] if d['id'] not in active]
    if p.shape!=(3,) or not positions or not np.isfinite(np.r_[p,np.concatenate(positions)]).all():raise ValueError('Invalid inactive positions for speed cap')
    distance=min(float(np.linalg.norm(p-q)) for q in positions)
    available=max(0.,distance-2.-.5)
    limit=(-2.04+math.sqrt(2.04**2+9.6*available))/2.
    # Downward quantization prevents tiny scene jitter changing every lease generation.
    return min(10.5,math.floor(max(0.,limit)*10.)/10.)
