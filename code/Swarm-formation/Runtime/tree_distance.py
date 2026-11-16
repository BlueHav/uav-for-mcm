"""Per-group XY cohesion for the existing leading five-member star.

Caps only the aircraft ahead on a stretched adjacency. Obstacle avoidance,
sensor freshness and collision stopping retain authority over every command.
"""
import numpy as np

STAR_EDGES=((2,0),(2,1),(0,3),(1,4))

def member_speed_limit(reference,member):
    base=float(reference['speed'])
    if not np.isfinite(base) or base<=0:raise ValueError('Invalid nominal member speed')
    if 'member_speed_caps' not in reference:return base
    caps=np.asarray(reference['member_speed_caps'],float)
    if caps.shape!=(5,) or not np.isfinite(caps).all() or np.any(caps<0) or np.any(caps>base+1e-8):
        raise ValueError('Invalid five-member speed caps')
    return min(base,float(caps[member]))

def validate_cohesion_reference(reference,now=None):
    c=reference.get('cohesion')
    if c is None:
        if reference.get('cohesion_required'):raise ValueError('Missing required cohesion observation')
        return None
    if not isinstance(c,dict):raise ValueError('Invalid cohesion observation')
    p=np.asarray(c.get('positions'),float);v=np.asarray(c.get('velocities'),float)
    if p.shape!=(5,3) or v.shape!=(5,3) or not np.isfinite(np.r_[p.ravel(),v.ravel()]).all():
        raise ValueError('Invalid measured cohesion positions/velocities')
    edges=c.get('edges',[])
    if not isinstance(edges,list) or len(edges)!=4:raise ValueError('Invalid cohesion adjacency')
    identities=[]
    for e in edges:
        if (not isinstance(e,dict) or type(e.get('parent')) is not int or type(e.get('child')) is not int
                or type(e.get('active')) is not bool):raise ValueError('Invalid cohesion edge')
        identities.append((e['parent'],e['child']))
    if set(identities)!=set(STAR_EDGES):raise ValueError('Unknown/duplicate cohesion adjacency')
    stamp=float(c.get('stamp',float('nan')));reference_stamp=float(reference.get('stamp',float('nan')))
    factor=float(c.get('slow_factor',float('nan')))
    samples=np.asarray(c.get('sample_stamps'),float)
    if (not np.isfinite([stamp,reference_stamp,factor]).all() or abs(stamp-reference_stamp)>1e-6
            or not 0<factor<1 or type(c.get('fresh')) is not bool):raise ValueError('Invalid cohesion epoch/factor')
    if samples.shape!=(5,) or not np.isfinite(samples).all() or np.any(samples>stamp+.1):
        raise ValueError('Invalid paired cohesion observation times')
    minimum=float(c.get('minimum_speed_mps',2.))
    soft=np.asarray(c.get('formation_speed_caps',[reference['speed']]*5),float)
    priorities=c.get('order_recovery_members',[])
    if (not np.isfinite(minimum) or not 2.<=minimum<=float(reference['speed'])
            or soft.shape!=(5,) or not np.isfinite(soft).all()
            or np.any(soft<minimum) or np.any(soft>float(reference['speed'])+1e-8)
            or not isinstance(priorities,list) or any(type(i) is not int or not 0<=i<5 for i in priorities)):
        raise ValueError('Invalid formation speed limits')
    checked=stamp if now is None else float(now)
    fresh=c['fresh'] and -.1<=checked-stamp<=.3 and bool(np.all((checked-samples>=-.1)&(checked-samples<=.3)))
    return c,p,v,fresh

def _incident_direction_cap(c,positions,velocities,member,position,desired,hard,nominal=None):
    horizontal=np.asarray(desired,float)[:2];speed=float(np.linalg.norm(horizontal))
    if speed<1e-9:return hard
    direction=horizontal/speed
    minimum=float(c.get('minimum_speed_mps',2.))
    cap=min(hard,float(c.get('formation_speed_caps',[hard]*5)[member]))
    # A predecessor may regain normal formation permission to restore order,
    # but every incident distance edge must still constrain outward motion.
    # Catching one wing must not pull the other wing farther away.
    for e in c['edges']:
        if not e['active'] or member not in (e['parent'],e['child']):continue
        other=e['child'] if member==e['parent'] else e['parent']
        delta=np.asarray(position,float)[:2]-positions[other,:2];distance=float(np.linalg.norm(delta))
        if distance<1e-6:continue
        radial=delta/distance;outward=float(np.dot(direction,radial))
        if outward<=1e-6:continue
        # Only radial neighbor motion catches this aircraft. Sideways motion
        # must not grant permission to restart outward on alternating headings.
        closing=max(0.,float(np.dot(velocities[other,:2],radial)))
        cap=min(cap,float(c['slow_factor'])*(hard if nominal is None else nominal),.5*closing/outward)
    return min(hard,max(minimum,cap))

def directional_speed_limit(reference,member,position,desired,now=None):
    hard=member_speed_limit(reference,member)
    observation=validate_cohesion_reference(reference,now)
    if observation is None:return hard
    c,p,v,fresh=observation
    # Stale group geometry cannot justify a direction-dependent decision.
    # Fall back to the minimum soft policy cap. Local sensor/peer safety is
    # checked independently and may still require a genuine safety stop.
    if not fresh:return min(hard,float(c.get('minimum_speed_mps',2.)))
    position=np.asarray(position,float);desired=np.asarray(desired,float)
    if position.shape!=(3,) or desired.shape!=(3,) or not np.isfinite(np.r_[position,desired]).all():
        raise ValueError('Invalid current cohesion command state')
    return _incident_direction_cap(c,p,v,member,position,desired,hard,float(reference['speed']))

class TreeDistanceGate:
    def __init__(self,template,trigger=50.,resume=25.,slow_factor=.35,max_speed=6.,lookahead=3.5,braking_margin=22.,minimum_speed=2.):
        template=np.asarray(template,float)
        values=np.array([trigger,resume,slow_factor,max_speed,lookahead,braking_margin,minimum_speed],float)
        if template.shape!=(5,2) or not np.isfinite(template).all() or not np.isfinite(values).all():
            raise ValueError('Invalid star distance configuration')
        if not 0<resume<trigger or not 0<slow_factor<1 or not 2<=minimum_speed<=max_speed<=6 or lookahead<0 or not 0<=braking_margin<trigger-resume:
            raise ValueError('Invalid distance hysteresis or speed bounds')
        forward=-np.mean(template-template[2],axis=0)
        if np.linalg.norm(forward)<1e-6:raise ValueError('Leading star direction is required')
        self.forward=forward/np.linalg.norm(forward)
        self.trigger=float(trigger);self.resume=float(resume);self.factor=float(slow_factor)
        self.speed=float(max_speed);self.lookahead=float(lookahead)
        self.braking_margin=float(braking_margin)
        self.minimum=float(minimum_speed)
        self.nominal=[float(np.linalg.norm(template[a]-template[b])) for a,b in STAR_EDGES]
        self.active=[False]*4;self.triggers=[0]*4;self.releases=[0]*4;self.peaks=[0.]*4
        self.courses=[self.forward.copy() for _ in STAR_EDGES]

    def update(self,positions,velocities,fresh=True,formation=None):
        p,v=np.asarray(positions,float),np.asarray(velocities,float)
        if p.shape!=(5,3) or v.shape!=(5,3) or not np.isfinite(np.r_[p.ravel(),v.ravel()]).all():
            raise ValueError('Invalid measured group state')
        caps=np.full(5,self.speed);edges=[]
        for k,(parent,child) in enumerate(STAR_EDGES):
            delta=p[parent,:2]-p[child,:2];distance=float(np.linalg.norm(delta))
            separating=float(np.dot(v[parent,:2]-v[child,:2],delta/max(distance,1e-9)))
            predicted=distance+max(0.,separating)*self.lookahead
            # Velocity alone misses acceleration from rest and a turn that
            # swaps the front aircraft. Keep room for that new front to stop.
            # 22 m covers the configured 6 m/s jerk-limited stop plus tracking
            # reserve; the physical envelope still needs real-flight auditing.
            guarded=distance+max(self.braking_margin,max(0.,separating)*self.lookahead)
            self.peaks[k]=max(self.peaks[k],distance)
            if fresh:
                if not self.active[k] and guarded>self.trigger:
                    self.active[k]=True;self.triggers[k]+=1
                elif self.active[k] and distance<=self.resume and predicted<=self.trigger:
                    self.active[k]=False;self.releases[k]+=1
            # Use actual travel, including avoidance turns and overtaking.
            # At a stop retain the last observed course, seeded by the star.
            measured=v[parent,:2]+v[child,:2]
            if fresh and np.linalg.norm(measured)>.4:self.courses[k]=measured/np.linalg.norm(measured)
            course=self.courses[k]
            front,rear=(parent,child) if np.dot(delta,course)>=0 else (child,parent)
            edges.append(dict(parent=parent,child=child,front=front,rear=rear,course_xy=course.tolist(),
                nominal_distance_m=self.nominal[k],distance_m=distance,predicted_distance_m=predicted,guard_distance_m=guarded,
                peak_distance_m=self.peaks[k],active=self.active[k],triggers=self.triggers[k],releases=self.releases[k]))
        result=dict(edges=edges,fresh=bool(fresh),positions=p.tolist(),velocities=v.tolist(),slow_factor=self.factor,
                    trigger_m=self.trigger,resume_m=self.resume,lookahead_s=self.lookahead,braking_margin_m=self.braking_margin,
                    minimum_speed_mps=self.minimum)
        if formation is not None:
            result.update(formation=formation,formation_speed_caps=formation['speed_caps'],
                          order_recovery_members=formation['order_recovery_members'])
        # Diagnostic limits along measured motion. Each planner evaluates its
        # proposed direction again, so inward catch-up remains available.
        caps=[_incident_direction_cap(result,p,v,i,p[i],v[i],self.speed) if fresh else self.minimum for i in range(5)]
        result['speed_caps']=caps
        return result
