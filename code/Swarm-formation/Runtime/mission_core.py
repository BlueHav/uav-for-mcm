"""Pure mission limits; contains no scene geometry or simulator oracle."""
import numpy as np

BODY_OFFSET = 0.083263099193573
ACC_XY = 2.2  # Tracking reserve below the 3 m/s² physical acceptance limit.
VERTICAL_SPEED = 1.8  # Legacy non-ten-mode command limit.
JERK_XY = 3.0

def brake_settled(velocity,reference_velocity,acceleration,full_3d=False):
    axes=slice(None) if full_3d else slice(0,2)
    return max(np.linalg.norm(np.asarray(velocity)[axes]),np.linalg.norm(np.asarray(reference_velocity)[axes]))<=.15 and np.linalg.norm(np.asarray(acceleration)[axes])<=.2


def limit_norm(value, limit):
    value = np.asarray(value, float)
    return value * min(1., float(limit) / max(float(np.linalg.norm(value)), 1e-9))


def made_local_progress(origin,position,velocity,desired,independent=False):
    """Count measured commanded restart, while rejecting jitter and drift."""
    delta=np.asarray(position,float)-np.asarray(origin,float)
    if np.linalg.norm(delta)>.5:return True
    v=np.asarray(velocity,float)[:2];command=np.asarray(desired,float)[:2]
    return bool(independent and np.linalg.norm(delta[:2])>.15 and
                np.linalg.norm(v)>.2 and np.dot(v,command)>0.)


def slew_velocity(previous, target, dt):
    previous, target = np.asarray(previous, float), np.asarray(target, float)
    result = previous.copy()
    result[:2] += limit_norm(target[:2] - previous[:2], ACC_XY * dt)
    result[2] += np.clip(target[2] - previous[2], -1.5*dt, 1.5*dt)
    result[2] = np.clip(result[2], -VERTICAL_SPEED, VERTICAL_SPEED)
    return result


def braking_speed(observed_distance, latency=.25, margin=1., acceleration=ACC_XY):
    # v*latency + v²/(2a) <= measured usable corridor.
    latency += acceleration/(2*JERK_XY)
    distance = max(0., float(observed_distance)-margin)
    return max(0., -acceleration*latency + np.sqrt((acceleration*latency)**2+2*acceleration*distance))


def smooth_velocity(previous, acceleration, target, dt,vertical_speed=VERTICAL_SPEED,vertical_acc=1.2,vertical_jerk=6.):
    """S-curve velocity command: acceleration and its rate are independent limits."""
    previous=np.asarray(previous,float);acceleration=np.asarray(acceleration,float);target=np.asarray(target,float)
    error=target-previous
    # Damped velocity response. A sign/sqrt acceleration request with a jerk
    # limiter has no acceleration damping and sustains a position limit cycle.
    jerk=36.*error-12.*acceleration
    next_acc=acceleration.copy();next_acc[:2]+=limit_norm(jerk[:2],JERK_XY)*dt
    next_acc[:2]=limit_norm(next_acc[:2],ACC_XY)
    next_acc[2]+=np.clip(jerk[2],-vertical_jerk,vertical_jerk)*dt
    bounded=np.clip(next_acc[2],-vertical_acc,vertical_acc)
    next_acc[2]=np.clip(bounded,acceleration[2]-vertical_jerk*dt,acceleration[2]+vertical_jerk*dt)
    velocity=previous+(acceleration+next_acc)*dt*.5
    velocity[2]=np.clip(velocity[2],-vertical_speed,vertical_speed)
    return velocity,next_acc


def ten_stop_envelopes(p,v,acceleration,desired,terrain_velocity=None):
    """Certify two distinct curved stops: actual controller and slower watchdog.

    Both include .3 s command latency and preserve current acceleration through
    jerk-bounded profile transitions. No single 3D curve contains the other.
    """
    duration=max(np.linalg.norm(v[:2])/ACC_XY+2*ACC_XY/JERK_XY,
                 abs(v[2])/1.2+abs(acceleration[2])/6.)+3.
    paths=[]
    for watchdog in (False,True):
        position=np.array(p,float);velocity=np.array(v,float);acc=np.array(acceleration,float);points=[]
        for step in range(int(np.ceil(duration/.02))):
            driving=step<15;target=np.array(desired,float) if driving else np.zeros(3)
            if driving and terrain_velocity is not None:target[2]=terrain_velocity(position,velocity)
            slow=watchdog and not driving
            new,acc=smooth_velocity(velocity,acc,target,.02,vertical_speed=7.,vertical_acc=1.2 if slow else 2.5,vertical_jerk=6. if slow else 8.)
            position=position+(velocity+new)*.01;velocity=new;points.append(position.copy())
            if not driving and np.linalg.norm(velocity)<.02 and np.linalg.norm(acc)<.05:break
        paths.append(np.array(points))
    return paths

def landing_column_ready(grid,p,v,ground):
    if not np.isfinite(np.r_[p,v,ground]).all() or np.linalg.norm(v)>.15:return False
    xy=np.asarray(p)[:2]+np.array([[-.25,-.25],[-.25,.25],[.25,-.25],[.25,.25],[0,0]])
    support=grid.ground_at_many(xy)
    if not np.isfinite(support).all() or np.ptp(support)>.15:return False
    heights=np.arange(ground+.65,max(ground+.65,p[2])+.05,.2)
    column=np.tile(p,(len(heights),1));column[:,2]=heights
    return bool(np.all(grid.points_clear(column,radius=.35)))


def altitude_target(mode, height, ground):
    if mode not in ('ABS', 'AGL') or not np.isfinite(height):
        raise ValueError('height_mode must be ABS or AGL; height must be finite')
    if mode == 'AGL':
        if not np.isfinite(ground):
            raise ValueError('Local ground measurement unavailable')
        if height < BODY_OFFSET:
            raise ValueError('AGL target intersects the body ground contact level')
        return float(ground+height)
    if np.isfinite(ground) and height < ground+BODY_OFFSET:
        raise ValueError('Absolute target is below local ground/body contact level')
    return float(height)


def validate_task(task, position, ground_at, bounds=(-8470.625, 1529.375, -6573.4375, 3426.5625, -33.850982666015625, 860.8286743164062)):
    op = task.get('op')
    if op not in ('takeoff','goto','terrain_follow','hold','land','cancel','mission'):
        raise ValueError('Unknown operation')
    mode = task.get('height_mode', 'AGL' if op == 'terrain_follow' else 'ABS')
    x, y = float(task.get('x',position[0])), float(task.get('y',position[1]))
    height = float(task.get('height',position[2]))
    speed = float(task.get('speed',10.5))
    if not np.isfinite([x,y,height,speed]).all() or not (bounds[0]<x<bounds[1] and bounds[2]<y<bounds[3]):
        raise ValueError('Target outside task boundary or nonfinite')
    if not 0 < speed <= 10.5:
        raise ValueError('Speed must be >0 and <=10.5 m/s')
    if op in ('goto','takeoff','terrain_follow'):
        ground=ground_at(x,y)
        # A distant AGL route is resolved progressively from local measurements.
        # Its destination is never declared free merely because it is unobserved.
        if mode=='AGL' and not np.isfinite(ground):ground=ground_at(position[0],position[1])
        z = altitude_target(mode,height,ground)
        if not bounds[4] <= z <= bounds[5]:
            raise ValueError('Target outside altitude boundary')
    return dict(op=op,height_mode=mode,height=height,x=x,y=y,speed=speed)


class CruiseGate:
    def __init__(self):
        self.samples=[]; self.streak=0.; self.best=0.; self.last=None; self.mean=0.

    def update(self, stamp, speed):
        self.samples.append((float(stamp),float(speed)))
        while len(self.samples)>1 and self.samples[1][0] <= stamp-1.:
            self.samples.pop(0)
        if self.samples[-1][0]-self.samples[0][0] < .95:
            return False
        t,v=np.asarray(self.samples).T
        self.mean=float(np.trapz(v,t)/(t[-1]-t[0]))
        dt=0. if self.last is None else stamp-self.last
        self.streak=self.streak+dt if self.mean>=10. else 0.
        self.best=max(self.best,self.streak);self.last=stamp
        return self.best>=60.
