"""Count1 stopping replay for the bounded pair climb only.

Unlike ten_stop_envelopes, this uses the actual count1 vertical limits.
A replay is a reference trajectory, not a physical tracking certificate.
"""
import math
import numpy as np
from mission_core import smooth_velocity,ACC_XY,JERK_XY,VERTICAL_SPEED

def count1_stop_path(position,velocity,acceleration,desired,delay,dt=.02):
    p,v,a,d=[np.asarray(x,float).copy() for x in (position,velocity,acceleration,desired)]
    if any(x.shape!=(3,) or not np.isfinite(x).all() for x in (p,v,a,d)):raise ValueError('Invalid count1 stopping vectors')
    if not np.isfinite([delay,dt]).all() or not 0<=delay<=1. or not .001<=dt<=.05:raise ValueError('Invalid stopping timing')
    if np.linalg.norm(a[:2])>ACC_XY+1e-6 or abs(a[2])>1.2+1e-6:raise ValueError('Count1 acceleration limit exceeded')
    if np.linalg.norm(d)> .800001 or np.linalg.norm(v)>1.200001:raise ValueError('Pair climb speed envelope exceeded')
    points=[p.copy()];elapsed=0.
    # Eight seconds bounds the low-speed ramp, damping and conservative delay;
    # an unsettled final state is rejected rather than silently truncated.
    while elapsed<8.:
        h=min(dt,delay-elapsed) if elapsed<delay else dt
        if h<1e-9:h=dt
        target=d if elapsed<delay-1e-9 else np.zeros(3)
        nv,na=smooth_velocity(v,a,target,h)
        p=p+(v+nv)*h*.5;v,a=nv,na;elapsed+=h;points.append(p.copy())
        if elapsed>=delay and np.linalg.norm(v)<.02 and np.linalg.norm(a)<.05:return np.array(points)
    raise ValueError('Count1 stop did not settle within bounded replay')

def independent_path_lower_bound(first,second):
    """Conservative continuous distance; vertex distance minus segment lengths.

Every point of a segment is within half its length of an endpoint. This
    bound covers arbitrary asynchronous progress without a synchrony assumption.
    """
    a,b=[np.asarray(x,float) for x in (first,second)]
    if any(x.ndim!=2 or x.shape[1]!=3 or len(x)<2 or not np.isfinite(x).all() for x in (a,b)):raise ValueError('Invalid stopping paths')
    minimum=min(float(np.min(np.linalg.norm(p-b,axis=1))) for p in a)
    error=.5*(np.max(np.linalg.norm(np.diff(a,axis=0),axis=1))+np.max(np.linalg.norm(np.diff(b,axis=0),axis=1)))
    return minimum-float(error)

def watchdog_stop_path(position,velocity,dt=.02):
    """NetworkControl.cpp watchdogStopTick, including reset of stop_acc_ to zero."""
    p=np.asarray(position,float).copy();v=np.asarray(velocity,float).copy();a=np.zeros(3);points=[p.copy()]
    if p.shape!=(3,) or v.shape!=(3,) or not np.isfinite(np.r_[p,v]).all():raise ValueError('Invalid watchdog state')
    for _ in range(400):
        speed=float(np.linalg.norm(v[:2]));target=np.zeros(3)
        if speed>1e-9:target[:2]=-v[:2]/speed*min(2.2,math.sqrt(6*speed))
        if abs(v[2])>1e-9:target[2]=-math.copysign(min(1.2,math.sqrt(6*abs(v[2]))),v[2])
        delta=target[:2]-a[:2];na=a.copy();na[:2]+=delta*min(1.,3*dt/max(float(np.linalg.norm(delta)),1e-9));na[2]+=max(-3*dt,min(3*dt,target[2]-a[2]))
        nv=v+.5*(a+na)*dt;p+=v*dt+.5*a*dt*dt+(na-a)*(dt*dt/6.)
        if np.dot(nv[:2],v[:2])<=0:nv[:2]=0.;na[:2]=0.
        if nv[2]*v[2]<=0:nv[2]=0.;na[2]=0.
        v,a=nv,na;points.append(p.copy())
        if np.linalg.norm(v)<1e-9:return np.array(points)
    raise ValueError('Watchdog replay did not stop')

def climb_stop_certificate(odometry,status,goals,geometry,now):
    """Simulation acceptance envelope, conditional on the existing 1.5m tracking
    and 3m/s² physical-acceleration bounds. Not a formal physical guarantee.
    Source loss or violation revokes both permits; static-body checks remain.
    """
    bodies=[];reports=[]
    for group,i in enumerate((2,7)):
        o=odometry[i];state=status[i];proof=state['control_reference']
        stamp=float(o['stamp']);rs=float(proof['stamp']);age=max(now-stamp,now-rs)
        if not -.01<=now-stamp<=.15 or not -.01<=now-rs<=.15 or abs(stamp-rs)>.05:raise ValueError('Parallel climb source stale/skewed')
        p=np.asarray(o['p'],float);v=np.asarray(o['v'],float);rp=np.asarray(proof['position'],float);rv=np.asarray(proof['velocity'],float);ra=np.asarray(proof['acceleration'],float);desired=np.asarray(proof['shaped_desired'],float)
        if any(x.shape!=(3,) or not np.isfinite(x).all() for x in (p,v,rp,rv,ra,desired)):raise ValueError('Invalid parallel climb control proof')
        if np.linalg.norm(v)>.9 or np.linalg.norm(rv)>.9 or np.linalg.norm(desired)>.800001:raise ValueError('Parallel climb speed bound')
        if np.linalg.norm(p-rp)>1.5 or np.linalg.norm(v-rv)>.3:raise ValueError('Parallel climb tracking envelope')
        measured_a=np.asarray(o['acceleration'],float);interval=float(o['acceleration_interval'])
        if measured_a.shape!=(3,) or not np.isfinite(measured_a).all() or not 0<interval<=.1 or np.linalg.norm(measured_a)>3.:raise ValueError('Parallel climb acceleration observation')
        start=np.asarray(geometry['starts'][group]);goal=np.asarray(geometry['targets'][group]);delta=goal-start
        progress=float(np.dot(p-start,delta)/np.dot(delta,delta));nearest=start+np.clip(progress,0,1)*delta
        if np.linalg.norm(p-nearest)>1.5 or np.linalg.norm(np.asarray(goals[i])-goal)>.01:raise ValueError('Parallel climb corridor/goal mismatch')
        delay=max(age,0.)+.2+.05+.05 # source age, permit lease, control tick, scheduling
        normal=count1_stop_path(rp,rv,ra,desired,delay)
        # Watchdog latches measured p/v, not the normal controller's acceleration.
        watchdog=watchdog_stop_path(p+v*delay,v)
        duration=max(max(len(normal),len(watchdog))*.02,(float(np.linalg.norm(v))+3.*delay)/1.2+2.*2.2/3.+1.)
        # During the reaction window an unknown physical acceleration can change
        # both position and velocity. Carry the resulting velocity error through
        # the entire modeled stop, rather than assuming a future measured a.
        radius=1.5+.3*delay+1.5*delay*delay+3.*delay*duration
        paths=[normal,np.vstack([p,watchdog])]
        extent=max(float(np.max(np.linalg.norm(path-p,axis=1))) for path in paths)+radius
        bodies.append((p,extent));reports.append(dict(id=i,odom_stamp=stamp,control_stamp=rs,source_age=age,delay=delay,stop_duration=duration,radius=radius,extent=extent,observed_speed=float(np.linalg.norm(v))))
    lower=float(np.linalg.norm(bodies[1][0]-bodies[0][0]))-bodies[0][1]-bodies[1][1]
    return dict(safe=lower>2.,scope='pair CLIMB/CLIMB_READY only; independent full-stop enclosing balls',lower_clearance=lower,required_clearance=2.,members=reports,
                assumptions='Simulation envelope: physical acceleration<=3m/s2; tracking error<=1.5m; count1 normal and actual watchdog reference models')
