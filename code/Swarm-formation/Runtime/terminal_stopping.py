"""Scalar replay of the existing two brakes for flight safety checks.

Same 20 ms steps, 300 ms drive delay, limits and termination as
mission_core.ten_stop_envelopes without a terrain callback. Avoid hundreds of
small NumPy operations while holding the observed-map lock.
"""
import math
import numpy as np
from mission_core import ACC_XY,JERK_XY


def actual_stop_paths(p,v,acceleration,desired):
    values=[np.asarray(x,float) for x in (p,v,acceleration,desired)]
    if any(x.shape!=(3,) or not np.isfinite(x).all() for x in values):
        raise ValueError('Invalid terminal stopping state')
    initial_p,initial_v,initial_a,drive=[x.tolist() for x in values]
    duration=max(math.sqrt(initial_v[0]**2+initial_v[1]**2)/ACC_XY+2*ACC_XY/JERK_XY,
                 abs(initial_v[2])/1.2+abs(initial_a[2])/6.)+3.
    count=int(math.ceil(duration/.02));paths=[]
    for watchdog in (False,True):
        px,py,pz=initial_p;vx,vy,vz=initial_v;ax,ay,az=initial_a;points=[]
        for step in range(count):
            driving=step<15
            tx,ty,tz=drive if driving else (0.,0.,0.)
            slow=watchdog and not driving
            vertical_acc,vertical_jerk=(1.2,6.) if slow else (2.5,8.)
            jx=36.*(tx-vx)-12.*ax;jy=36.*(ty-vy)-12.*ay
            scale=min(1.,JERK_XY/max(math.sqrt(jx*jx+jy*jy),1e-9))
            bx=ax+jx*scale*.02;by=ay+jy*scale*.02
            scale=min(1.,ACC_XY/max(math.sqrt(bx*bx+by*by),1e-9))
            bx*=scale;by*=scale
            jz=36.*(tz-vz)-12.*az
            bz=az+max(-vertical_jerk,min(vertical_jerk,jz))*.02
            bounded=max(-vertical_acc,min(vertical_acc,bz))
            bz=max(az-vertical_jerk*.02,min(az+vertical_jerk*.02,bounded))
            wx=vx+(ax+bx)*.01;wy=vy+(ay+by)*.01
            wz=max(-7.,min(7.,vz+(az+bz)*.01))
            px+=(vx+wx)*.01;py+=(vy+wy)*.01;pz+=(vz+wz)*.01
            vx,vy,vz=wx,wy,wz;ax,ay,az=bx,by,bz
            points.append((px,py,pz))
            if not driving and math.sqrt(vx*vx+vy*vy+vz*vz)<.02 and math.sqrt(ax*ax+ay*ay+az*az)<.05:
                break
        paths.append(np.asarray(points))
    return paths


def peer_stop_clear(p,v,acceleration,desired,peers,clearance):
    """Check time-aligned stops, including a peer already commanding a brake."""
    if not peers:
        return True
    own=actual_stop_paths(p,v,acceleration,desired)
    for other,velocity,acc,drive in peers:
        margin=max(2.,min(clearance,float(np.linalg.norm(p-other))))
        for other_drive in (drive,np.zeros(3)):
            stop=actual_stop_paths(other,velocity,acc,other_drive)
            for a,b in zip(own,stop):
                n=max(len(a),len(b))
                i=np.arange(n)
                delta=a[np.minimum(i,len(a)-1)]-b[np.minimum(i,len(b)-1)]
                if np.any(np.einsum('ij,ij->i',delta,delta)<(margin-1e-8)**2):
                    return False
    return True


def clear_reduced_merge_velocity(position, velocity, acceleration, desired, points_clear):
    for scale in (.85, .7, .5, .25):
        candidate = desired * scale
        if all(points_clear(path, radius=.35).all()
               for path in actual_stop_paths(position, velocity, acceleration, candidate)):
            return candidate
    return None
