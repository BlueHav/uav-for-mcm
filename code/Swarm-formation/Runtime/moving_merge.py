"""Continuous moving formation references; local sensing still gates every command.

Only the final L boundary has zero velocity. Intermediate boundaries preserve
forward motion and C2 continuity. This is a reference, not a flight certificate.
"""
import numpy as np
from point_mission_core import STAR
from cloud_config import STAR_ENTRY_SPEED

# 24 m/s exceeded the unchanged simulator's drag/45-degree tilt authority
# (about 21.3 m/s level flight). Retain headroom for tracking and descent.
CATCHUP_SPEED=20.
MERGE_ENTRY_DISTANCE=4600.
MOVING_RADIUS=30.

def polynomial(p,v,a,q,w,b,duration):
    t=float(duration)
    c=np.zeros((6,)+p.shape)
    c[0]=p;c[1]=v*t;c[2]=a*t*t/2
    rhs=np.stack((q-c[0]-c[1]-c[2],w*t-c[1]-2*c[2],b*t*t-2*c[2]))
    c[3:]=np.linalg.solve(np.array([[1.,1,1],[3,4,5],[6,12,20]]),rhs.reshape(3,-1)).reshape((3,)+p.shape)
    return c

class MovingMerge:
    def __init__(self,model,group,position,velocity,acceleration=None):
        p=np.asarray(position,float);v=np.asarray(velocity,float)
        if p.shape!=(5,3) or v.shape!=(5,3) or not np.isfinite(np.r_[p.ravel(),v.ravel()]).all():raise ValueError('Invalid moving merge entry')
        d=model.direction;side=model.normal*(-80. if group==0 else 80.)
        star=STAR*(MOVING_RADIUS/3.3)
        # Once group centers are close, reserve disjoint lateral strips. A
        # wide world-axis star would enter the other group's stop corridors.
        # Fore/aft spacing remains long until speed has actually been planned
        # down; final low-speed contraction returns to the exact world slots.
        def corridor_shape(length):
            return -STAR[:,0,None]*model.normal+STAR[:,1,None]/3.3*d*length
        anchor=model.assembly_anchor(group)
        first=np.c_[anchor-600*d+side+star,(200. if group==0 else 110.)+3.*np.arange(5)]
        second=np.c_[anchor-300*d+model.offset(group)+corridor_shape(100.),(130. if group==0 else 60.)+3.*np.arange(5)]
        near=np.c_[anchor-40*d+model.offset(group)+corridor_shape(10.),(60. if group==0 else 20.)+3.*np.arange(5)]
        end=model.descent(group)[1]
        # Open lateral lanes at cruise speed before closing longitudinal gaps.
        # Otherwise a catching-up tail can reach its predecessor's current
        # body inside its complete stopping corridor during early convergence.
        lane_time=15.;lane=p+np.r_[d*model.cruise_speed*lane_time,0.]
        current=(p[:,:2]-p[2,:2])@model.normal
        lane[:,:2]+=(star@model.normal-current)[:,None]*model.normal
        first_time=float(np.dot(first[2,:2]-lane[2,:2],d)/model.cruise_speed)
        if first_time<60.:raise ValueError('Insufficient distance for moving catch-up')
        # Exchange group sides while their altitude bands are still separated.
        # This is a real lateral turn; reserve time for its bounded curvature.
        turn_time=1.2*np.linalg.norm(second[2,:2]-first[2,:2])/model.cruise_speed
        self.durations=np.array([lane_time,first_time,turn_time,520./(model.cruise_speed+3.),80./3.])
        self.duration=float(self.durations.sum());self.coefficients=[]
        a=np.zeros((5,3)) if acceleration is None else np.asarray(acceleration,float)
        if a.shape!=(5,3) or not np.isfinite(a).all():raise ValueError('Invalid moving merge acceleration')
        for i,(q,t) in enumerate(zip((lane,first,second,near,end),self.durations)):
            vz=0. if i==0 else (-.8 if group==0 else -2.) if i==1 else (-3. if group==0 else -2.)
            if i==1:
                # A longer catch-up interval needs a gentler exit descent.
                # Excessive negative endpoint speed otherwise makes a quintic
                # rise first to compensate for its fixed total height loss.
                vz=-min(-vz,2.*max(0.,float(np.min(p[:,2]-q[:,2])))/t)
            w=np.tile(np.r_[d*model.cruise_speed,vz],(5,1)) if i<3 else np.tile(np.r_[d*3.,0.],(5,1)) if i==3 else np.zeros((5,3))
            b=np.zeros((5,3));self.coefficients.append(polynomial(p,v,a,q,w,b,t));p,v,a=q,w,b
        # Cheap admission guard; map/stop certification is still performed by
        # each aircraft using its current local observations before control.
        for t in np.linspace(0.,self.duration,81):
            q,w,b=self.sample(t)
            speed=np.linalg.norm(w[:,:2],axis=1)
            if (speed.max()>CATCHUP_SPEED-.2 or speed[2]>model.cruise_speed+(.5 if t<lane_time else .1) or
                np.abs(w[:,2]).max()>6. or np.linalg.norm(b[:,:2],axis=1).max()>2. or
                np.abs(b[:,2]).max()>2. or q[:,2].max()>300.):
                raise ValueError('Moving merge exceeds dynamic admission limits')

    def sample(self,elapsed):
        t=float(np.clip(elapsed,0.,self.duration));edges=np.cumsum(self.durations)
        i=min(len(edges)-1,int(np.searchsorted(edges,t,side='right')))
        duration=self.durations[i];u=(t-(edges[i-1] if i else 0.))/duration
        c=self.coefficients[i]
        return tuple(sum(c[k]*np.prod(np.arange(k-order+1,k+1))*u**(k-order)/duration**order for k in range(order,6)) for order in (0,1,2))


class RelayStarMerge(MovingMerge):
    """Widen the delayed line, then contract the star as forward speed falls."""

    def __init__(self, model, group, position, velocity, acceleration):
        p, v, a = (np.asarray(x, float) for x in (position, velocity, acceleration))
        if any(x.shape != (5,3) or not np.isfinite(x).all() for x in (p,v,a)):
            raise ValueError('Invalid relay descent boundary')
        anchor = model.assembly_anchor(group)
        direction = anchor - p[2,:2]
        remaining = float(np.linalg.norm(direction))
        if remaining < 750.:
            raise ValueError('Insufficient room to form star')
        direction /= remaining
        normal = np.array([-direction[1], direction[0]])
        wide_star = STAR * 8.
        lateral = (p[:,:2]-p[2,:2]) @ normal
        lane = p.copy()
        lane[:,:2] += 78.*direction + ((wide_star @ normal)-lateral)[:,None]*normal
        lane[:,2] -= 4.
        near_xy = anchor - 100.*direction + wide_star
        near = np.c_[near_xy, model.ground(near_xy)+25.]
        end = model.descent(group)[1]
        self.durations = np.array([12., (remaining-178.)/9.5, 60.])
        self.duration = float(self.durations.sum())
        self.coefficients = []
        for q, speed, vz, duration in zip((lane,near,end),(STAR_ENTRY_SPEED,3.,0.),(-.35,-.4,0.),self.durations):
            w = np.tile(np.r_[speed*direction,vz],(5,1))
            b = np.zeros((5,3))
            self.coefficients.append(polynomial(p,v,a,q,w,b,duration))
            p,v,a=q,w,b
        samples = [self.sample(t) for t in np.linspace(0.,self.duration,401)]
        q,w,b = (np.asarray(x) for x in zip(*samples))
        if (np.max(np.linalg.norm(w[:,:,:2],axis=2))>19.8
                or np.max(np.linalg.norm(b,axis=2))>2.3 or np.max(np.abs(w[:,:,2]))>4.
                or np.min(q[:,:,2]-model.ground(q[:,:,:2].reshape(-1,2)).reshape(-1,5))<3.5):
            raise ValueError('Relay star trajectory violates dynamics or terrain clearance')
        pairs=np.triu_indices(5,1)
        if np.min(np.linalg.norm(q[:,pairs[0]]-q[:,pairs[1]],axis=2))<2.8:
            raise ValueError('Relay star trajectory violates member clearance')
