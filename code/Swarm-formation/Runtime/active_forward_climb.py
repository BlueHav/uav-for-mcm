"""Bounded observed forward climb; existing executor still certifies stopping."""
import math
import numpy as np

def observed_climb(position,goal,yaw,speed,local_map,vertical_speed):
    p=np.asarray(position,float);goal=np.asarray(goal,float)
    if p.shape!=(3,) or goal.shape!=(3,) or not np.isfinite(np.r_[p,goal,yaw,speed,vertical_speed]).all() or speed<=0 or vertical_speed<=0:raise ValueError('Invalid climb inputs')
    delta=goal-p;distance=float(np.linalg.norm(delta))
    if distance<.05:return np.zeros(3),p[None,:],'Forward climb goal reached'
    heading=math.atan2(delta[1],delta[0]);error=math.atan2(math.sin(heading-yaw),math.cos(heading-yaw))
    if np.linalg.norm(delta[:2])>.05 and abs(error)>.65:return np.zeros(3),p[None,:],'Turning sensor toward forward climb'
    unit=delta/distance;length=min(8.,distance)
    path=p+np.linspace(0.,length,max(3,int(math.ceil(length/.25))+1))[:,None]*unit
    if not np.all(local_map.points_clear(path,radius=.35)):return np.zeros(3),path,'Forward climb corridor unobserved/occupied'
    magnitude=min(float(speed),2.5,math.sqrt(max(0.,2.*1.2*max(0.,distance-.05))))
    if abs(unit[2])>1e-9:magnitude=min(magnitude,float(vertical_speed)/abs(unit[2]))
    return unit*magnitude,path,'Observed three-dimensional forward climb'
