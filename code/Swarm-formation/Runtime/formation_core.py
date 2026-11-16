"""Five-agent geometry; Swarm-Formation@967a4bd graph metric, no world map.

Templates are the first five entries of uav_swarm_pro's types 4/5/6,
scaled by two. Shape similarity is supplemented by metric slot tracking.
"""
import numpy as np

def shape(name):
    values={'assembly':[[0,0,0],[-2,-2,0],[-2,2,0],[2,-2,0],[2,2,0]],
            'line':[[0,0,0],[0,-2,0],[0,-4,0],[0,-6,0],[0,-8,0]],
            'final':[[0,0,0],[-3,-1.5,0],[3,-1.5,0],[-1.5,-3,0],[1.5,-3,0]]}
    if name not in values:raise ValueError('Unknown formation')
    return np.array(values[name],float)*2

def laplacian(p):
    p=np.asarray(p,float);a=np.sum((p[:,None]-p[None,:])**2,axis=2)
    degree=a.sum(axis=1)
    if np.any(degree<1e-10):raise ValueError('Degenerate formation graph')
    return np.eye(len(p))-a/np.sqrt(degree[:,None]*degree[None,:])

def graph_cost(actual,desired):
    return float(np.sum((laplacian(actual)-laplacian(desired))**2))

def graph_gradient(actual,desired):
    # Differentiate the same normalized-Laplacian cost. Five vertices only;
    # central differences avoid importing the reference's ROS/Gazebo stack.
    p=np.asarray(actual,float);g=np.zeros_like(p);eps=1e-3
    for i in range(5):
        for k in range(3):
            delta=np.zeros_like(p);delta[i,k]=eps
            g[i,k]=(graph_cost(p+delta,desired)-graph_cost(p-delta,desired))/(2*eps)
    return g

def min_separation(p):
    p=np.asarray(p,float)
    return min(float(np.linalg.norm(p[i]-p[j])) for i in range(len(p)) for j in range(i))

def transition(before,after):
    before,after=np.asarray(before,float),np.asarray(after,float)
    ceiling=max(np.max(before[:,2]),np.max(after[:,2]))
    lifted=before.copy();lifted[:,2]=ceiling+3*np.arange(5)
    transferred=after.copy();transferred[:,2]=lifted[:,2]
    return [lifted,transferred,after.copy()]

def cruise_step(distance,dt,max_error,blocked):
    return float(distance)+(.8*min(.25,max(0.,dt)) if max_error<1.5 and not blocked else 0.)

def validate_reference(ref,now,clearance=2.,max_speed=1.):
    p=np.asarray(ref['targets'],float);speed=float(ref['speed']);age=now-float(ref['stamp'])
    if p.shape!=(5,3) or not np.all(np.isfinite(p)):raise ValueError('Invalid five-agent targets')
    if not -.1<=age<=1.2:raise ValueError('Formation reference stale')
    if not 0<speed<=max_speed:raise ValueError('Formation speed outside configured limits')
    if not np.all((p[:,0]>-8470.625)&(p[:,0]<1529.375)&(p[:,1]>-6573.4375)&(p[:,1]<3426.5625)&(p[:,2]>-33.850982666015625)&(p[:,2]<860.8286743164062)):
        raise ValueError('Formation reference outside task bounds')
    if min_separation(p)<clearance:raise ValueError('Formation targets overlap')
    if not isinstance(ref.get('run_id'),str) or not ref['run_id']:raise ValueError('Missing run id')
    return p

def peer_safe(p,v,other,other_v,horizon=2.5,clearance=2.):
    delta=np.asarray(p,float)-other;vel=np.asarray(v,float)-other_v
    t=np.clip(-np.dot(delta,vel)/max(np.dot(vel,vel),1e-12),0,horizon)
    return bool(np.linalg.norm(delta+t*vel)>=clearance)

def damped_settle(error,velocity,observed):
    error,velocity,observed=[np.asarray(v,float) for v in (error,velocity,observed)]
    target=.5*error-.65*velocity
    result=observed.copy()
    if np.linalg.norm(error[:2])<3.:
        cap=min(.8,np.linalg.norm(observed[:2]));result[:2]=target[:2]*min(1.,cap/max(np.linalg.norm(target[:2]),1e-9))
    result[2]=np.clip(target[2],-min(.6,abs(observed[2])),min(.6,abs(observed[2])))
    return result
