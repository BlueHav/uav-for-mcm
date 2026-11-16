"""Standalone Swarm-Formation MINCO/L-BFGS adapter. No command publisher.
Backend includes actual 967a4bd headers. Native PolyTraj communication is optional.
Every caller must also collision-check returned dense samples in RollingLocalMap.
"""
import portable_paths
import os
import subprocess
import numpy as np

BACKEND = (str(portable_paths.SWARM) + '/swarm_minco')

def optimize(candidate, durations, neighbors=None, neighbor_dt=.1, start_velocity=(0,0,0), start_acceleration=(0,0,0), end_velocity=(0,0,0), end_acceleration=(0,0,0), timeout=.8, executable=BACKEND, max_acc=3., max_vz=2.):
    candidate, durations = np.asarray(candidate,float), np.asarray(durations,float)
    if not np.isfinite(max_acc+max_vz) or not 0<max_acc<=3 or not 0<max_vz<=2:
        return dict(accepted=False,reason='invalid_dynamic_limits')
    neighbors = np.empty((0,2,3)) if neighbors is None else np.asarray(neighbors,float)
    if candidate.ndim != 2 or candidate.shape[1] != 3 or len(candidate)<2 or len(candidate)>12 or durations.shape!=(len(candidate)-1,) or np.any(durations<=0):
        return dict(accepted=False,reason='invalid_candidate')
    if neighbors.ndim != 3 or neighbors.shape[2]!=3 or neighbors.shape[1]<2 or neighbor_dt<=0:
        return dict(accepted=False,reason='invalid_neighbors')
    arrays=[candidate,durations,np.asarray(start_velocity),np.asarray(start_acceleration),np.asarray(end_velocity),np.asarray(end_acceleration),neighbors]
    if not all(np.all(np.isfinite(a)) for a in arrays):
        return dict(accepted=False,reason='nonfinite_input')
    if len(neighbors) and (neighbors.shape[1]-1)*neighbor_dt < durations.sum()-1e-6:
        return dict(accepted=False,reason='neighbor_prediction_too_short')
    values=[len(candidate),len(neighbors),neighbors.shape[1],neighbor_dt]
    for a in arrays: values.extend(a.reshape(-1).tolist())
    try:
        environment=os.environ.copy();environment.update(YOPO_MINCO_MAX_ACC=str(max_acc),YOPO_MINCO_MAX_VZ=str(max_vz))
        proc=subprocess.run([executable],input=' '.join(map(str,values)),text=True,capture_output=True,timeout=timeout,env=environment)
    except subprocess.TimeoutExpired:
        return dict(accepted=False,reason='optimizer_timeout')
    except OSError as exc:
        return dict(accepted=False,reason='optimizer_unavailable',detail=str(exc))
    if proc.returncode:
        return dict(accepted=False,reason='optimizer_failed',detail=proc.stderr[-200:])
    try:
        lines=proc.stdout.splitlines(); status,pieces=map(int,lines[0].split()); raw=np.asarray([list(map(float,line.split())) for line in lines[1:]])
        coef=raw[:,1:].reshape(pieces,3,6); ds=raw[:,0]; total=ds.sum()
        times=np.linspace(0,total,int(np.ceil(total/.02))+1); ends=np.cumsum(ds)
        ids=np.minimum(np.searchsorted(ends,times,side='right'),pieces-1); local=times-np.r_[0,ends[:-1]][ids]
        def evaluate(order):
            out=np.zeros((len(times),3))
            for power in range(order,6):
                factor=1
                for j in range(order):factor*=power-j
                out+=coef[ids,:,5-power]*(factor*local**(power-order))[:,None]
            return out
        pos,vel,acc=evaluate(0),evaluate(1),evaluate(2)
    except (ValueError,IndexError):
        return dict(accepted=False,reason='invalid_optimizer_output')
    reason=None
    if not np.all(np.isfinite(pos)) or np.max(np.linalg.norm(acc,axis=1))>max_acc+.001 or np.max(np.abs(vel[:,2]))>max_vz+.001:
        reason='dynamic_limit'
    distances=[]
    for nb in neighbors:
        predicted=np.column_stack([np.interp(times,np.arange(len(nb))*neighbor_dt,nb[:,j]) for j in range(3)])
        distances.append(np.linalg.norm(pos-predicted,axis=1))
    if distances and (np.min(distances)<30. or np.max(distances)>80.):reason='pair_distance_hard_band'
    return dict(accepted=reason is None,reason=reason,status=status,samples=pos,velocities=vel,accelerations=acc,times=times,durations=ds,coefficients=coef,backend='Swarm-Formation@967a4bd/MinJerkOpt+lbfgs',pair_min=None if not distances else float(np.min(distances)),pair_max=None if not distances else float(np.max(distances)))

def to_polytraj(result, drone_id, traj_id, start_time):
    """Return native traj_utils/PolyTraj for swarm exchange, never control output."""
    if not result.get('accepted'):raise ValueError('Cannot communicate rejected trajectory')
    from traj_utils.msg import PolyTraj
    msg=PolyTraj();msg.drone_id=drone_id;msg.traj_id=traj_id;msg.start_time=start_time;msg.order=5
    msg.duration=result['durations'].tolist()
    for axis,name in enumerate(('coef_x','coef_y','coef_z')):
        setattr(msg,name,result['coefficients'][:,axis,:].reshape(-1).tolist())
    return msg

def sample_polytraj(msg, relative_times):
    """Decode native exchanged PolyTraj. Reject extrapolation/stale horizons."""
    ds=np.asarray(msg.duration,float);times=np.asarray(relative_times,float)
    if msg.order!=5 or len(ds)==0 or np.any(ds<=0) or np.any(times<0) or np.any(times>ds.sum()+1e-8):
        raise ValueError('Invalid or insufficient neighbor trajectory horizon')
    coef=np.stack([np.asarray(getattr(msg,name),float).reshape(len(ds),6) for name in ('coef_x','coef_y','coef_z')],axis=1)
    ends=np.cumsum(ds);ids=np.minimum(np.searchsorted(ends,times,side='right'),len(ds)-1)
    local=times-np.r_[0,ends[:-1]][ids];points=np.zeros((len(times),3))
    for power in range(6):points+=coef[ids,:,5-power]*local[:,None]**power
    if not np.all(np.isfinite(points)):raise ValueError('Nonfinite neighbor trajectory')
    return points

