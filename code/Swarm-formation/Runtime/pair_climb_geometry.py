"""Two outward, forward climb corridors; no scheduling priority is implied."""
import numpy as np

def pair_climb_geometry(model):
    starts=model.spawn()[[2,7]].copy()
    starts[:,2]=model.ground(starts[:,:2])+2.
    targets=starts.copy()
    targets[:,:2]+=200.*model.direction+np.array([-1.,1.])[:,None]*105.*model.normal
    targets[:,2]=270.
    if not model.sweep_clear(starts,targets):raise ValueError('Pair climb terrain/building sweep failed')
    # The lateral component increases away from the separating plane for each
    # independent progress variable. Its exact lower bound is the initial width.
    lateral=np.dot(targets[:,:2]-starts[:,:2],model.normal)
    width=float(np.dot(starts[1,:2]-starts[0,:2],model.normal))
    if width<=0 or lateral[0]>=0 or lateral[1]<=0:raise ValueError('Pair corridors cross lateral order')
    worst=starts.copy();worst[:,2]=model.ground(worst[:,:2])+1.75
    inactive=model.spawn()[[0,1,3,4,5,6,8,9]];clearances=[]
    for a,b in zip(worst,targets):
        delta=b-a;progress=np.clip((inactive-a)@delta/(delta@delta),0.,1.)
        clearances.append(float(np.min(np.linalg.norm(a+progress[:,None]*delta-inactive,axis=1))))
    if min(clearances)<3.2 or not model.sweep_clear(worst,targets):raise ValueError('Pair climb inactive/low-hover clearance insufficient')
    active_limit=float(np.ceil(max(np.linalg.norm(targets-worst,axis=1))/.8+20.))
    distance=float(np.linalg.norm(targets[1]-targets[0]))
    if distance<220.:raise ValueError('Pair climb target spacing insufficient')
    return dict(starts=starts.tolist(),targets=targets.tolist(),target_distance=distance,
                lengths=np.linalg.norm(targets-starts,axis=1).tolist(),
                continuous_asynchronous_distance_lower_bound=width,
                inactive_minimum_distances=clearances,minimum_start_agl=1.75,climb_active_limit=active_limit,
                forward_m=200.,outward_m=105.,speed_mps=.8,
                completion_xy_tolerance_m=.5,completion_z_tolerance_m=.3,
                completion_speed_mps=.2,completion_stable_seconds=2.)
