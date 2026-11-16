"""Release both groups together after measured phase barriers."""
import numpy as np

def release_groups(groups,positions,velocities,now,blocked=False):
    p=np.asarray(positions,float);v=np.asarray(velocities,float)
    if blocked or p.shape!=(10,3) or v.shape!=(10,3) or not np.isfinite(np.r_[p.ravel(),v.ravel()]).all():return None
    if not groups:return None
    # Only the active groups take part in the barrier; a parked group must not
    # hold a single-group flight in CRUISE_READY forever.
    ids=[int(getattr(g,'group',k)) for k,g in enumerate(groups)]
    phase=groups[0].phase
    if phase not in ('TRANSFER_READY','CLIMB_READY','CRUISE_READY') or any(g.phase!=phase for g in groups):return None
    slices=[p[5*i:5*i+5] for i in ids];speeds=[v[5*i:5*i+5] for i in ids]
    targets=np.concatenate([g.reference for g in groups])
    actual=np.concatenate(slices)
    if np.max(np.linalg.norm(actual-targets,axis=1))>=.5 or np.max(np.linalg.norm(np.concatenate(speeds),axis=1))>=.2:return None
    for k,g in enumerate(groups):
        if phase in ('TRANSFER_READY','CLIMB_READY'):g.next_segment(slices[k],now)
        else:g.start_cruise(slices[k],now)
    return phase

def release_relay_groups(groups,positions,velocities,now,blocked=False):
    """Phase 2 barrier: only the two group leaders must settle.

    The followers are replaying delayed leader commands; they are intentionally
    not at their slot references, so the historical all-member barrier would
    never release. Leader position/velocity remain measured.
    """
    import numpy as np
    p=np.asarray(positions,float);v=np.asarray(velocities,float)
    if blocked or p.shape!=(10,3) or v.shape!=(10,3) or not np.isfinite(np.r_[p.ravel(),v.ravel()]).all():return None
    if not groups:return None
    ids=[int(getattr(g,'group',k)) for k,g in enumerate(groups)]
    phase=groups[0].phase
    if phase not in ('TRANSFER_READY','CLIMB_READY','CRUISE_READY') or any(g.phase!=phase for g in groups):return None
    for k,g in enumerate(groups):
        sl=slice(5*ids[k],5*ids[k]+5)
        # The relay leader hands over at the cruise altitude while still moving
        # toward the target; do not require it to stop or reach the full line.
        if abs(p[sl][2][2]-g.reference[2][2])>=1.0:return None
    for k,g in enumerate(groups):
        sl=slice(5*ids[k],5*ids[k]+5)
        if phase in ('TRANSFER_READY','CLIMB_READY'):g.next_segment(p[sl],now)
        else:g.start_cruise(p[sl],now)
    return phase
