"""Check the geometry actually consumed by a group executor, not only its config."""
import numpy as np

def terminal_distance_problem(model):
    region=model.config.get('assembly_region')
    if region is None:return None
    minimum=float(region['minimum_endpoint_distance_m'])
    if not np.isfinite(minimum) or minimum<1000.:return 'Invalid terminal minimum distance'
    for group in model.layout.group_ids:
        targets=np.asarray(model.descent(group)[1],float)
        if targets.shape!=(5,3) or not np.isfinite(targets).all():return 'Invalid actual descent targets'
        distance=np.linalg.norm(targets[:,:2]-model.ends[group],axis=1)
        if np.min(distance)<minimum:
            return 'Actual group %d descent targets do not implement configured >= %.0f m terminal distance'%(group,minimum)
    return None
