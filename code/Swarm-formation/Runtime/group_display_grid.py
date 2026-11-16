"""Bounded map-display sampling; never a sensor or planner input."""
import numpy as np
from display_sampling import display_grid

DISPLAY_RADIUS_M=1000.
DENSE_RADIUS_M=100.

def group_display_grid(focus):
    focus=np.asarray(focus,float).reshape(-1,2)
    if not len(focus) or not np.isfinite(focus).all():
        raise ValueError('Fresh finite group positions required')
    batches=[display_grid(p,[p],radius=DISPLAY_RADIUS_M,near_radius=DENSE_RADIUS_M,near_step=2.,far_step=20.) for p in focus]
    points=np.unique(np.concatenate([p for p,_ in batches]),axis=0)
    dense=np.zeros(len(points),bool)
    for p in focus:
        dense |= np.linalg.norm(points-p,axis=1)<=DENSE_RADIUS_M+1e-8
    return points,dense
