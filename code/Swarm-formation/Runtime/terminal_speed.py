"""Bounded recovery of cruising speed; smaller safety limits apply immediately."""
import math
import numpy as np


def bounded_speed(previous,target,dt,recovery_acc=.6):
    values=(previous,target,dt,recovery_acc)
    if not all(math.isfinite(x) for x in values) or min(values)<0:
        raise ValueError('Invalid continuous speed state')
    return min(target,previous+recovery_acc*min(dt,.25))


def select_continuous_velocity(desired,clear):
    desired=np.asarray(desired,float)
    if desired.shape!=(3,) or not np.isfinite(desired).all():raise ValueError('Invalid velocity')
    for scale in (1.,.85,.7,.5,.35,.25):
        candidate=desired.copy();candidate[:2]*=scale
        if clear(candidate):return candidate
    return None


def stable_candidates(candidates,previous,gain=.25):
    ranked=sorted(candidates,key=lambda x:x[0],reverse=True)
    if not ranked or previous is None:return ranked
    near=[x for x in ranked if float(np.dot(x[1],previous))>.99]
    if near and ranked[0][0]-near[0][0]<=gain:
        preferred=near[0]
        return [preferred]+[x for x in ranked if x is not preferred]
    return ranked
