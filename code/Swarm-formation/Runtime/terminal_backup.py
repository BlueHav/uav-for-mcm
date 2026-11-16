"""Certified finite backup appended to a continuing joint nominal trajectory."""
import numpy as np
from swarm_native_bridge import NativeTrajectory
from terminal_formation import tracking_reservation


def append_backup(plans):
    if len(plans)!=5:raise ValueError('Five nominal trajectories required')
    duration=plans[0].duration
    if any(abs(p.duration-duration)>1e-7 for p in plans) or duration>=15.:
        raise ValueError('No finite backup horizon')
    tail=np.array([p.sample(duration) for p in plans])
    speed=np.linalg.norm(tail[:,1],axis=1).max()
    # A quintic boundary solve preserves P/V/A. Its duration is increased
    # until the same independent dynamics gate as execution accepts it.
    brake=max(2.,1.6*speed/2.1)
    while duration+brake<=15.+1e-8:
        output=[]
        for plan,(p,v,a) in zip(plans,tail):
            goal=p+v*brake*.5+a*brake*brake/12.
            matrix=np.array([[brake**3,brake**4,brake**5],
                             [3*brake**2,4*brake**3,5*brake**4],
                             [6*brake,12*brake**2,20*brake**3]])
            high=np.linalg.solve(matrix,np.array([goal-p-v*brake-a*brake**2/2.,-v-a*brake,-a]))
            coeff=np.column_stack((high[::-1].T,a/2.,v,p))
            joined=NativeTrajectory(np.r_[plan.durations,brake],np.concatenate((plan.coefficients,coeff[None])),plan.milliseconds)
            joined.nominal_duration=duration
            output.append(joined)
        try:
            for p in output:tracking_reservation(p)
            return output
        except ValueError:brake+=.25
    raise ValueError('No dynamically feasible finite backup within horizon')
