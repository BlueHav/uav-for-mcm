"""Measured horizontal straight-cruise evidence, independent of requested speed."""
from collections import deque
import math
import numpy as np
from cloud_config import CRUISE_MINIMUM,CRUISE_SECONDS

class CruiseEvidence:
    def __init__(self,minimum=CRUISE_MINIMUM,seconds=CRUISE_SECONDS):
        self.minimum=minimum;self.seconds=seconds
        self.members={i:dict(window=deque(),last=None,since=None,heading=None,best=0.,peak=0.,mean=0.) for i in range(10)}
    def update(self,i,stamp,velocity,cruising):
        s=self.members[i]
        if s['last'] is not None and stamp<=s['last']:return
        gap=0. if s['last'] is None else stamp-s['last'];s['last']=stamp
        if not cruising or gap>.3:
            s['window'].clear();s['since']=None;s['heading']=None;return
        v=np.asarray(velocity,float)[:2];speed=float(np.linalg.norm(v));heading=math.atan2(v[1],v[0])
        s['window'].append((stamp,speed))
        cutoff=stamp-1.
        # Keep the sample bracketing the beginning of the one-second window.
        # Discarding it made valid 5 Hz telemetry appear to have only .8 s of
        # coverage, repeatedly resetting an otherwise continuous fast segment.
        while len(s['window'])>2 and s['window'][1][0]<=cutoff:s['window'].popleft()
        samples=np.asarray(s['window'],float)
        covered=stamp-samples[0,0]>=1.-1e-8
        if covered:
            inside=samples[:,0]>cutoff
            times=np.r_[cutoff,samples[inside,0]]
            speeds=np.r_[np.interp(cutoff,samples[:,0],samples[:,1]),samples[inside,1]]
            mean=float(np.sum(np.diff(times)*(speeds[:-1]+speeds[1:])*.5))
        else:mean=float(samples[:,1].mean())
        s['mean']=mean;s['peak']=max(s['peak'],speed)
        ready=covered and mean>=self.minimum and speed>=self.minimum-.2
        turning=s['heading'] is not None and abs(math.atan2(math.sin(heading-s['heading']),math.cos(heading-s['heading'])))>math.radians(5)
        if not ready:s['since']=None;s['heading']=None
        elif s['since'] is None or turning:s['since']=stamp;s['heading']=heading
        else:s['best']=max(s['best'],stamp-s['since'])
    def report(self,ids=None):
        members={i:dict(best_straight_seconds=s['best'],speed_1s=s['mean'],peak_speed=s['peak'],passed=s['best']>=self.seconds) for i,s in self.members.items()}
        selected=[i for i in (sorted(ids) if ids is not None else sorted(members)) if i in members]
        # Inactive aircraft in a fixed ten-body scene never cruise; only the
        # explicitly active members can carry the cruise requirement.
        return dict(minimum_horizontal_mps=self.minimum,required_straight_seconds=self.seconds,heading_tolerance_degrees=5.,checked_members=selected,passed=bool(selected) and all(members[i]['passed'] for i in selected),members=members)
