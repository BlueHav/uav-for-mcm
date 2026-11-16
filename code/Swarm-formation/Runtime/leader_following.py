"""One cruise guide per group; followers trail measured leader motion by arc length.

These are reference targets, not collision clearance certificates. Every vehicle
retains local obstacle and peer checks; actual group means enforce the 2km limit.
"""
import numpy as np


def group_order(group):
    if group not in (0, 1):raise ValueError('Group must be 0 or 1')
    offset=5*group
    return [offset+2,offset,offset+1,offset+3,offset+4]


class LeaderTrail:
    def __init__(self, spacing=20., max_speed=20., max_sample_gap=1.2):
        if not np.isfinite([spacing,max_speed,max_sample_gap]).all() or spacing<2 or max_speed<=0 or max_sample_gap<=0:
            raise ValueError('Invalid trail spacing or speed bound')
        self.spacing=float(spacing);self.max_speed=float(max_speed)
        self.max_sample_gap=float(max_sample_gap)
        self.points=[];self.stamp=None

    def seed(self, verified_formation_path, stamp):
        """Seed rear-to-front positions only after the initial line has been validated."""
        points=np.asarray(verified_formation_path,float)
        if points.ndim!=2 or points.shape[1]!=3 or len(points)<2 or not np.isfinite(points).all() or not np.isfinite(stamp):
            raise ValueError('Finite rear-to-front initial path required')
        if np.any(np.linalg.norm(np.diff(points,axis=0),axis=1)<1e-6):
            raise ValueError('Duplicate initial path vertices')
        self.points=[p.copy() for p in points];self.stamp=float(stamp)

    def append(self, position, stamp):
        p=np.asarray(position,float)
        if p.shape!=(3,) or not np.isfinite(p).all() or not np.isfinite(stamp):
            raise ValueError('Finite actual leader position and timestamp required')
        if self.stamp is not None:
            dt=float(stamp)-self.stamp
            distance=float(np.linalg.norm(p-self.points[-1]))
            if dt==0 and distance<1e-6:return
            if dt<=0:raise ValueError('Leader odometry is not chronological')
            if dt>self.max_sample_gap:raise ValueError('Leader odometry gap; cannot interpolate an unobserved trajectory')
            if distance>self.max_speed*dt+2:raise ValueError('Discontinuous leader position; revalidate/reset formation')
        if not self.points or np.linalg.norm(p-self.points[-1])>1e-6:self.points.append(p.copy())
        self.stamp=float(stamp)
        # Retain all bends relevant to the four followers, with an extra 100m.
        if len(self.points)>2:
            lengths=np.linalg.norm(np.diff(np.asarray(self.points),axis=0),axis=1)
            remaining=float(lengths.sum());drop=0
            for length in lengths[:-1]:
                if remaining-length<4*self.spacing+100:break
                remaining-=length;drop+=1
            if drop:self.points=self.points[drop:]

    def targets(self, now):
        if not np.isfinite(now) or self.stamp is None or not 0 <= float(now)-self.stamp <= self.max_sample_gap:
            return None
        if len(self.points)<2:return None
        p=np.asarray(self.points)
        lengths=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1))]
        if lengths[-1]<4*self.spacing-1e-8:return None
        out=[]
        for behind in range(5):
            distance=max(0.,lengths[-1]-behind*self.spacing)
            i=min(len(p)-1,max(1,int(np.searchsorted(lengths,distance))))
            t=(distance-lengths[i-1])/(lengths[i]-lengths[i-1])
            out.append(p[i-1]+t*(p[i]-p[i-1]))
        return np.asarray(out)

    def slot(self,rank,now,anticipation=0.):
        """Open one rank only after that much real leader travel is observed."""
        if rank not in range(5):raise ValueError('Invalid follower rank')
        if not np.isfinite(anticipation) or not 0<=anticipation<=2.:raise ValueError('Invalid observed-trail anticipation')
        if not np.isfinite(now) or self.stamp is None or not 0<=now-self.stamp<=self.max_sample_gap or not self.points:return None
        p=np.asarray(self.points);arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1))]
        end=arc[-1]-rank*self.spacing
        if end < -1e-8:return None
        end=min(arc[-1],end+anticipation)
        return np.array([np.interp(max(0.,end),arc,p[:,axis]) for axis in range(3)])

    def slot_direction(self,rank,now,anticipation=0.):
        if self.slot(rank,now,anticipation) is None or len(self.points)<2:return None
        p=np.asarray(self.points);arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1))]
        s=min(arc[-1],max(0.,arc[-1]-rank*self.spacing)+anticipation)
        a=np.array([np.interp(max(0.,s-.5),arc,p[:,axis]) for axis in range(3)])
        b=np.array([np.interp(min(arc[-1],s+.5),arc,p[:,axis]) for axis in range(3)])
        delta=b-a;return delta/max(np.linalg.norm(delta),1e-9)

    def directions(self, now):
        """Local trail tangents at each slot, not the leader's current heading."""
        if self.targets(now) is None:return None
        p=np.asarray(self.points);arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1))]
        result=[]
        for rank in range(5):
            s=max(0.,arc[-1]-rank*self.spacing)
            a=np.array([np.interp(max(0.,s-.5),arc,p[:,axis]) for axis in range(3)])
            b=np.array([np.interp(min(arc[-1],s+.5),arc,p[:,axis]) for axis in range(3)])
            delta=b-a;result.append(delta/max(np.linalg.norm(delta),1e-9))
        return np.asarray(result)

    def route_to_slot(self,position,rank,now,anticipation=0.):
        """Measured trail between the nearest local projection and this slot."""
        target=self.slot(rank,now,anticipation)
        if target is None or len(self.points)<2:return []
        p=np.asarray(self.points);segment=np.diff(p,axis=0);length=np.linalg.norm(segment,axis=1)
        arc=np.r_[0.,np.cumsum(length)];end=min(arc[-1],arc[-1]-rank*self.spacing+anticipation)
        t=np.clip(np.sum((np.asarray(position)-p[:-1])*segment,axis=1)/np.maximum(length**2,1e-12),0,1)
        along=arc[:-1]+t*length;projection=p[:-1]+t[:,None]*segment
        eligible=(along>=max(0.,end-max(40.,2*self.spacing)))&(along<=end+2.)
        if not eligible.any():return []
        costs=np.linalg.norm(projection-position,axis=1);costs[~eligible]=np.inf;i=int(np.argmin(costs))
        if along[i]>=end:return []  # Accepted small overshoot retains forward feed.
        vertices=p[(arc>along[i]+1e-6)&(arc<end-1e-6)]
        return np.vstack([projection[i],vertices,target]).tolist()

def trail_waypoint(route,position):
    """Use the next ordered bend; never shortcut vertices by more than 15 cm."""
    points=np.asarray(route,float).reshape(-1,3)
    if len(points)<2:return None
    p=np.asarray(position);start=points[0];chosen=1
    # A horizon cutoff is not a turn. When every observed vertex lies on one
    # forward chord, track its actual slot directly without an artificial bend
    # speed cap. Reversals and deviations retain the ordered-bend path below.
    chord=points[-1]-start;norm=float(np.dot(chord,chord))
    if norm>1e-12:
        along=(points-start)@chord/norm
        deviation=np.linalg.norm(points-(start+along[:,None]*chord),axis=1)
        if deviation.max()<=.15 and np.all(np.diff(along)>=-1e-9) and along.min()>=-1e-9 and along.max()<=1.+1e-9:
            return points[-1].copy(),False,chord/np.sqrt(norm)
    for j in range(2,len(points)):
        delta=points[j]-start;norm=float(np.dot(delta,delta))
        t=np.clip((points[:j]-start)@delta/max(norm,1e-12),0,1)
        deviation=np.linalg.norm(points[:j]-(start+t[:,None]*delta),axis=1)
        if deviation.max()>.15 or np.linalg.norm(points[j]-start)>6.:break
        chosen=j
    target=points[chosen];tangent=target-start
    # Once within 25 cm of a vertex, move onto its next measured segment.
    if np.linalg.norm(target-p)<.25 and chosen+1<len(points):
        chosen+=1;target=points[chosen];tangent=target-points[chosen-1]
    tangent=tangent/max(np.linalg.norm(tangent),1e-9)
    return target.copy(),chosen<len(points)-1,tangent
