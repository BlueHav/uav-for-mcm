"""Finite joint terminal packets; no ROS and no command publisher.

Peer certificates compare complete spatial stopping boxes, including a member
that remains at the start. Environment certification is performed separately by
every member using its own observed image/ground map.
"""
import hashlib
import json
import itertools
import numpy as np
from swarm_native_bridge import NativeTrajectory

PACKET_CLEARANCE=2.4
TRACKING_LIMIT=.15
SAMPLE_PADDING=.06

def _digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()

def make_packet(run,epoch,serial,group,plans,stamp,envelopes=None,corridors=None,flexible=False,yopo_reference_members=None):
    if epoch!=2 or group not in (0,1) or len(plans)!=5:raise ValueError('Terminal ownership/shape')
    value=dict(run_id=run,epoch=epoch,serial=serial,group=group,created=float(stamp),
               durations=[p.durations.tolist() for p in plans],coefficients=[p.coefficients.tolist() for p in plans])
    if flexible:value['flexible']=True
    if yopo_reference_members is not None:value['yopo_reference_members']=list(yopo_reference_members)
    nominal=[getattr(p,'nominal_duration',None) for p in plans]
    if any(t is not None for t in nominal):
        if any(t!=nominal[0] for t in nominal) or not .5<=nominal[0]<plans[0].duration:raise ValueError('Invalid nominal horizon')
        value['nominal_duration']=float(nominal[0])
    if envelopes is not None:value['envelopes']=[np.asarray(x).tolist() for x in envelopes]
    if corridors is None and any(hasattr(p,'stop_corridor') for p in plans):corridors=[p.stop_corridor for p in plans]
    if corridors is not None:value['corridors']=[np.asarray(x).tolist() for x in corridors]
    value['digest']=_digest(value)
    return value

def packet_paths(packet):
    data=dict(packet);digest=data.pop('digest',None)
    if digest!=_digest(data):raise ValueError('Joint packet digest mismatch')
    if 'flexible' in data and data['flexible'] is not True:raise ValueError('Invalid avoidance mode')
    if data.get('epoch')!=2 or data.get('group') not in (0,1):raise ValueError('Joint packet ownership')
    if len(data['durations'])!=5 or len(data['coefficients'])!=5:raise ValueError('Five trajectories required')
    plans=[NativeTrajectory(t,c,0.) for t,c in zip(data['durations'],data['coefficients'])]
    if max(p.duration for p in plans)-min(p.duration for p in plans)>1e-6:raise ValueError('Unshared timeline')
    if not .5<=plans[0].duration<=15:raise ValueError('Unbounded terminal horizon')
    nominal=data.get('nominal_duration')
    if nominal is not None:
        if not np.isfinite(nominal) or not .5<=nominal<plans[0].duration:raise ValueError('Invalid nominal horizon')
        for p in plans:
            if not np.any(np.isclose(np.cumsum(p.durations)[:-1],nominal,atol=1e-7,rtol=0)):
                raise ValueError('Nominal/backup boundary must be a polynomial join')
            p.nominal_duration=float(nominal)
    if 'corridors' in data:
        if len(data['corridors'])!=5:raise ValueError('Five stopping corridors required')
        for i,p in enumerate(plans):
            c=np.asarray(data['corridors'][i],float);box=np.asarray(data['envelopes'][i],float)
            if (c.ndim!=3 or c.shape[1:]!=(8,3) or not 1<=len(c)<=76 or not np.isfinite(c).all()
                    or np.any(c<box.min(0)-1e-7) or np.any(c>box.max(0)+1e-7)):
                raise ValueError('Invalid stopping corridor')
            p.stop_corridor=c
    return plans

class PacketGate:
    def __init__(self,packet):self.packet=packet;self.acks={}
    def ack(self,member,digest,accepted,stamp):
        if member not in range(5) or digest!=self.packet['digest']:return
        self.acks[member]=(bool(accepted),float(stamp))
    def ready(self,now):
        return len(self.acks)==5 and all(ok and -.05<=now-stamp<=.35 for ok,stamp in self.acks.values())

def _smooth_batch(v,a,target,dt,vertical_acc,vertical_jerk):
    jerk=36.*(target-v)-12.*a
    n=np.linalg.norm(jerk[:,:2],axis=1)
    next_a=a.copy();next_a[:,:2]+=jerk[:,:2]*np.minimum(1.,3./np.maximum(n,1e-12))[:,None]*dt
    n=np.linalg.norm(next_a[:,:2],axis=1)
    next_a[:,:2]*=np.minimum(1.,2.2/np.maximum(n,1e-12))[:,None]
    next_a[:,2]+=np.clip(jerk[:,2],-vertical_jerk,vertical_jerk)*dt
    bounded=np.clip(next_a[:,2],-vertical_acc,vertical_acc)
    next_a[:,2]=np.clip(bounded,a[:,2]-vertical_jerk*dt,a[:,2]+vertical_jerk*dt)
    out=v+(a+next_a)*dt*.5;out[:,2]=np.clip(out[:,2],-7.,7.)
    return out,next_a

def _corners(lo,hi):return np.array(list(itertools.product(*zip(lo,hi))))

def tracking_reservation(plan):
    """Pair the measured velocity tolerance with its reserved stopping padding."""
    times=np.linspace(0,plan.duration,max(2,int(np.ceil(plan.duration/.02))+1))
    samples=plan.samples(times)
    vmax=np.linalg.norm(samples[:,1],axis=1).max();amax=np.linalg.norm(samples[:,2],axis=1).max()
    if vmax>6.001 or amax>2.151:raise ValueError('Packet exceeds certified dynamics')
    return (.10,SAMPLE_PADDING) if vmax<=1.201 and amax<=1.201 else (.25,1.25)

def stop_clouds(plans,corridors=False):
    """Bound all 50 Hz abort states, delayed commands, and both existing brakes.

    Packets are bounded to 6 m/s and 2.15 m/s². Slow packets keep 6 cm
    padding; faster packets reserve 1.25 m for sampled abort/stop states and
    measured velocity error up to 0.25 m/s. The actual stopping integration
    remains mandatory before and during execution; padding alone cannot ACK.
    The independent
    2.4 m box clearance then reserves 15 cm tracking error for each member.
    Boxes also contain every nominal segment, so arbitrary execution offsets
    and non-execution are checked without extrapolating a peer trajectory.
    """
    boxes=[]
    for plan in plans:
        times=np.linspace(0,plan.duration,max(2,int(np.ceil(plan.duration/.02))+1))
        samples=plan.samples(times)
        p,v,a=samples[:,0],samples[:,1],samples[:,2]
        vmax=np.linalg.norm(v,axis=1).max();amax=np.linalg.norm(a,axis=1).max()
        if vmax>6.001 or amax>2.151:raise ValueError('Packet exceeds certified dynamics')
        _,padding=tracking_reservation(plan)
        if np.linalg.norm(v[-1])>.001 or np.linalg.norm(a[-1])>.001:raise ValueError('Packet must end at rest')
        indices=np.arange(0,len(p),12)
        lower=lambda q:np.minimum.reduceat(q,indices,axis=0) if corridors else q.min(0)
        upper=lambda q:np.maximum.reduceat(q,indices,axis=0) if corridors else q.max(0)
        lo,hi=lower(p),upper(p)
        for delay in (0,15):
            for watchdog in (False,True):
                q=p.copy();w=v.copy();b=a.copy();drive=v.copy()
                for step in range(600):
                    driving=step<delay;target=drive if driving else np.zeros_like(w)
                    slow=watchdog and not driving
                    new,b=_smooth_batch(w,b,target,.02,1.2 if slow else 2.5,6. if slow else 8.)
                    q+=(w+new)*.01;w=new
                    lo=np.minimum(lo,lower(q));hi=np.maximum(hi,upper(q))
                    if not driving and np.linalg.norm(w,axis=1).max()<.005 and np.linalg.norm(b,axis=1).max()<.01:break
                else:raise ValueError('Incomplete terminal stopping envelope')
        boxes.append(np.array([_corners(a-padding,b+padding) for a,b in zip(lo,hi)]) if corridors else _corners(lo-padding,hi+padding))
    return boxes

def spatially_separated(clouds,clearance=PACKET_CLEARANCE):
    """Exact distance between conservative axis-aligned spatial boxes."""
    boxes=[]
    for cloud in clouds:
        points=np.asarray(cloud,float).reshape(-1,3)
        if not len(points) or not np.isfinite(points).all():return False,0.
        boxes.append((points.min(0),points.max(0)))
    minimum=float('inf')
    for i,(lo,hi) in enumerate(boxes):
        for other_lo,other_hi in boxes[:i]:
            gap=np.maximum(np.maximum(lo-other_hi,other_lo-hi),0.)
            minimum=min(minimum,float(np.linalg.norm(gap)))
    return minimum>=clearance,minimum

def environment_clear(grid,box):
    """Cover the whole stopping box, not just its corners; no unknown bypass."""
    box=np.asarray(box,float);lo=box.min(0);hi=box.max(0)
    if not np.isfinite(box).all():return False
    if all(hasattr(grid,k) for k in ('state','origin','resolution','shape','ground','FREE')):
        if grid.origin is None:return False
        # Conservative cube dilation includes the entire former sphere sweep.
        # Every intersecting native voxel and every bilinear ground vertex is
        # checked, including right/top interpolation vertices.
        lower=lo-.5;upper=hi+.5
        ij=np.floor(lower/grid.resolution).astype(int)-grid.origin
        end=np.floor(upper/grid.resolution).astype(int)-grid.origin
        if (ij<0).any() or (end>=grid.shape).any() or (end[:2]+1>=grid.ground.shape).any():return False
        cells=tuple(slice(int(a),int(b)+1) for a,b in zip(ij,end))
        ground=grid.ground[ij[0]:end[0]+2,ij[1]:end[1]+2]
        return bool(np.all(grid.state[cells]==grid.FREE) and np.isfinite(ground).all() and ground.max()<lower[2])
    axes=[np.linspace(a,b,max(2,int(np.ceil((b-a)/.15))+1)) for a,b in zip(lo,hi)]
    if np.prod([len(a) for a in axes])>100000:return False
    points=np.array(list(itertools.product(*axes)))
    return bool(grid.points_clear(points,radius=.5,check_peers=False).all())
