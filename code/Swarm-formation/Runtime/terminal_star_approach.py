"""Synchronized terminal tracking targets; observed-map admission stays local."""
import numpy as np
from moving_merge import polynomial


def certificates_ready(records,token,now):
    return len(records)==5 and all(isinstance(x,dict) and x.get('token')==token and
        x.get('clear') is True and -.1<=now-x.get('stamp',-1e9)<=.35 for x in records)


def boundary_matches(plan,p,v,a):
    actual=[np.asarray(x,float) for x in (p,v,a)]
    return all(x.shape==(5,3) and np.isfinite(x).all() and
        np.linalg.norm(x-y,axis=1).max()<=limit
        for x,y,limit in zip(actual,plan.sample(0),(.25,.15,.2)))


def settled_boundary(p,v,boundary):
    if boundary is None:return False
    p,v=np.asarray(p),np.asarray(v)
    return (np.linalg.norm(p-boundary[0],axis=1).max()<=.25 and
            max(np.linalg.norm(v,axis=1).max(),np.linalg.norm(boundary[1],axis=1).max())<.1
            and np.linalg.norm(boundary[2],axis=1).max()<.1)


class StarApproach:
    def __init__(self,p,v,a,q,token):
        p,v,a,q=[np.asarray(x,float) for x in (p,v,a,q)]
        if any(x.shape!=(5,3) or not np.isfinite(x).all() for x in (p,v,a,q)):
            raise ValueError('Invalid star approach boundary')
        if np.linalg.norm(p[:,:2]-q[:,:2],axis=1).max()>30.:
            raise ValueError('Outside star approach region')
        self.token=token
        for duration in np.arange(max(4.,np.linalg.norm(q-p,axis=1).max()/1.4),46.,1.):
            self.duration=float(duration)
            self.coefficients=polynomial(p,v,a,q,np.zeros_like(v),np.zeros_like(a),duration)
            times=np.linspace(0,duration,int(duration/.05)+2)
            positions,speeds,acc=self.samples(times)
            # Reserve actuator authority and a 0.5 m tracking tube per member.
            if np.linalg.norm(speeds[:,:,:2],axis=2).max()>2.5 or np.abs(speeds[:,:,2]).max()>1. or np.linalg.norm(acc,axis=2).max()>.6:continue
            if any(np.linalg.norm(positions[:,i]-positions[:,j],axis=1).min()<3.8 for i in range(5) for j in range(i)):continue
            return
        raise ValueError('No bounded collision-free star approach')

    def sample(self,elapsed):
        u=float(np.clip(elapsed/self.duration,0.,1.));c=self.coefficients
        return (sum(c[k]*u**k for k in range(6)),
                sum(k*c[k]*u**(k-1) for k in range(1,6))/self.duration,
                sum(k*(k-1)*c[k]*u**(k-2) for k in range(2,6))/self.duration**2)

    def samples(self,times):
        u=np.clip(np.asarray(times)/self.duration,0.,1.)
        c=self.coefficients
        return (np.einsum('tk,kij->tij',np.stack([u**k for k in range(6)],axis=1),c),
                np.einsum('tk,kij->tij',np.stack([k*u**(k-1)/self.duration for k in range(1,6)],axis=1),c[1:]),
                np.einsum('tk,kij->tij',np.stack([k*(k-1)*u**(k-2)/self.duration**2 for k in range(2,6)],axis=1),c[2:]))

    def packet(self):
        return dict(token=self.token,duration=self.duration,coefficients=self.coefficients.tolist())

    @classmethod
    def from_packet(cls,packet):
        obj=cls.__new__(cls);obj.token=str(packet['token']);obj.duration=float(packet['duration'])
        obj.coefficients=np.asarray(packet['coefficients'],float)
        if not 0<obj.duration<=46 or obj.coefficients.shape!=(6,5,3) or not np.isfinite(obj.coefficients).all():
            raise ValueError('Invalid star approach packet')
        return obj


def observed_member_clear(grid,plan,member,position):
    samples=plan.samples(np.linspace(0,plan.duration,int(plan.duration/.05)+2))[0][:,member]
    if not grid.corridor_clear(position,samples[0],radius=.5):return False
    if not np.all(grid.points_clear(samples,radius=1.,check_peers=False)):return False
    goal=samples[-1];ground=grid.ground_at(*goal[:2])
    support=grid.ground_at_many(goal[:2]+np.array([[-.25,-.25],[-.25,.25],[.25,-.25],[.25,.25],[0,0]]))
    if not np.isfinite(ground) or not np.isfinite(support).all() or np.ptp(support)>.15:return False
    column=np.tile(goal,(max(2,int((goal[2]-ground)/.15)+1),1))
    column[:,2]=np.linspace(ground+.65,goal[2],len(column))
    return bool(np.all(grid.points_clear(column,radius=.35,check_peers=False)))
