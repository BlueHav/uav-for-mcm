"""Pure contracts for the current static Unity field (metres, ROS world frame)."""
import numpy as np

class SceneSafety:
    def __init__(self, layout, radius=0.6):
        self.bounds=layout['bounds']; self.radius=radius
        objects=layout['objects']
        self.terrain=layout.get('terrain')
        if self.terrain:
            self.grid=np.asarray(self.terrain['heights']).reshape(self.terrain['height'],self.terrain['width'])
            self.dx=(self.bounds['x_max']-self.bounds['x_min'])/(self.grid.shape[1]-1)
            self.dy=(self.bounds['y_max']-self.bounds['y_min'])/(self.grid.shape[0]-1)
        self.xy=np.array([[o['x'],o['y']] for o in objects],dtype=float).reshape(-1,2)
        self.r=np.array([o['radius']+radius for o in objects])
        self.box=np.array([o['kind']=='building' and o.get('width',0)>0 and o.get('depth',0)>0 for o in objects],dtype=bool)
        self.half=np.array([[o.get('width',0)/2+radius+(0 if o.get('model') else .2),o.get('depth',0)/2+radius+(0 if o.get('model') else .2)] for o in objects]).reshape(-1,2)
        self.h=np.array([o.get('base_z',0)+o['height']+(0.3 if o['kind']=='building' and not o.get('model') else 0)+radius for o in objects])
    def terrain_clear(self,points,radius=None):
        # Upper envelope of every grid cell touched by a square footprint.
        # Conservative on steep slopes; never interpolates across a hidden ridge.
        if self.terrain is None:return True
        p=np.asarray(points,dtype=float).reshape(-1,3)
        r=self.radius if radius is None else radius
        u=(p[:,0]-self.bounds['x_min'])/self.dx
        v=(self.bounds['y_max']-p[:,1])/self.dy
        c0=np.floor(u-r/self.dx).astype(int);c1=np.floor(u+r/self.dx).astype(int)+1
        r0=np.floor(v-r/self.dy).astype(int);r1=np.floor(v+r/self.dy).astype(int)+1
        top=np.full(len(p),-np.inf)
        for dc in range(int(np.max(c1-c0,initial=0))+1):
            ci=np.clip(np.minimum(c0+dc,c1),0,self.grid.shape[1]-1)
            for dr in range(int(np.max(r1-r0,initial=0))+1):
                ri=np.clip(np.minimum(r0+dr,r1),0,self.grid.shape[0]-1)
                top=np.maximum(top,self.grid[ri,ci])
        return bool(np.all(p[:,2]>top+r))
    def points_clear(self, points):
        p=np.asarray(points,dtype=float).reshape(-1,3)
        if not np.isfinite(p).all(): return False
        b=self.bounds; r=self.radius
        if any(np.any(p[:,i]<b[k+'_min']+r) or np.any(p[:,i]>b[k+'_max']-r) for i,k in enumerate('xyz')): return False
        if not self.terrain_clear(p):return False
        if len(self.xy):
            ds=((p[:,None,:2]-self.xy[None,:,:])**2).sum(axis=2)
            inside=ds<=self.r[None,:]**2
            inside[:,self.box]=np.all(np.abs(p[:,None,:2]-self.xy[None,:,:])[:,self.box,:]<=self.half[None,self.box,:],axis=2)
            if np.any(inside&(p[:,None,2]<=self.h[None,:])):return False
        return True
    def segment_clear(self, start, end):
        a,b=np.asarray(start,dtype=float),np.asarray(end,dtype=float)
        if not self.points_clear([a,b]):return False
        if self.terrain is not None:
            # Every segment is covered by overlapping 3D boxes centred on samples.
            count=max(1,int(np.ceil(np.linalg.norm(b-a)/.5)))
            points=a+np.linspace(0,1,count+1)[:,None]*(b-a)
            margin=np.linalg.norm(b-a)/count/2
            if not self.terrain_clear(points,self.radius+margin):return False
        if not len(self.xy):return True
        delta=b-a;lo=np.zeros(len(self.h));hi=np.ones(len(self.h))
        if abs(delta[2])<1e-12:
            valid=a[2]<=self.h
        else:
            crossing=(self.h-a[2])/delta[2]
            if delta[2]>0:hi=np.minimum(hi,crossing)
            else:lo=np.maximum(lo,crossing)
            valid=lo<=hi
        # Axis-aligned building boxes leave actual streets traversable; trees keep cylinders.
        if self.box.any():
            lower=np.c_[self.xy[self.box]-self.half[self.box],np.full(self.box.sum(),-np.inf)]
            upper=np.c_[self.xy[self.box]+self.half[self.box],self.h[self.box]]
            enter=np.zeros(self.box.sum());leave=np.ones(self.box.sum());ok=np.ones(self.box.sum(),dtype=bool)
            for axis in range(3):
                if abs(delta[axis])<1e-12:ok&=(a[axis]>=lower[:,axis])&(a[axis]<=upper[:,axis])
                else:
                    t0=(lower[:,axis]-a[axis])/delta[axis];t1=(upper[:,axis]-a[axis])/delta[axis]
                    enter=np.maximum(enter,np.minimum(t0,t1));leave=np.minimum(leave,np.maximum(t0,t1))
            if np.any(ok&(enter<=leave)):return False
        valid&=~self.box
        length2=np.dot(delta[:2],delta[:2])
        t=np.zeros(len(self.h)) if length2<1e-12 else ((self.xy-a[:2])*delta[:2]).sum(axis=1)/length2
        t=np.maximum(lo,np.minimum(hi,t))
        closest=a[:2]+t[:,None]*delta[:2]
        return not np.any(valid&(((closest-self.xy)**2).sum(axis=1)<=self.r**2))

def validate_goal(xyz,frame,scene):
    if frame!='world':raise ValueError('Goal frame must be world')
    p=np.asarray(xyz,dtype=float)
    if p.shape!=(3,) or not scene.points_clear([p]):
        raise ValueError('Goal is nonfinite, outside flight clearance, or inside a static obstacle')
    return p.copy()

class ArrivalGate:
    def __init__(self,tolerance=0.5,speed=0.35,dwell=1.0):
        self.tolerance=tolerance;self.speed=speed;self.dwell=dwell;self.since=None
    def update(self,pos,vel,goal,now):
        valid=np.linalg.norm(np.asarray(pos)-goal)<=self.tolerance and np.linalg.norm(vel)<=self.speed
        if not valid:self.since=None;return False
        if self.since is None:self.since=now
        return now-self.since>=self.dwell

def decode_depth(raw,height,width,step,encoding,bigendian):
    if encoding not in ('16UC1','32FC1'):raise ValueError('Unsupported depth encoding')
    size=2 if encoding=='16UC1' else 4
    if height<=0 or width<=0 or step<width*size or len(raw)<height*step:raise ValueError('Malformed depth payload')
    dtype=np.dtype(('>' if bigendian else '<')+('u2' if size==2 else 'f4'))
    value=np.ndarray((height,width),dtype=dtype,buffer=raw,strides=(step,size)).astype(np.float32)
    if size==2:value/=1000.0
    value[~np.isfinite(value)|(value<=0)]=20.0
    return np.clip(value,0.04,20)
