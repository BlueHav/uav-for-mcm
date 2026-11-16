"""Latest point mission geometry; map terrain guides leaders, never certifies local flight."""
import hashlib
import json
from pathlib import Path
import numpy as np
from display_sampling import world_height
from formation_core import transition
from leader_following import group_order
from mission_core import BODY_OFFSET
from active_layout import ActiveLayout
from assembly_region import validate_assembly_region

SQUARE=np.array([[-3.3,-3.3],[3.3,-3.3],[0,0],[-3.3,3.3],[3.3,3.3]])
STAR=np.array([[0,-3.3],[-3.3,0],[0,0],[3.3,0],[0,3.3]])

class PointMission:
    def __init__(self, path):
        self.path=Path(path).resolve();self.config=json.loads(self.path.read_text(encoding='utf-8-sig'))
        c=self.config
        self.layout=ActiveLayout(c.get('execution',{}).get('mode','ten'))
        mp=Path(c['paths']['map'])
        if not mp.is_absolute():
            # Migrated manifests retain repository-relative config/maps paths.
            base=self.path.parent.parent if mp.parts[0]=='config' and self.path.parent.name=='config' else self.path.parent
            mp=base/mp
        self.hashes={'point':hashlib.sha256(self.path.read_bytes()).hexdigest()}
        for key in ('world','metadata'):
            p=mp/c['paths']['map_files'][key];digest=hashlib.sha256(p.read_bytes()).hexdigest()
            if digest!=c['map_fingerprints'][key]:raise ValueError('Map fingerprint mismatch: '+key)
            self.hashes[key]=digest
        self.world=json.loads((mp/c['paths']['map_files']['world']).read_text())
        def xy(name):return np.array([c['points'][name][k] for k in ('x','y')],float)
        self.start=xy('start');self.arrival=xy('landing');self.ends=np.array([xy('end'),xy('end2')])
        self.altitude=float(c['cruise_altitude']);self.assembly_agl=float(c['assembly_agl'])
        self.arrival_agl=float(c['low_arrival_agl']);self.end_agl=float(c['terminal_end_agl'])
        if not np.isfinite(np.r_[self.start,self.arrival,self.ends.ravel(),self.altitude,self.assembly_agl,self.arrival_agl,self.end_agl]).all():raise ValueError('Nonfinite mission')
        if not 0<self.altitude<=300 or not 0<np.linalg.norm(self.ends[0]-self.ends[1])<=1000:raise ValueError('Mission altitude/endpoints outside limits')
        self.direction=self.arrival-self.start;self.direction/=np.linalg.norm(self.direction)
        self.normal=np.array([-self.direction[1],self.direction[0]])
        self.routes=[np.array([[q['x'],q['y']] for q in r]) for r in c['plan']['cruise_routes']]
        if len(self.routes)!=2 or any(not np.allclose(r[[0,-1]],[self.start,self.arrival],atol=.05) for r in self.routes):raise ValueError('Cruise intent endpoints mismatch')
        self.speed=6.;self.group_delay=5.;self.stable_seconds=2.;self.spacing=20.
        # Low-altitude assembly handoffs come from the user-authorized region
        # (>=1 km from each endpoint, inside the landing area). Without them the
        # old code assembled at the landing centre, only ~766 m from an endpoint.
        self.region=validate_assembly_region(c,self.world) if c.get('assembly_region') else None
        self.handoffs=np.asarray(self.region['handoffs'],float) if self.region is not None else None

    def ground(self,xy):return world_height(self.world,xy)

    def offset(self,g):return self.normal*(6. if g==0 else -6.)

    def spawn(self):
        if getattr(self,'forward_departure',False):
            xy=self.start+(np.arange(10)-4.5)[:,None]*3.3*self.normal
            return np.c_[xy,self.ground(xy)+BODY_OFFSET]
        xy=np.concatenate([self.start+self.offset(g)+SQUARE for g in range(2)])
        return np.c_[xy,self.ground(xy)+BODY_OFFSET]

    def assembly(self,g):
        # Preserve row order in the lateral axis; pairs fan forward/backward
        # only after vertical takeoff, without crossing the leader's slot.
        slots=np.array([[-3.3,3.3],[-3.3,-3.3],[0.,0.],[3.3,3.3],[3.3,-3.3]])
        xy=self.start-self.offset(g)+slots[:,0,None]*self.normal+slots[:,1,None]*self.direction
        return np.c_[xy,self.ground(xy)+self.assembly_agl]

    def landing_slots(self,g):
        xy=self.ends[g]+STAR
        return np.c_[xy,self.ground(xy)+BODY_OFFSET]

    def line(self,g,leader,z,direction=None):
        d=self.direction if direction is None else direction
        p=np.zeros((5,3))
        for rank,i in enumerate(group_order(g)):
            p[i%5,:2]=leader-rank*self.spacing*d;p[i%5,2]=z
        return p

    def departure(self,g,before):
        if getattr(self,'forward_departure',False):
            # Phase 2: the leader must climb forward+up so it actually vacates
            # the square slot; a purely vertical climb makes every delayed
            # follower stack in the same column.
            destination=np.asarray(before,float).copy()
            destination[:,:2]+=100.*self.direction
            destination[:,2]=self.altitude
            return [('CLIMB',destination)]
        d=self.routes[g][1]-self.start;d=d/np.linalg.norm(d)
        synchronized=getattr(self,'synchronized_climb',False)
        if synchronized:d=self.direction
        offset=-self.offset(g) if synchronized else self.offset(g)
        xy=self.start+offset+100*d
        low=self.line(g,xy,0,d)
        # Validate an initial line before any trail history can be seeded.
        swept=np.concatenate([np.linspace(a,b,101)[:,:2] for a,b in zip(before,low)])
        z=float(np.max(self.ground(swept)))+15
        if synchronized:
            # Parallel histories avoid the crossing of the two original rays.
            # Put A on its outgoing side and exchange lanes only in disjoint
            # altitude bands during the earlier formation transfer.
            samples=[swept]
            for other in (0,1):
                source=np.asarray(before) if other==g else self.spawn()[5*other:5*other+5]
                dest=self.line(other,self.start-self.offset(other)+100*d,0.,d)
                samples.extend(np.linspace(a,b,101)[:,:2] for a,b in zip(source,dest))
            z=float(np.max(self.ground(np.concatenate(samples))))+15.+16.*g
        low[:,2]=z
        if z+12>self.altitude:raise ValueError('No initial line transition below cruise altitude')
        lift,transfer,settle=transition(before,low)
        climb=self.line(g,xy+100*d,self.altitude,d)
        return [('ALIGN_LIFT',lift),('ALIGN_MOVE',transfer),('ALIGN_SETTLE',settle),('CLIMB',climb)]

    def descent(self,g):
        path=self.descent_path(g)
        return path[0],path[-1]

    def assembly_anchor(self,g):
        """Low-altitude star assembly point for one group."""
        if getattr(self,'handoffs',None) is not None:return np.asarray(self.handoffs[g],float).copy()
        return self.arrival+self.offset(g)

    def descent_path(self,g):
        anchor=self.assembly_anchor(g)
        d=self.arrival-self.routes[g][-2];d/=np.linalg.norm(d)
        endxy=anchor+STAR
        end=np.c_[endxy,self.ground(endxy)+self.arrival_agl]
        if getattr(self,'synchronized_climb',False):
            d=self.direction;side=self.normal*(-80. if g==0 else 80.)
            from moving_merge import MERGE_ENTRY_DISTANCE
            entry_distance=MERGE_ENTRY_DISTANCE if getattr(self,'moving_merge',False) else 1200.
            entry=anchor-entry_distance*d+side+STAR
            before=anchor-600*d+side+STAR
            after=anchor-300*d+self.offset(g)+STAR
            level=150. if g==0 else 110.
            # Keep each incoming corridor on its own side. Exchange lateral
            # order at separated heights, then enter the original L slots.
            return [np.c_[entry,np.full(5,self.altitude)],np.c_[before,np.full(5,level)],np.c_[after,np.full(5,level)],end]
        startxy=endxy-1200*d
        return [np.c_[startxy,np.full(5,self.altitude)],end]

    def descent_merge(self,g,before):
        # Descending layers separate fixed IDs while the long column contracts.
        # No target is above its incoming altitude; the original L slots remain.
        path=self.descent_path(g);first=path[1].copy();second=path[2].copy()
        first[:,2]+=3.*np.arange(5)
        second[:,2]=(80. if g==0 else 60.)+3.*np.arange(5)
        return [('DESCENT_MERGE',first),('DESCEND',second),('DESCEND',path[-1])]

    def terminal_agl(self,progress):
        return float(self.arrival_agl+(self.end_agl-self.arrival_agl)*np.clip(progress,0,1))

    def group_safe(self,p,v,horizon=3.,limit=2000.,active_groups=None):
        p=np.asarray(p,float);v=np.asarray(v,float)
        if p.shape!=(10,3) or v.shape!=(10,3) or not np.isfinite(np.r_[p.ravel(),v.ravel()]).all():return False
        groups=tuple((0,1) if active_groups is None else active_groups)
        for t in (0,horizon):
            q=p+t*v
            for g in groups:
                members=q[5*g:5*g+5]
                if np.max(np.linalg.norm(members[:,None]-members[None,:],axis=2))>limit:return False
            if len(groups)==2 and np.linalg.norm(q[:5].mean(0)-q[5:].mean(0))>limit:return False
        return True

    def landed(self,states,ids=None):
        if len(states)!=10:return False
        ids=list(range(10) if ids is None else sorted(ids))
        slots=np.concatenate([self.landing_slots(g) for g in range(2)])
        for i in ids:
            s=states[i]
            if s.get('phase')!='LANDED' or not s.get('contact') or s.get('armed',True):return False
            p=np.asarray(s['position']);v=np.asarray(s['velocity'])
            if not np.isfinite(np.r_[p,v]).all() or np.linalg.norm(v)>=.15 or np.linalg.norm(p[:2]-slots[i,:2])>=.5:return False
        return True

    def sweep_clear(self,a,b,minimum_agl=1.5):
        a,b=np.asarray(a),np.asarray(b)
        samples=np.concatenate([np.linspace(x,y,max(2,int(np.ceil(np.linalg.norm(y-x)/.5))+1)) for x,y in zip(a,b)])
        h=self.ground(samples[:,:2])
        if not np.isfinite(samples).all() or not np.isfinite(h).all() or np.any(samples[:,2]>300) or np.any(samples[:,2]-h<minimum_agl-1e-6):return False
        for o in self.world['objects']:
            if 'width' not in o:continue
            theta=np.deg2rad(o.get('yaw',0));delta=samples[:,:2]-[o['x'],o['y']]
            x=delta[:,0]*np.cos(theta)+delta[:,1]*np.sin(theta)
            y=-delta[:,0]*np.sin(theta)+delta[:,1]*np.cos(theta)
            hit=(np.abs(x)<=o['width']/2+.5)&(np.abs(y)<=o['depth']/2+.5)&(samples[:,2]>=o.get('bottom_z',o['base_z'])-.5)&(samples[:,2]<=o['base_z']+o['height']+.5)
            if np.any(hit):return False
        return True

    def cloud(self,position):
        c=np.asarray(position)[:2];lo=np.floor((c-1000)/10).astype(int);hi=np.ceil((c+1000)/10).astype(int)
        x,y=np.meshgrid(np.arange(lo[0],hi[0]+1)*10,np.arange(lo[1],hi[1]+1)*10)
        xy=np.c_[x.ravel(),y.ravel()];xy=xy[np.linalg.norm(xy-c,axis=1)<=1000]
        z=self.ground(xy);return np.c_[xy[np.isfinite(z)],z[np.isfinite(z)]]
