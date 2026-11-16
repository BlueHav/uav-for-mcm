"""Bounded sensor-only rolling map. World and body are ROS FLU; depth is optical
(right, down, forward), in metres, NOT YOPO's normalized 20 m tensor.
Unknown is never implicitly free. Call update_pose before integrating sensors.
All public inputs/outputs are numpy arrays; no ROS dependency or world oracle.
"""
import time
import heapq
import ctypes
from collections import OrderedDict
import os
import numpy as np

try:
    _lib = ctypes.CDLL(os.path.join(os.path.dirname(__file__), 'ray_carve.so'))
    _carve = _lib.carve
    _carve.restype = None
    _carve.argtypes = [ctypes.c_void_p]*3+[ctypes.c_double]+[ctypes.c_void_p]*2+[ctypes.c_int]
except OSError:
    _carve = None


_carve_gpu = _carve_device = None
_cuda_carve_requested = os.environ.get('YOPO_CUDA_RAY_CARVE') == '1'
if _cuda_carve_requested or os.environ.get('YOPO_CUDA_DEPTH_ONLY')=='1':
    try:
        _gpu_lib = ctypes.CDLL(os.path.join(os.path.dirname(__file__), 'ray_carve_cuda.so'))
        _carve_gpu = _gpu_lib.carve_cuda
        _carve_gpu.restype = ctypes.c_int
        _carve_gpu.argtypes = [ctypes.c_void_p]*3+[ctypes.c_double]+[ctypes.c_void_p]*2+[ctypes.c_int]
        _carve_device = _gpu_lib.carve_cuda_device
        _carve_device.restype = ctypes.c_int
        _carve_device.argtypes = _carve_gpu.argtypes
    except OSError:
        pass


def route_corridor_clear(grid,a,b,radius=.35):
    """Physical sweep plus tracking margin beyond the departure segment."""
    a,b=np.asarray(a,float),np.asarray(b,float)
    if not grid.corridor_clear(a,b,radius=.35):return False
    if radius<=.35:return True
    if not grid.points_clear([b],radius=radius)[0]:return False
    points=np.linspace(a,b,max(2,int(np.ceil(np.linalg.norm(b-a)/.25))+1))
    buffered=points[np.linalg.norm(points-a,axis=1)>.75]
    return bool(np.all(grid.points_clear(buffered,radius=radius))) if len(buffered) else True


def visible_detour_waypoint(grid,position,path,lookahead=8,radius=.35):
    """A path index alone is not a safe shortcut around an obstacle corner."""
    position=np.asarray(position,float);path=np.asarray(path,float)
    nearest=int(np.argmin(np.linalg.norm(path-position,axis=1)))
    for point in path[nearest:min(len(path),nearest+lookahead+1)][::-1]:
        if np.linalg.norm(point-position)>.1 and route_corridor_clear(grid,position,point,radius=radius):return point.copy()
    return None


def protected_occupied_indices(state,occupied_stamps,stamp,ttl):
    # Only occupied cells can be protected. Avoid allocating/subtracting an
    # entire float64 volume on every camera frame when hits are sparse.
    occupied=np.flatnonzero(state.reshape(-1)==2)
    return occupied[stamp-occupied_stamps.reshape(-1)[occupied]<=ttl]


class RollingLocalMap:
    UNKNOWN, FREE, OCCUPIED = 0, 1, 2

    def __init__(self, size=(60., 60., 30.), resolution=.5, lead_seconds=1., max_lead=10., depth_stride=4, occupied_ttl=.75):
        self.resolution = float(resolution)
        self.shape = np.ceil(np.asarray(size) / resolution).astype(int)
        self.state = np.zeros(tuple(self.shape), dtype=np.uint8)
        # Attribution is used ONLY by optimization hints. Safety continues to
        # see every occupied endpoint, including sensed aircraft bodies.
        self.peer_only = np.zeros(tuple(self.shape),dtype=bool)
        self.last_occupied_stamp = np.full(tuple(self.shape),-np.inf,dtype=np.float64)
        self.occupied_ttl=float(occupied_ttl)
        self.protected_occ_restores=0
        self.cuda_carves=0
        self.cuda_carve_fallbacks=0
        self.near_depth_returns=OrderedDict()
        self.ground = np.full(tuple(self.shape[:2]), np.nan, dtype=np.float32)
        self.origin = None
        self.position = np.zeros(3)
        self.lead_seconds, self.max_lead = lead_seconds, min(float(max_lead), 10.)
        self.depth_stride = int(depth_stride)
        self.depth_stamp = self.ground_stamp = None
        self.integration_seconds = 0.
        self.dynamic_peers=np.empty((0,3));self.dynamic_clearance=2.2

    def update_pose(self, p, v=(0., 0., 0.)):
        self._detour_grid_cache=None
        p, v = np.asarray(p, float), np.asarray(v, float)
        lead = v * self.lead_seconds
        # Reserve quantization error before snapping the actual window center.
        # At .5 m, a 9.5 m requested lead plus sqrt(3)*.25 m rounding error
        # stays below 10 m in all directions, including negative velocities.
        reserve=max(.5,np.sqrt(3.)*self.resolution*.5)
        budget=max(0.,self.max_lead-reserve)
        lead *= min(1., budget / max(np.linalg.norm(lead), 1e-9))
        new = np.rint((p + lead) / self.resolution-self.shape*.5).astype(int)
        self.position = p.copy()
        if self.origin is None:
            self.origin = new
            return
        shift = new - self.origin
        if not np.any(shift):
            return
        # Copy only overlap. Exited observations are irretrievably discarded.
        old_state, old_ground, old_occ = self.state, self.ground, self.last_occupied_stamp
        old_peer=self.peer_only;self.peer_only=np.zeros_like(old_peer)
        self.state = np.zeros_like(old_state)
        self.ground = np.full_like(old_ground, np.nan)
        self.last_occupied_stamp=np.full_like(old_occ,-np.inf)
        if np.all(np.abs(shift) < self.shape):
            src = tuple(slice(max(int(s), 0), min(int(n+s), int(n))) for s, n in zip(shift, self.shape))
            dst = tuple(slice(max(int(-s), 0), min(int(n-s), int(n))) for s, n in zip(shift, self.shape))
            self.state[dst] = old_state[src]
            self.peer_only[dst]=old_peer[src]
            self.last_occupied_stamp[dst]=old_occ[src]
        if np.all(np.abs(shift[:2]) < self.shape[:2]):
            src = tuple(slice(max(int(s), 0), min(int(n+s), int(n))) for s, n in zip(shift[:2], self.shape[:2]))
            dst = tuple(slice(max(int(-s), 0), min(int(n-s), int(n))) for s, n in zip(shift[:2], self.shape[:2]))
            self.ground[dst] = old_ground[src]
        self.origin = new

    def _indices(self, points):
        idx = np.floor(np.asarray(points) / self.resolution).astype(int) - self.origin
        return idx, np.all((idx >= 0) & (idx < self.shape), axis=-1)

    def integrate_depth(self, depth_m, position, rotation_body_to_world, fx=240., fy=240., cx=239.5, cy=134.5, stamp=None, dense_center=False, focus_directions=None, peer_mask=None, cuda_peers=(), camera=None):
        self._detour_grid_cache=None
        start = time.perf_counter()
        if self.origin is None:
            self.update_pose(position)
        on_device=hasattr(depth_m,"cuda_pixels")
        d=depth_m if on_device else np.asarray(depth_m)
        if peer_mask is not None and (not isinstance(peer_mask,np.ndarray) or peer_mask.shape!=d.shape or peer_mask.dtype!=np.bool_):raise ValueError('invalid peer endpoint mask')
        yy, xx = np.mgrid[0:d.shape[0]:self.depth_stride, 0:d.shape[1]:self.depth_stride]
        if dense_center:
            # Add real wide-camera rays along the braking corridor; never
            # replace unobserved gaps with assumed free voxels.
            if focus_directions is None:
                gy,gx=np.mgrid[max(0,int(cy)-80):min(d.shape[0],int(cy)+81):2,max(0,int(cx)-80):min(d.shape[1],int(cx)+81):2]
            else:
                body=np.asarray(focus_directions,float).reshape(-1,3)@np.asarray(rotation_body_to_world,float)
                body=body[np.isfinite(body).all(1)&(body[:,0]>1.5)]
                px=np.r_[cx,cx-body[:,1]/body[:,0]*fx];py=np.r_[cy,cy-body[:,2]/body[:,0]*fy]
                # Dense actual pixels around both measured and requested stop
                # directions, rather than a large sparsely sampled square.
                x0=max(0,int(np.floor(px.min()))-20);x1=min(d.shape[1],int(np.ceil(px.max()))+21)
                y0=max(0,int(np.floor(py.min()))-20);y1=min(d.shape[0],int(np.ceil(py.max()))+21)
                gy,gx=np.mgrid[y0:y1,x0:x1]
            yy=np.r_[yy.ravel(),gy.ravel()];xx=np.r_[xx.ravel(),gx.ravel()]
        pos=np.asarray(position,float)
        if on_device:
            if _carve_device is None:raise RuntimeError("CUDA depth requires the device ray-carve library")
            rays,endpoints,peer_hit=d.rays(yy,xx,pos,rotation_body_to_world,fx,fy,cx,cy,cuda_peers)
        else:
            z=d[yy,xx].reshape(-1)
            valid=np.isfinite(z)&(z>.05)
            z,xx,yy=z[valid],xx.reshape(-1)[valid],yy.reshape(-1)[valid]
            peer_hit=np.zeros(len(z),bool) if peer_mask is None else peer_mask[yy,xx]
            # Optical z-forward/x-right/y-down -> FLU x-forward/y-left/z-up.
            body=np.column_stack((z,-(xx-cx)*z/fx,-(yy-cy)*z/fy))
            rays=body@np.asarray(rotation_body_to_world,float).T
            endpoints=pos+rays
        sensor_stamp=time.time() if stamp is None else float(stamp)
        effective_stamp=max(sensor_stamp,self.depth_stamp if self.depth_stamp is not None else sensor_stamp)
        # Different cameras may graze different parts of the same coarse voxel.
        # A free ray cannot erase a recently observed solid endpoint. Old hits
        # clear only when actually crossed by a subsequent free observation.
        protected=protected_occupied_indices(self.state,self.last_occupied_stamp,effective_stamp,self.occupied_ttl)
        # Half-cell sampling avoids holes; bounded vector chunks avoid large arrays.
        if on_device:
            shape=np.ascontiguousarray(self.shape,dtype=np.int64)
            origin=np.ascontiguousarray(self.origin,dtype=np.int64)
            args=(self.state.ctypes.data,shape.ctypes.data,origin.ctypes.data,
                  self.resolution,pos.ctypes.data,rays.data_ptr(),len(rays))
            if _carve_device(*args)!=0:raise RuntimeError("CUDA device ray carve failed")
            self.cuda_carves+=1
            carved=True
        elif _carve is not None or _carve_gpu is not None:
            rays = np.ascontiguousarray(rays,dtype=np.float64)
            pos = np.ascontiguousarray(pos,dtype=np.float64)
            shape = np.ascontiguousarray(self.shape,dtype=np.int64)
            origin = np.ascontiguousarray(self.origin,dtype=np.int64)
            args=(self.state.ctypes.data,shape.ctypes.data,origin.ctypes.data,
                  self.resolution,pos.ctypes.data,rays.ctypes.data,len(rays))
            carved=_carve_gpu is not None and _carve_gpu(*args)==0
            if _cuda_carve_requested:
                self.cuda_carves+=int(carved)
                self.cuda_carve_fallbacks+=int(not carved)
            if not carved and _carve is not None:
                _carve(*args)
                carved=True
        else:
            carved=False
        if not carved:
            length=np.linalg.norm(rays,axis=1)
            unit=rays/np.maximum(length[:,None],1e-9)
            max_dist=float(np.linalg.norm(self.shape*self.resolution))
            for distance in np.arange(0., min(float(length.max(initial=0)), max_dist), self.resolution*.5):
                live = length > distance + self.resolution*.75
                pts = pos + unit[live]*distance
                idx, inside = self._indices(pts)
                ii = idx[inside]
                self.state[tuple(ii.T)] = self.FREE
        erased=np.count_nonzero(self.state.reshape(-1)[protected]!=self.OCCUPIED)
        self.protected_occ_restores+=int(erased)
        self.state.reshape(-1)[protected]=self.OCCUPIED
        terrain = self.ground_at_many(endpoints[:, :2])
        terrain_hit = np.isfinite(terrain) & (np.abs(endpoints[:,2]-np.nan_to_num(terrain)) <= .08)
        idx, inside = self._indices(endpoints)
        inside &= ~terrain_hit
        ii = idx[inside]
        if on_device:
            near=inside & (np.linalg.norm(endpoints-pos,axis=1)<1.5)
            for index in np.flatnonzero(near):
                cell=tuple((idx[index]+self.origin).tolist())
                forward,left,up=(endpoints[index]-pos)@rotation_body_to_world
                pixel=[int(round(cx-fx*left/forward)),int(round(cy-fy*up/forward))]
                self.near_depth_returns[cell]=dict(camera=camera,stamp=sensor_stamp,
                    origin=pos.tolist(),endpoint=endpoints[index].tolist(),peer=bool(peer_hit[index]),
                    pixel=pixel,depth=float(forward))
                self.near_depth_returns.move_to_end(cell)
            while len(self.near_depth_returns)>128:self.near_depth_returns.popitem(last=False)
        # A peer observation cannot erase a static surface in the same voxel.
        # Mixed pixels and unmarked legacy input always remain static hints.
        new=ii[self.state[tuple(ii.T)]!=self.OCCUPIED]
        self.peer_only[tuple(new.T)]=True
        static=idx[inside & ~peer_hit]
        self.peer_only[tuple(static.T)]=False
        self.state[tuple(ii.T)] = self.OCCUPIED
        self.last_occupied_stamp[tuple(ii.T)] = np.maximum(self.last_occupied_stamp[tuple(ii.T)],sensor_stamp)
        self.depth_stamp = effective_stamp
        self.integration_seconds = time.perf_counter()-start

    def known_body_volume(self, p, half=(.25,.25,.09)):
        """Current physically occupied body cells contain no external obstacle.
        This seeds only cells touched by the actual body box, never future air.
        Terrain exclusion remains an independent metric-height test.
        """
        self._detour_grid_cache=None
        p,half=np.asarray(p,float),np.asarray(half,float)
        if self.origin is None:self.update_pose(p)
        lo=np.floor((p-half)/self.resolution).astype(int)-self.origin
        hi=np.floor((p+half)/self.resolution).astype(int)-self.origin
        lo=np.maximum(lo,0);hi=np.minimum(hi,self.shape-1)
        if np.all(hi>=lo):
            cells=tuple(slice(int(a),int(b)+1) for a,b in zip(lo,hi))
            self.state[cells]=self.FREE;self.last_occupied_stamp[cells]=-np.inf

    def integrate_ground(self, pointsXYZ, stamp=None):
        self._detour_grid_cache=None
        points = np.asarray(pointsXYZ, float).reshape(-1, 3)
        if self.origin is None:
            self.update_pose(self.position)
        ij = np.rint(points[:, :2]/self.resolution).astype(int)-self.origin[:2]
        valid = np.all(np.isfinite(points), axis=1) & np.all((ij >= 0) & (ij < self.shape[:2]), axis=1)
        self.ground[tuple(ij[valid].T)] = points[valid, 2]
        self.ground_stamp = time.time() if stamp is None else float(stamp)

    def ground_at(self, x, y):
        if self.origin is None or not np.isfinite(x+y):
            return float('nan')
        q = np.array([x, y])/self.resolution-self.origin[:2]
        ij = np.floor(q).astype(int)
        if np.any(ij < 0) or np.any(ij+1 >= self.shape[:2]):
            return float('nan')
        values = self.ground[ij[0]:ij[0]+2, ij[1]:ij[1]+2]
        weights = np.outer([1-(q[0]-ij[0]), q[0]-ij[0]], [1-(q[1]-ij[1]), q[1]-ij[1]])
        needed = weights > 1e-9
        if not np.all(np.isfinite(values[needed])):
            return float('nan')
        return float(np.sum(values[needed]*weights[needed]))

    def ground_at_many(self, xy):
        xy = np.asarray(xy,float).reshape(-1,2)
        if self.origin is None:
            return np.full(len(xy),np.nan)
        finite = np.all(np.isfinite(xy),axis=1)
        q = np.where(np.isfinite(xy),xy,0)/self.resolution-self.origin[:2]
        ij = np.floor(q).astype(int)
        valid = finite & np.all((ij>=0)&(ij+1<self.shape[:2]),axis=1)
        out = np.full(len(xy),np.nan)
        ids = np.flatnonzero(valid); cell=ij[valid]; frac=q[valid]-cell
        total=np.zeros(len(ids)); known=np.ones(len(ids),bool)
        for a,b in ((0,0),(0,1),(1,0),(1,1)):
            w=(frac[:,0] if a else 1-frac[:,0])*(frac[:,1] if b else 1-frac[:,1])
            z=self.ground[cell[:,0]+a,cell[:,1]+b]
            known &= np.isfinite(z)|(w<1e-9)
            total+=np.where(np.isfinite(z),z,0)*w
        out[ids[known]]=total[known]
        return out

    def points_clear(self, points, radius=.35, unknownunsafe=True,check_peers=True):
        points = np.asarray(points, float).reshape(-1, 3)
        if self.origin is None:
            return np.zeros(len(points),bool)
        result = np.all(np.isfinite(points), axis=1)
        steps = int(np.ceil(radius/self.resolution))
        # Enumerate nearby voxels, then test their boxes against the actual
        # sphere. A fixed 3x3x3 cube wrongly blocks corners outside the radius.
        offsets = np.array([(a,b,c) for a in range(-steps,steps+1) for b in range(-steps,steps+1) for c in range(-steps,steps+1)])
        sample = (points[:,None,:] + offsets[None,:,:]*self.resolution).reshape(-1,3)
        idx, inside = self._indices(np.where(np.isfinite(sample),sample,0))
        query=np.repeat(points,len(offsets),axis=0)
        lower=(idx+self.origin)*self.resolution
        distance=np.maximum(np.maximum(lower-query,query-(lower+self.resolution)),0.)
        intersects=np.sum(distance*distance,axis=1)<=radius*radius+1e-12
        states = np.zeros(len(sample),dtype=np.uint8)
        states[inside] = self.state[tuple(idx[inside].T)]
        clear = inside & ((states == self.FREE) if unknownunsafe else (states != self.OCCUPIED))
        terrain = self.ground_at_many(sample[:,:2])
        below = np.zeros(len(sample),bool)
        known = np.isfinite(terrain)&np.isfinite(sample[:,2])
        below[known] = sample[known,2] <= terrain[known]
        clear &= ~below
        clear |= ~intersects
        result &= np.all(clear.reshape(len(points),len(offsets)),axis=1)
        peers=getattr(self,'dynamic_peers',np.empty((0,3))) if check_peers else []
        for peer in peers:
            # A member already inside the extra planning buffer may escape
            # outward, but must never move closer or cross the 2 m body limit.
            clearance=max(2.,min(self.dynamic_clearance,float(np.linalg.norm(self.position-peer))))
            result &= np.linalg.norm(points-peer,axis=1)>=clearance-1e-8
        return result

    def _margin_blockers(self,p,radius=.35):
        steps=int(np.ceil(radius/self.resolution))
        offsets=np.array([(x,y,z) for x in range(-steps,steps+1) for y in range(-steps,steps+1) for z in range(-steps,steps+1)])
        sample=p+offsets*self.resolution;idx,inside=self._indices(sample)
        lower=(idx+self.origin)*self.resolution
        distance=np.maximum(np.maximum(lower-p,p-lower-self.resolution),0.)
        d2=np.sum(distance*distance,axis=1);intersects=d2<=radius*radius+1e-12
        states=np.zeros(len(idx),np.uint8);states[inside]=self.state[tuple(idx[inside].T)]
        ground=self.ground_at_many(sample[:,:2])
        terrain=bool(np.any(intersects&np.isfinite(ground)&(sample[:,2]<=ground)))
        return {tuple(cell):float(d) for cell,d in zip(idx[intersects&(states!=self.FREE)],d2[intersects&(states!=self.FREE)])},terrain

    def escape_points_clear(self,points,start):
        """Only retreat from an existing margin overlap, never erase occupancy.

        Unity's collision sweep has radius 0.30 m. Every such body sphere must
        remain in measured free air, and distance to each initially blocking
        voxel must increase monotonically. New 0.35 m margin overlaps fail.
        """
        points=np.asarray(points,float).reshape(-1,3);start=np.asarray(start,float)
        result=np.zeros(len(points),bool)
        if self.origin is None or not self.points_clear([start],radius=.30)[0]:return result
        initial,terrain=self._margin_blockers(start)
        if terrain:return result
        if not initial:return self.points_clear(points,radius=.35)
        result=self.points_clear(points,radius=.30)
        result&=np.linalg.norm(points-start,axis=1)<=1.5
        cells=np.array(list(initial));lower=(cells+self.origin)*self.resolution
        last=np.array(list(initial.values()))
        for i,p in enumerate(points):
            blockers,terrain=self._margin_blockers(p)
            distance=np.maximum(np.maximum(lower-p,p-lower-self.resolution),0.)
            d2=np.sum(distance*distance,axis=1)
            result[i]&=not terrain and set(blockers).issubset(initial) and bool(np.all(d2>=last-1e-9))
            last=d2
        return result

    def buffer_escape_target(self,start,goal,agl=2.):
        start,goal=np.asarray(start,float),np.asarray(goal,float)
        if not self.points_clear([start],radius=.30)[0]:return None
        choices=[]
        for angle in np.arange(16)*np.pi/8:
            xy=start[:2]+.75*np.array([np.cos(angle),np.sin(angle)])
            q=np.r_[xy,self.ground_at(*xy)+agl]
            if not self.points_clear([q],radius=.5)[0]:continue
            path=np.linspace(start,q,21)
            if np.all(self.escape_points_clear(path,start)):
                choices.append((float(np.linalg.norm(q[:2]-goal[:2])),q))
        return min(choices,key=lambda item:item[0])[1] if choices else None

    def corridor_clear(self, a, b, radius=.35, unknownunsafe=True):
        a, b = np.asarray(a,float), np.asarray(b,float)
        points = np.linspace(a,b,max(2,int(np.ceil(np.linalg.norm(b-a)/(self.resolution*.5)))+1))
        return bool(np.all(self.points_clear(points,radius,unknownunsafe)))

    def reject_reason(self, points, radius=.35):
        return None if np.all(self.points_clear(points, radius)) else 'occupied_unknown_or_outside_local_window'

    def local_detour(self, start, goal, agl=2., radius=.35, max_expansions=3000, search_budget=None):
        """Bounded 2D A* at terrain+AGL. Returns world points or None.
        Missing terrain/unknown air is blocked; never extrapolates terrain.
        """
        started=time.perf_counter()
        start,goal=np.asarray(start,float),np.asarray(goal,float)
        if self.origin is None:return None
        a=tuple(np.floor(start[:2]/self.resolution).astype(int)-self.origin[:2])
        b=tuple(np.floor(goal[:2]/self.resolution).astype(int)-self.origin[:2])
        nx,ny=map(int,self.shape[:2])
        if not (0<=a[0]<nx and 0<=a[1]<ny and 0<=b[0]<nx and 0<=b[1]<ny):return None
        # Candidate endpoints in the same planning cycle share an unchanged
        # sensor grid. Every pose/sensor integration invalidates this cache.
        key=(float(agl),float(radius),self.dynamic_peers.tobytes(),float(self.dynamic_clearance))
        cached=getattr(self,'_detour_grid_cache',None)
        if cached is not None and cached[0]==key:points,allowed,terrain_z=cached[1:]
        else:
            ii,jj=np.meshgrid(np.arange(nx),np.arange(ny),indexing='ij')
            xy=(np.c_[ii.ravel(),jj.ravel()]+self.origin[:2]+.5)*self.resolution
            z=self.ground_at_many(xy)+agl;points=np.c_[xy,z]
            allowed=np.zeros(len(points),bool)
            # Unknown ground is already blocked. Avoid expanding 27 voxel
            # queries for each of those unusable cells on every map refresh.
            known=np.flatnonzero(np.isfinite(z))
            for offset in range(0,len(known),2048):
                ids=known[offset:offset+2048]
                allowed[ids]=self.points_clear(points[ids],radius)
            allowed=allowed.reshape(nx,ny);terrain_z=z.reshape(nx,ny)
            self._detour_grid_cache=(key,points,allowed,terrain_z)
        self.last_detour_stats=dict(preprocess_seconds=time.perf_counter()-started,allowed_cells=int(np.sum(allowed)),expansions=0)
        def connect_endpoint(position,cell,departure=False):
            choices=[]
            for dx in (-1,0,1):
                for dy in (-1,0,1):
                    candidate=(cell[0]+dx,cell[1]+dy)
                    if not (0<=candidate[0]<nx and 0<=candidate[1]<ny) or not allowed[candidate]:continue
                    q=points[candidate[0]*ny+candidate[1]]
                    choices.append((float(np.linalg.norm(q-position)),candidate,q))
            for _,candidate,q in sorted(choices,key=lambda x:x[0]):
                if (route_corridor_clear(self,position,q,radius) if departure else self.corridor_clear(position,q,radius)):return candidate
            return None
        a,b=connect_endpoint(start,a,True),connect_endpoint(goal,b)
        if a is None or b is None:return None
        queue=[(np.linalg.norm(np.asarray(a)-b),0.,a)];costs={a:0.};parent={};closed=set()
        deadline=float('inf') if search_budget is None else time.perf_counter()+search_budget
        while queue and len(closed)<max_expansions:
            if len(closed)%32==0 and time.perf_counter()>=deadline:break
            _,cost,cell=heapq.heappop(queue)
            if cell in closed:continue
            closed.add(cell)
            if cell==b:
                cells=[cell]
                while cell in parent:cell=parent[cell];cells.append(cell)
                cells.reverse();path=np.array([points[c[0]*ny+c[1]] for c in cells])
                self.last_detour_stats.update(expansions=len(closed),seconds=time.perf_counter()-started)
                # Connect actual start and goal only after measured swept checks.
                if not route_corridor_clear(self,start,path[0],radius) or not self.corridor_clear(path[-1],goal,radius):return None
                return np.vstack([start,path,goal])
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                nxt=(cell[0]+dx,cell[1]+dy)
                if not (0<=nxt[0]<nx and 0<=nxt[1]<ny) or not allowed[nxt] or nxt in closed:continue
                # No diagonal cuts across occupied or unknown orthogonal cells.
                if dx and dy and (not allowed[cell[0]+dx,cell[1]] or not allowed[cell[0],cell[1]+dy]):continue
                if abs(terrain_z[nxt]-terrain_z[cell])>self.resolution:continue
                nextcost=cost+np.hypot(dx,dy)
                if nextcost>=costs.get(nxt,float('inf')):continue
                costs[nxt]=nextcost;parent[nxt]=cell
                heapq.heappush(queue,(nextcost+np.linalg.norm(np.asarray(nxt)-b),nextcost,nxt))
        self.last_detour_stats.update(expansions=len(closed),seconds=time.perf_counter()-started)
        return None
    def stats(self):
        return dict(bytes=self.state.nbytes+self.ground.nbytes+self.last_occupied_stamp.nbytes, shape=self.shape.tolist(), origin=None if self.origin is None else (self.origin*self.resolution).tolist(), free=int(np.count_nonzero(self.state==1)), occupied=int(np.count_nonzero(self.state==2)), unknown=int(np.count_nonzero(self.state==0)), depth_stamp=self.depth_stamp, ground_stamp=self.ground_stamp, integration_seconds=self.integration_seconds,occupied_ttl=self.occupied_ttl,protected_occ_restores=self.protected_occ_restores,cuda_carves=self.cuda_carves,cuda_carve_fallbacks=self.cuda_carve_fallbacks,detour=getattr(self,'last_detour_stats',None))
