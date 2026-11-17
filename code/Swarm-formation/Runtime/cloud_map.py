"""Terrain-only airspace, explicitly authorized for the high-altitude stage.

Finite samples define a height field. Missing ground is unknown, not free air.
This backend claims no building/tree detection and never integrates images.
"""
import copy
import numpy as np
from local_map import RollingLocalMap

def complete_ground_origin(points,resolution):
    """Validate the full unordered 121-square lattice in linear time."""
    if len(points)!=121*121:return None
    xy=points[:,:2]/resolution
    if not np.isfinite(xy).all() or np.any(np.abs(xy)>=2.**62):return None
    indices=np.rint(xy).astype(np.int64);lo=indices.min(0);hi=indices.max(0)
    if not np.allclose(xy,indices,rtol=0.,atol=1e-4) or not np.array_equal(hi-lo,[120,120]):return None
    # Bounds above make every flat index lie in [0,14640]. One occurrence
    # in every bin proves both uniqueness and complete rectangular coverage.
    relative=indices-lo
    flat=relative[:,0]*121+relative[:,1]
    if not np.all(np.bincount(flat,minlength=121*121)==1):return None
    return lo

class CloudTerrainMap(RollingLocalMap):
    def integrate_depth(self,*args,**kwargs):
        raise RuntimeError('Image input forbidden in terrain-cloud mode')
    def integrate_ground(self,points,stamp=None):
        self.ground.fill(np.nan)
        super().integrate_ground(points,stamp)
        heights=np.full(self.ground.shape,np.nan)
        corners=np.stack([self.ground[:-1,:-1],self.ground[1:,:-1],self.ground[:-1,1:],self.ground[1:,1:]])
        covered=np.isfinite(corners).all(0)
        heights[:-1,:-1]=np.where(covered,np.max(corners,axis=0),np.nan)
        bottom=(self.origin[2]+np.arange(self.shape[2]))*self.resolution
        # Write the uint8 voxel buffer directly. Nested where with Python int
        # branches allocates two full int64 volumes on every sensor update.
        np.greater(bottom[None,None,:],heights[:,:,None],out=self.state)
        np.subtract(np.uint8(self.OCCUPIED),self.state,out=self.state)
        self.state[~np.isfinite(heights)]=self.UNKNOWN
    def known_body_volume(self,p):
        # Body presence alone cannot fill missing terrain coverage.
        return None
    def points_clear(self,points,radius=.35,unknownunsafe=True,check_peers=True):
        p=np.asarray(points,float).reshape(-1,3)
        result=np.isfinite(p).all(1)&(p[:,2]+radius<=300.)
        if self.origin is None:return np.zeros(len(p),bool)
        # Include every vertex of every bilinear cell touched by the footprint.
        # Sampling only center/edges can miss a grid peak between those samples.
        safe=np.where(np.isfinite(p),p,0.)
        low=np.floor((safe[:,:2]-radius)/self.resolution).astype(int)-self.origin[:2]
        high=np.floor((safe[:,:2]+radius)/self.resolution).astype(int)-self.origin[:2]+1
        width=int(np.ceil(2*radius/self.resolution))+2
        offsets=np.array([(x,y) for x in range(width) for y in range(width)])
        idx=low[:,None,:]+offsets
        needed=(idx<=high[:,None,:]).all(2)
        inside=((idx>=0)&(idx<self.shape[:2])).all(2)
        heights=np.full(needed.shape,np.nan)
        cells=idx[inside];heights[inside]=self.ground[cells[:,0],cells[:,1]]
        known=~needed|(inside&np.isfinite(heights))
        upper=np.max(np.where(needed,np.nan_to_num(heights,nan=np.inf),-np.inf),axis=1)
        result &= known.all(1)&(safe[:,2]-radius>upper)
        for peer in self.dynamic_peers if check_peers else []:
            clearance=max(2.,min(self.dynamic_clearance,float(np.linalg.norm(self.position-peer))))
            result &= np.linalg.norm(p-peer,axis=1)>=clearance-1e-8
        return result
    def stats(self):
        value=super().stats();value.update(input_kind='terrain_cloud',buildings_observed=False)
        return value

class LayeredCloudMap(CloudTerrainMap):
    """Actual fine ground plus a separate conservative long-stop envelope."""
    def __init__(self):
        # Unity supplies 121 vertices spanning 60 m. Keep both boundary rows.
        super().__init__(size=(60.5,60.5,60.))
        from cloud_config import CONTEXT_SPAN,CONTEXT_RESOLUTION
        self.context=CloudTerrainMap(size=(CONTEXT_SPAN,CONTEXT_SPAN,CONTEXT_RESOLUTION),resolution=CONTEXT_RESOLUTION,lead_seconds=0.,max_lead=0.)
        self.context_hash=None
    def update_pose(self,p,v=(0.,0.,0.)):
        p=np.asarray(p,float)
        anchor=p.copy()
        if self.origin is not None:anchor[:2]=(self.origin[:2]+self.shape[:2]*.5)*self.resolution
        super().update_pose(anchor,np.zeros(3));self.position=p.copy();self.context.update_pose(p,v)
    def integrate_ground(self,points,stamp=None):
        points=np.asarray(points,float).reshape(-1,3)
        lo=complete_ground_origin(points,self.resolution)
        if lo is None:
            # A truncated edge cannot shrink the fine domain and expose coarse
            # fallback. Invalidate the observation and let readiness hold flight.
            self.ground.fill(np.nan);self.state.fill(self.UNKNOWN);self.ground_stamp=None;return
        if self.origin is None:super().update_pose(self.position)
        self.origin[:2]=lo
        super().integrate_ground(points,stamp)
    def integrate_context(self,points,stamp):self.context.integrate_ground(points,stamp)
    def planning_grid(self):
        self.context.dynamic_peers=self.dynamic_peers.copy();self.context.dynamic_clearance=self.dynamic_clearance
        return self.context
    def certification_snapshot(self):
        frozen=copy.copy(self);frozen.context=copy.copy(self.planning_grid())
        for target,source in ((frozen,self),(frozen.context,self.context)):
            for key in ('ground','origin','position','dynamic_peers'):
                value=getattr(source,key);setattr(target,key,None if value is None else value.copy())
        return frozen
    def points_clear(self,points,radius=.35,unknownunsafe=True,check_peers=True):
        p=np.asarray(points,float).reshape(-1,3)
        if self.origin is None:return np.zeros(len(p),bool)
        safe=np.where(np.isfinite(p),p,0.)
        low=np.floor((safe[:,:2]-radius)/self.resolution).astype(int)-self.origin[:2]
        high=np.floor((safe[:,:2]+radius)/self.resolution).astype(int)-self.origin[:2]+1
        near=((low>=0)&(high<self.shape[:2])).all(1)
        result=np.zeros(len(p),bool)
        if near.any():result[near]=super().points_clear(p[near],radius,unknownunsafe,check_peers)
        if (~near).any():result[~near]=self.planning_grid().points_clear(p[~near],radius,unknownunsafe,check_peers)
        # A straddling footprint must obey every fine vertex it touches.
        # Coarse coverage cannot erase an unknown or peak on the inside edge.
        width=int(np.ceil(2*radius/self.resolution))+2
        offsets=np.array([(x,y) for x in range(width) for y in range(width)])
        idx=low[:,None,:]+offsets
        overlap=(idx<=high[:,None,:]).all(2)&((idx>=0)&(idx<self.shape[:2])).all(2)
        heights=np.zeros(overlap.shape);cells=idx[overlap]
        heights[overlap]=self.ground[cells[:,0],cells[:,1]]
        result&=(~overlap|(np.isfinite(heights)&(safe[:,2,None]-radius>np.nan_to_num(heights,nan=np.inf)))).all(1)
        return result
    def stats(self):
        value=super().stats();value.update(context_stamp=self.context.ground_stamp,context_shape=self.context.shape.tolist(),
                context_hash=self.context_hash,context_height_kind='conservative_terrain_upper_bound',context_bytes=self.context.state.nbytes+self.context.ground.nbytes)
        return value
