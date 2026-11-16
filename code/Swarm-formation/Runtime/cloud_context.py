"""Conservative bounded terrain envelope from the authoritative fused terrain."""
import numpy as np
from cloud_config import CONTEXT_SPAN,CONTEXT_RESOLUTION
from display_sampling import world_height

def gradient_bound(world):
    t=world['terrain'];b=world['bounds'];h=np.asarray(t['heights'],float).reshape(t['height'],t['width'])
    dx=(b['x_max']-b['x_min'])/(t['width']-1);dy=(b['y_max']-b['y_min'])/(t['height']-1)
    a,z,top,e=h[:-1,:-1],h[:-1,1:],h[1:,:-1],h[1:,1:]
    slopes=np.r_[np.hypot((z-a)/dx,(top-a)/dy).ravel(),np.hypot((e-top)/dx,(e-z)/dy).ravel()]
    if not np.isfinite(slopes).all():raise ValueError('Unknown terrain cannot supply a global slope certificate')
    return float(slopes.max())

def context_cloud(world,position,slope,span=CONTEXT_SPAN,resolution=CONTEXT_RESOLUTION):
    center=np.round(np.asarray(position,float)[:2]/resolution)*resolution
    axis=np.arange(-span/2,span/2+resolution*.5,resolution)
    x,y=np.meshgrid(center[0]+axis,center[1]+axis,indexing='ij');xy=np.c_[x.ravel(),y.ravel()]
    if np.max(np.linalg.norm(xy-center,axis=1))>1000:raise ValueError('Cloud context exceeds 1km authorization')
    heights=world_height(world,xy)
    # Every vertex certifies all adjacent 2m cells, including interior source
    # triangle edges/peaks. This envelope is NOT an actual ground-height input.
    heights+=slope*np.sqrt(2)*resolution
    b=world['bounds'];inside=(xy[:,0]-resolution>=b['x_min'])&(xy[:,0]+resolution<=b['x_max'])&(xy[:,1]-resolution>=b['y_min'])&(xy[:,1]+resolution<=b['y_max'])
    heights[~inside]=np.nan
    return np.c_[xy,heights].astype(np.float32)
