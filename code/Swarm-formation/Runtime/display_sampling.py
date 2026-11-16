"""Variable-density display points. This module supplies no planner observations."""
import numpy as np
import hashlib


def verify_display_cache(manifest, paths):
    for key in ('world', 'metadata', 'source', 'cache'):
        digest = hashlib.sha256(paths[key].read_bytes()).hexdigest()
        if manifest.get(key+'_sha256') != digest:
            raise ValueError('Display cache source changed: '+key+'; regenerate the DEM cache')


def display_grid(center, focus, radius=10000., near_radius=1000., near_step=10., far_step=100.):
    center = np.asarray(center, float)
    focus = np.asarray(focus, float).reshape(-1, 2)
    if center.shape != (2,) or not np.isfinite(center).all() or not len(focus) or not np.isfinite(focus).all():
        raise ValueError('Finite display center and focus points required')
    if not np.isfinite([radius, near_radius, near_step, far_step]).all() or not 0 < near_step < far_step or not 0 < near_radius <= radius:
        raise ValueError('Invalid display extent or spacing')
    if (2*radius/far_step+3)**2 + len(focus)*(2*near_radius/near_step+3)**2 > 1000000:
        raise ValueError('Display sampling capacity exceeded')

    def disk(c, r, step):
        lo = np.floor((c-r)/step).astype(int)
        hi = np.ceil((c+r)/step).astype(int)
        x, y = np.meshgrid(np.arange(lo[0], hi[0]+1)*step, np.arange(lo[1], hi[1]+1)*step)
        xy = np.c_[x.ravel(), y.ravel()]
        return xy[np.linalg.norm(xy-c, axis=1) <= r+1e-8]

    near = np.unique(np.concatenate([disk(c, near_radius, near_step) for c in focus]), axis=0)
    near = near[np.linalg.norm(near-center, axis=1) <= radius+1e-8]
    far = disk(center, radius, far_step)
    keep = np.ones(len(far), bool)
    for c in focus:
        keep &= np.linalg.norm(far-c, axis=1) > near_radius+1e-8
    far = far[keep]
    return np.concatenate([near, far]), np.r_[np.ones(len(near), bool), np.zeros(len(far), bool)]


def world_height(world, xy):
    xy = np.asarray(xy, float).reshape(-1, 2)
    b, t = world['bounds'], world['terrain']
    n, m = t['width'], t['height']
    valid = np.isfinite(xy).all(1) & (xy[:, 0] >= b['x_min']) & (xy[:, 0] <= b['x_max']) & (xy[:, 1] >= b['y_min']) & (xy[:, 1] <= b['y_max'])
    out = np.full(len(xy), np.nan)
    uv = (xy[valid]-[b['x_min'], b['y_max']])/[(b['x_max']-b['x_min'])/(n-1), -(b['y_max']-b['y_min'])/(m-1)]
    cell = np.clip(np.floor(uv).astype(int), [0, 0], [n-2, m-2])
    u, v = (uv-cell).T
    c, r = cell.T
    h = np.asarray(t['heights']).reshape(m, n)
    a, z, top, e = h[r, c], h[r, c+1], h[r+1, c], h[r+1, c+1]
    out[valid] = np.where(u+v <= 1, a+(z-a)*u+(top-a)*v, e+(top-e)*(1-u)+(z-e)*(1-v))
    return out


def cached_dem_height(cache, xy):
    xy = np.asarray(xy, float).reshape(-1, 2)
    h = cache['heights']; x0, y0, step = float(cache['x0']), float(cache['y0']), float(cache['step'])
    uv = (xy-[x0, y0])/step
    valid = np.isfinite(uv).all(1) & (uv[:, 0] >= 0) & (uv[:, 1] >= 0) & (uv[:, 0] <= h.shape[1]-1) & (uv[:, 1] <= h.shape[0]-1)
    out = np.full(len(xy), np.nan)
    cell = np.clip(np.floor(uv[valid]).astype(int), [0, 0], [h.shape[1]-2, h.shape[0]-2])
    u, v = (uv[valid]-cell).T; c, r = cell.T
    out[valid] = h[r,c]*(1-u)*(1-v)+h[r,c+1]*u*(1-v)+h[r+1,c]*(1-u)*v+h[r+1,c+1]*u*v
    return out
