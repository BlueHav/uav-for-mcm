"""Large-terrain direction hints from an existing ground cloud cropped to 1 km.

This terrain context supplies direction only. Local cloud trajectory, stopping
and peer checks certify executable motion; YOPO belongs to the later phase.
"""
import heapq
import math
import numpy as np


def terrain_context(cloud, position, radius=1000.):
    cloud = np.asarray(cloud, float).reshape(-1, 3)
    position = np.asarray(position, float)
    if position.shape != (3,) or not np.isfinite(position).all() or not np.isfinite(radius) or not 0 < radius <= 1000:
        raise ValueError('Finite position and radius in (0,1000] required')
    valid = np.isfinite(cloud).all(1)
    return cloud[valid & (np.linalg.norm(cloud[:, :2]-position[:2], axis=1) <= radius+1e-8)]


def grid_line_clear(a,b,blocked):
    """Exact grid traversal, including both cells at a diagonal corner."""
    q=np.asarray(a,float)+.5;d=np.asarray(b,float)-a
    cell=np.floor(q).astype(int);end=np.floor(np.asarray(b)+.5).astype(int);step=np.sign(d).astype(int)
    delta=np.full(2,np.inf);moving=np.abs(d)>1e-12;delta[moving]=1/np.abs(d[moving])
    crossing=np.full(2,np.inf)
    for k in range(2):
        if moving[k]:crossing[k]=((cell[k]+(1 if step[k]>0 else 0))-q[k])/d[k]
    def clear(c):return 0<=c[0]<blocked.shape[1] and 0<=c[1]<blocked.shape[0] and not blocked[c[1],c[0]]
    if not clear(cell):return False
    for _ in range(blocked.size+2):
        if np.array_equal(cell,end):return True
        if abs(crossing[0]-crossing[1])<1e-10:
            if not clear(cell+[step[0],0]) or not clear(cell+[0,step[1]]):return False
            cell+=step;crossing+=delta
        else:
            k=int(np.argmin(crossing));cell[k]+=step[k];crossing[k]+=delta[k]
        if not clear(cell):return False
    return False

def terrain_direction(position, goal, cloud, side, cruise_z=None,lookahead=100.):
    p, goal = np.asarray(position, float), np.asarray(goal, float)
    if p.shape != (3,) or goal.shape != (3,) or not np.isfinite(p).all() or not np.isfinite(goal).all() or side not in (-1, 1):
        raise ValueError('Finite xyz points and side +/-1 required')
    altitude = float(p[2] if cruise_z is None else cruise_z)
    if not np.isfinite(altitude) or not 0 <= altitude <= 300:
        raise ValueError('Cruise altitude must be finite and <=300m')
    hold = dict(available=False, target=p.tolist(), path=[], reason='insufficient clear terrain in local context',
                input_scope='terrain_cloud_xy_radius_1000m', local_obstacles_verified=False)
    points = terrain_context(cloud, p)
    step, n = 20., 101
    origin = np.round(p[:2]/step)*step-1000
    cells = np.rint((points[:,:2]-origin)/step).astype(int)
    keep = (cells >= 0).all(1) & (cells < n).all(1)
    cells, points = cells[keep], points[keep]
    height = np.full((n,n), -np.inf)
    np.maximum.at(height, (cells[:,1], cells[:,0]), points[:,2])
    blocked = ~np.isfinite(height) | (height+15. > altitude)
    # Reserve 50m around the centerline (rounded outward to whole cells).
    inflated = blocked.copy()
    pad = np.pad(blocked, 3, constant_values=True)
    for row in range(7):
        for col in range(7):
            if (row-3)**2+(col-3)**2 <= 9:
                inflated |= pad[row:row+n,col:col+n]
    yy,xx = np.mgrid[:n,:n]
    xy = np.stack([origin[0]+xx*step, origin[1]+yy*step],axis=-1)
    inflated |= np.linalg.norm(xy-p[:2],axis=2)>1000
    terrain_inflated=inflated.copy()
    direction = goal[:2]-p[:2]; remaining = float(np.linalg.norm(direction))
    if remaining < 20:
        hold['reason']='terrain guide goal reached; phase controller must decide next phase'
        return hold
    direction /= remaining; normal = np.array([-direction[1],direction[0]])
    start = tuple(np.rint((p[:2]-origin)/step).astype(int))
    side_blocked = ((xy-p[:2])@normal)*side < -step*.5
    # The true start lies on the side boundary; rounding can put its cell up
    # to sqrt(2)*step/2 across that boundary. Exempt only this side-policy bit.
    # Terrain, unknown-data and formation-buffer occupancy remain authoritative.
    side_blocked[start[1],start[0]]=False
    inflated |= side_blocked
    if inflated[start[1],start[0]]:
        return hold
    forward = min(800., remaining)
    targets = [p[:2]+direction*forward+normal*side*lateral for lateral in (0,160,320,480)]
    if lookahead>100. and grid_line_clear((p[:2]-origin)/step,(targets[0]-origin)/step,terrain_inflated):
        # The exact center ray lies on the side-policy boundary. A quantized
        # side mask must not force staircase turns on an otherwise clear ray.
        # Check the full 800 m (or remaining) corridor, not just the guide point.
        target=np.r_[p[:2]+direction*min(lookahead,forward),altitude]
        path=np.array([np.r_[p[:2],altitude],np.r_[targets[0],altitude]])
        return dict(available=True,target=target.tolist(),path=path.tolist(),side=side,
                    input_points=len(points),grid_step_m=step,terrain_margin_m=15,formation_buffer_m=60,
                    input_scope='terrain_cloud_xy_radius_1000m',local_obstacles_verified=False,
                    reason='continuous clear terrain direction; local trajectory and stopping checks remain required')

    def search(target):
        end=tuple(np.rint((target-origin)/step).astype(int))
        if min(end)<0 or max(end)>=n or inflated[end[1],end[0]]: return None
        costs={start:0.}; prev={}; heap=[(math.hypot(end[0]-start[0],end[1]-start[1]),0.,start)]
        while heap:
            _,cost,k=heapq.heappop(heap)
            if cost>costs[k]+1e-9:continue
            if k==end:
                path=[k]
                while k!=start:k=prev[k];path.append(k)
                return np.array(path[::-1],float)*step+origin
            for dy in (-1,0,1):
                for dx in (-1,0,1):
                    if dx==dy==0:continue
                    q=(k[0]+dx,k[1]+dy)
                    if min(q)<0 or max(q)>=n or inflated[q[1],q[0]]:continue
                    if dx and dy and (inflated[k[1],q[0]] or inflated[q[1],k[0]]):continue
                    new=cost+math.hypot(dx,dy)
                    if new<costs.get(q,float('inf')):
                        costs[q]=new;prev[q]=k;heapq.heappush(heap,(new+math.hypot(end[0]-q[0],end[1]-q[1]),new,q))
        return None

    path=None
    for target in targets:
        path=search(target)
        if path is not None:break
    if path is None:return hold
    # Publish a short ordered guide point, never skip a terrain corner.
    lengths=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]
    index=min(len(path)-1,max(1,int(np.searchsorted(lengths,lookahead))))
    if lookahead>100.:
        # Only remove lattice stair-steps when the entire direct segment stays
        # in the same known, inflated local terrain corridor.
        while index>1 and not grid_line_clear((p[:2]-origin)/step,(path[index]-origin)/step,inflated):index-=1
    else:
        first=path[1]-path[0]
        for i in range(1,index):
            if not np.allclose(path[i+1]-path[i],first):
                index=i
                break
    path3=np.c_[path,np.full(len(path),altitude)]
    return dict(available=True,target=path3[index].tolist(),path=path3.tolist(),side=side,
                input_points=len(points),grid_step_m=step,terrain_margin_m=15,formation_buffer_m=60,
                input_scope='terrain_cloud_xy_radius_1000m',local_obstacles_verified=False,
                reason='terrain direction; local trajectory and stopping checks remain required')
