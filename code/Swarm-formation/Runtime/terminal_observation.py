"""Bounded observed planning hints, never a free-space certificate."""
import numpy as np
from terminal_formation import packet_paths


def obstacle_hints(grid,position,limit=512):
    if not all(hasattr(grid,k) for k in ('state','origin','resolution','OCCUPIED')) or grid.origin is None:return []
    occupied=grid.state==grid.OCCUPIED
    if hasattr(grid,'peer_only'):occupied &= ~grid.peer_only
    cells=np.argwhere(occupied)+grid.origin
    if len(cells) and hasattr(grid,'ground_at_many'):
        centers=(cells+.5)*grid.resolution
        ground=grid.ground_at_many(centers[:,:2])
        # Native optimization already gets a separate terrain layer. Do not
        # turn a ground return into a floating sphere by coarsening its cell.
        cells=cells[~np.isfinite(ground)|(centers[:,2]-ground>grid.resolution/2.+.10)]
    if not len(cells):return []
    # Cover occupied fine voxels by enclosing coarse cubes. Unknown cells do
    # not create free-space claims. Independent admission sees the full map.
    coarse=np.unique(np.floor_divide(cells,4),axis=0)
    size=4*grid.resolution;centers=(coarse+.5)*size
    distance=np.sum((centers-np.asarray(position))**2,axis=1)
    if len(centers)>limit:centers=centers[np.argpartition(distance,limit-1)[:limit]]
    return np.c_[centers,np.full(len(centers),np.sqrt(3)*size/2.)].tolist()


def optimizer_obstacles(intents, observed_positions):
    hints=[hint for intent in intents for hint in intent.get('obstacles',[])]
    if not hints:
        return []
    spheres=np.asarray(hints,float)
    offsets=spheres[:,None,:3]-np.asarray(observed_positions,float)[None,:,:]
    overlaps=np.any(np.sum(offsets**2,axis=2)<=spheres[:,3,None]**2,axis=1)
    # Coarsening can wrap a real drone return or a nearby surface around a
    # measured body. Omit the contradictory seed sphere only. Exact occupied
    # cells still participate in all five independent executor admissions.
    return spheres[~overlaps][:2560].tolist()


def nearby_trees(world,positions):
    trees=np.array([[o['x'],o['y'],o['base_z'],o['height'],o['radius']]
                    for o in world.get('objects',()) if o.get('kind')=='tree'],float).reshape(-1,5)
    if not len(trees):return trees
    distance=np.linalg.norm(trees[:,None,:2]-np.asarray(positions)[None,:,:2],axis=2)
    return trees[np.any(distance<=60.,axis=1)]


def tree_canopies(trees):
    return trees[:,:3]+np.c_[np.zeros((len(trees),2)),trees[:,3]*.70]


def tree_corridors_clear(trees,corridors):
    if not len(trees):return True
    boxes=np.concatenate(corridors,axis=0)
    lower,upper=boxes.min(1)-.5,boxes.max(1)+.5
    canopies=tree_canopies(trees)
    radii=np.c_[trees[:,4],trees[:,4],trees[:,3]*.30]
    delta=np.maximum(np.maximum(lower[:,None]-canopies,canopies-upper[:,None]),0.)
    if np.any(np.sum((delta/radii)**2,axis=2)<=1.):return False
    xy=trees[None,:,:2]
    delta=np.maximum(np.maximum(lower[:,None,:2]-xy,xy-upper[:,None,:2]),0.)
    stem_xy=np.sum(delta**2,axis=2)<=.275**2
    stem_z=(upper[:,None,2]>=trees[:,2])&(lower[:,None,2]<=trees[:,2]+trees[:,3]*.55)
    return not np.any(stem_xy&stem_z)


def prioritized_directions(intents,toward,blocked=(),tree_detour=False,escape=False):
    order=list(dict.fromkeys([*blocked,2,0,1,3,4]))
    choices=[]
    deferred=[]
    forward=np.asarray(toward,float)/max(np.linalg.norm(toward),1e-9)
    for member in order:
        for index,candidate in enumerate(intents[member]['directions']):
            vector=np.asarray(candidate,float)
            if (vector.shape!=(2,) or not np.isfinite(vector).all()
                    or abs(np.linalg.norm(vector)-1.)>.01):
                continue
            alignment=float(np.dot(vector,forward))
            if member in blocked and alignment<-.15 and not escape:
                deferred.append((vector,member,index))
                continue
            if member!=2 and member not in blocked and alignment<=0.:
                continue
            if all(np.dot(vector,old[0])<.985 for old in choices):
                choices.append((vector,member,index))
    if tree_detour and choices and np.linalg.norm(toward)>1e-9:
        # A star beside a canopy may need a short outward segment first.
        lateral=np.array([-forward[1],forward[0]])
        for angle in np.deg2rad((75.,-75.,105.,-105.,135.,-135.)):
            vector=forward*np.cos(angle)+lateral*np.sin(angle)
            if all(np.dot(vector,old[0])<.985 for old in choices):
                choices.append((vector,None,None))
    # A blocked member may need to leave a wall before resuming the route.
    # Retain its actual YOPO directions after the goalward/lateral choices.
    for vector,member,index in deferred:
        if all(np.dot(vector,old[0])<.985 for old in choices):
            choices.append((vector,member,index))
    return choices


def curve_reference(intent,index,start,goal):
    curves=intent.get('curves',[])
    if index>=len(curves):return np.empty((0,3))
    curve=np.asarray(curves[index],float)
    if curve.ndim!=2 or curve.shape[1]!=3 or len(curve)<2 or not np.isfinite(curve).all():return np.empty((0,3))
    length=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(curve,axis=0),axis=1))]
    if length[-1]<.1:return np.empty((0,3))
    u=np.linspace(0.,1.,21)
    samples=np.array([np.interp(u*length[-1],length,curve[:,i]) for i in range(3)]).T
    # Retain the curve's lateral shape while fitting the future start and
    # chosen local endpoint. It remains a seed, subject to full validation.
    return samples+(1-u[:,None])*(start-samples[0])+u[:,None]*(goal-samples[-1])


def peer_predictions(reference,states,group,duration):
    slot=reference.get('peer_prediction');start=reference.get('planning_start',0.)
    if slot:
        plans=packet_paths(slot['packet']);times=np.linspace(start,start+duration,21)
        return np.stack([p.positions(np.clip(times-slot['start'],0,p.duration)) for p in plans],axis=1)
    if all(i in states for i in range(5*(1-group),5*(1-group)+5)):
        return np.tile(np.array([states[i]['position'] for i in range(5*(1-group),5*(1-group)+5)])[None],(21,1,1))
    return np.empty((21,0,3))


def unity_peer_envelopes(position,rotation,stamp,peers):
    """Camera-relative body envelopes from time-aligned peer observations."""
    for peer in peers.values():
        try:
            delta=stamp-float(peer['stamp'])
            p=np.asarray(peer['position'],float)
            v=np.asarray(peer['velocity'],float)
        except (KeyError,TypeError,ValueError):
            continue
        if abs(delta)>.1 or p.shape!=(3,) or v.shape!=(3,) or not np.isfinite(np.r_[p,v,delta]).all():
            continue
        center=(p+delta*v-position)@rotation
        # Enclose the Tracker mesh under arbitrary yaw, plus sample skew.
        radius=.37+abs(delta)*min(np.linalg.norm(v),6.)
        if center[0]>radius:
            yield center,radius


def unity_peer_depth_mask(depth, position, rotation, stamp, peers, focal, cx=240., cy=135.):
    """Identify real returns inside fresh, motion-aligned UAV body envelopes."""
    depth=np.asarray(depth)
    mask=np.zeros(depth.shape,bool)
    for center,radius in unity_peer_envelopes(position,rotation,stamp,peers):
        x,y,z=center
        u=cx-focal*y/x
        w=cy-focal*z/x
        extent=focal*radius/(x-radius)*(1.+max(abs(y),abs(z))/x)
        x0=max(0,int(np.floor(u-extent)));x1=min(depth.shape[1],int(np.ceil(u+extent))+1)
        y0=max(0,int(np.floor(w-extent)));y1=min(depth.shape[0],int(np.ceil(w+extent))+1)
        if x0>=x1 or y0>=y1:
            continue
        yy,xx=np.ogrid[y0:y1,x0:x1]
        d=depth[y0:y1,x0:x1]
        error=(d-x)**2+(-(xx-cx)*d/focal-y)**2+(-(yy-cy)*d/focal-z)**2
        mask[y0:y1,x0:x1]|=np.isfinite(d)&(d>.05)&(error<=radius**2)
    return mask
