"""Conservative union of full abort boxes grouped by nominal start time."""
import numpy as np
from terminal_formation import environment_clear,stop_clouds,_corners


def outer_box(corridor):
    c=np.asarray(corridor)
    return _corners(c.min(axis=(0,1)),c.max(axis=(0,1)))


def corridor_clear(grid,corridor):
    return all(environment_clear(grid,box) for box in corridor)


def contains_stops(corridor,paths):
    c=np.asarray(corridor);margin=.10/np.sqrt(3.)
    lo=c.min(axis=1)-margin;hi=c.max(axis=1)+margin
    # Each entire actual brake must fit one certified box. This is stronger
    # than testing discrete points in the union and requires no 50 Hz voxelization.
    return all(np.any(np.all(lo<=q.min(0),axis=1)&np.all(hi>=q.max(0),axis=1)) for q in paths)


def validated_box(plan,supplied):
    corridor=getattr(plan,'stop_corridor',None)
    if corridor is None:box=stop_clouds([plan])[0]
    else:
        expected=stop_clouds([plan],corridors=True)[0]
        if corridor.shape!=expected.shape or not np.allclose(corridor,expected,atol=1e-6,rtol=0):
            raise ValueError('Stopping corridor mismatch')
        box=outer_box(expected)
    supplied=np.asarray(supplied,float)
    if supplied.shape!=box.shape or not np.allclose(supplied,box,atol=1e-6,rtol=0):raise ValueError('Stopping box mismatch')
    return box


def packet_clear(grid,packet,member):
    if 'corridors' in packet:return corridor_clear(grid,packet['corridors'][member])
    return environment_clear(grid,np.asarray(packet['envelopes'][member]))


def unknown_view_heading(grid,packet,member,position):
    """Point the camera at a missing observation without changing the map."""
    if 'corridors' not in packet or not all(hasattr(grid,k) for k in
            ('origin','shape','resolution','state','UNKNOWN')) or grid.origin is None:
        return None
    for box in packet['corridors'][member]:
        box=np.asarray(box)
        lo=np.floor((box.min(0)-.5)/grid.resolution).astype(int)-grid.origin
        hi=np.floor((box.max(0)+.5)/grid.resolution).astype(int)-grid.origin
        if np.any(lo<0) or np.any(hi>=grid.shape):
            continue
        cells=tuple(slice(int(a),int(b)+1) for a,b in zip(lo,hi))
        unknown=np.argwhere(grid.state[cells]==grid.UNKNOWN)
        offsets=(unknown+lo+grid.origin+.5)*grid.resolution-position
        distance=np.linalg.norm(offsets[:,:2],axis=1)
        visible=(distance>=grid.resolution*.5)&(np.abs(offsets[:,2])<=.5*distance)
        if np.any(visible):
            x,y=offsets[np.argmin(np.where(visible,distance,np.inf)),:2]
            return float(np.arctan2(y,x))
    return None


def owned_packets(client,candidate=None):
    packets=[] if client.packet is None else [client.packet]
    c=getattr(client,'candidate',None)
    if c and c.get('accepted'):packets.append(c['packet'])
    if candidate is not None:packets.append(candidate)
    return list({p['digest']:p for p in packets}.values())


def owned_clear(client,grid,candidate=None):
    return all(packet_clear(grid,p,client.m.id%5) for p in owned_packets(client,candidate))


def observed_conflict(grid,corridor,detail=None,observer=None):
    if not all(hasattr(grid,k) for k in ('state','origin','shape','resolution','ground','FREE')) or grid.origin is None:
        return not corridor_clear(grid,corridor)
    for box in corridor:
        box=np.asarray(box);lower=box.min(0)-.5;upper=box.max(0)+.5
        lo=np.maximum(0,np.floor(lower/grid.resolution).astype(int)-grid.origin)
        hi=np.minimum(grid.shape-1,np.floor(upper/grid.resolution).astype(int)-grid.origin)
        if np.any(lo>hi):continue
        cells=tuple(slice(int(a),int(b)+1) for a,b in zip(lo,hi))
        occupied=grid.state[cells]==getattr(grid,'OCCUPIED',2)
        if np.any(occupied):
            if detail is not None:
                cells=np.argwhere(occupied)+lo+grid.origin
                points=(cells+.5)*grid.resolution
                if observer is None:
                    world_cell=cells[0]
                else:
                    offsets=points-np.asarray(observer)
                    world_cell=cells[np.argmin(np.linalg.norm(offsets,axis=1))]
                detail.update(kind='occupied',position=((world_cell+.5)*grid.resolution).tolist(),
                              box_min=box.min(0).tolist(),box_max=box.max(0).tolist())
                hit=getattr(grid,'near_depth_returns',{}).get(tuple(world_cell.tolist()))
                if hit is not None:detail['near_return']=hit
            return True
        ground=grid.ground[lo[0]:hi[0]+2,lo[1]:hi[1]+2]
        terrain=np.isfinite(ground)&(ground>=lower[2])
        if np.any(terrain):
            if detail is not None:
                cell=np.argwhere(terrain)[0]
                xy=(cell+lo[:2]+grid.origin[:2])*grid.resolution
                detail.update(kind='terrain',position=[*xy.tolist(),float(ground[tuple(cell)])])
            return True
    return False


def packet_conflict(grid,packet,member,detail=None,observer=None):
    corridor=packet['corridors'][member] if 'corridors' in packet else [packet['envelopes'][member]]
    return observed_conflict(grid,corridor,detail,observer)


def owned_conflict(client,grid,candidate=None):
    return any(packet_conflict(grid,p,client.m.id%5) for p in owned_packets(client,candidate))
