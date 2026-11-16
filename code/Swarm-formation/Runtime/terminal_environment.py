"""Immutable stopping-box observation from a complete, locked map boundary."""
from dataclasses import dataclass
from types import SimpleNamespace
import numpy as np
from terminal_formation import environment_clear
from terminal_corridor import owned_clear,owned_conflict


@dataclass(frozen=True)
class EnvironmentCertificate:
    generation: int
    owner: tuple
    slots: tuple
    lower: tuple
    upper: tuple
    sensor_stamp: float
    checked: float
    clear: bool
    conflict: bool


@dataclass(frozen=True)
class MapObservation:
    generation: int
    grid: object
    lower: tuple
    upper: tuple
    sensor_stamp: float
    checked: float
    clear: bool


def slots(client):
    result=[(client.packet['digest'],client.start)]
    candidate=getattr(client,'candidate',None)
    if candidate and candidate.get('accepted'):
        result.append((candidate['packet']['digest'],candidate['start']))
    return tuple(result)


def observe(client,now):
    m=client.m;box=np.array(client.box,copy=True)
    stamps=[m.map.ground_stamp,m.map.depth_stamp]
    stamps += [m.handoff.integrated.get(name) for name in ('front','up','down')]
    stamp=min(stamps) if all(s is not None and np.isfinite(s) for s in stamps) else -1e9
    grid=m.map;frozen=None;clear=False
    if all(hasattr(grid,k) for k in ('state','origin','shape','ground','resolution','FREE')) and grid.origin is not None:
        lo=np.maximum(0,np.floor((box.min(0)-.5)/grid.resolution).astype(int)-grid.origin-1)
        hi=np.minimum(grid.shape,np.floor((box.max(0)+.5)/grid.resolution).astype(int)-grid.origin+2)
        if np.all(hi>lo):
            state=grid.state[tuple(slice(int(a),int(b)) for a,b in zip(lo,hi))].copy()
            ground=grid.ground[lo[0]:hi[0]+1,lo[1]:hi[1]+1].copy()
            origin=grid.origin+lo;shape=np.array(state.shape)
            for a in (state,ground,origin,shape):a.flags.writeable=False
            frozen=SimpleNamespace(state=state,ground=ground,origin=origin,shape=shape,
                                   resolution=grid.resolution,FREE=grid.FREE)
    else:
        # Deterministic geometry adapters used by standalone tests/replays.
        # Freeze their result, never retain a callback to the mutable map.
        clear=bool(owned_clear(client,grid))
    with client.observation_lock:
        client.environment_generation+=1
        client.map_observation=MapObservation(client.environment_generation,frozen,
            tuple(box.min(0)),tuple(box.max(0)),stamp,now,clear)


def capture(client,now):
    observation=client.map_observation
    if observation is None:return None
    packet=client.packet;box=np.asarray(client.box)
    # A newer/larger reservation needs its own complete observation; promotion
    # to a subset of the observed old/new union is allowed without a blind gap.
    if np.any(box.min(0)<np.asarray(observation.lower)-1e-9) or np.any(box.max(0)>np.asarray(observation.upper)+1e-9):return None
    clear=observation.clear if observation.grid is None else owned_clear(client,observation.grid)
    conflict=not clear if observation.grid is None else owned_conflict(client,observation.grid)
    return EnvironmentCertificate(observation.generation,(packet['run_id'],packet['epoch'],packet['group']),
        slots(client),tuple(box.min(0)),tuple(box.max(0)),observation.sensor_stamp,observation.checked,bool(clear),bool(conflict))


def covers(certificate,client):
    if certificate is None or client.packet is None:return False
    if certificate.generation!=client.environment_generation:return False
    packet=client.packet
    if certificate.owner!=(packet['run_id'],packet['epoch'],packet['group']):return False
    required=set(client.proof_digests() if hasattr(client,'proof_digests') else [packet['digest']])
    current=dict(slots(client));covered=dict(certificate.slots)
    if any(d not in covered or covered[d]!=current.get(d) for d in required):return False
    # After promotion the smaller new reservation may use the old/new map
    # observation, but actual stopping is checked against the CURRENT box.
    box=np.asarray(client.box)
    return bool(np.all(box.min(0)>=np.asarray(certificate.lower)-1e-9) and
                np.all(box.max(0)<=np.asarray(certificate.upper)+1e-9))


def owns_execution(client):
    m=client.m;five=getattr(m,'five',None)
    return bool(client.committed and client.packet and client.ownership(client.packet)
        and m.armed and not m.failure and not m.stop_pending and not m.controller_fault
        and m.phase in ('TERRAIN_FOLLOW','END_HOLD')
        and five is not None and five.active and not five.suspended)
