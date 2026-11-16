"""Physical inactive bodies from Unity observations, never mission/ACK stand-ins."""
import numpy as np

def scene_observations(scene,received,now,active_ids,world_hash):
    if not np.isfinite([received,now]).all() or not -.05<=now-received<=.6:raise ValueError('Inactive scene observation stale')
    if scene.get('drone_count')!=10 or not isinstance(scene.get('frames'),int) or scene['frames']<1:raise ValueError('Inactive scene incomplete')
    if scene.get('map_sha256')!=world_hash:raise ValueError('Inactive scene map mismatch')
    if any(not isinstance(scene.get(k),int) or scene[k]!=0 for k in ('collision_events','bounds_events')):raise ValueError('Inactive scene event evidence missing/unsafe')
    drones=scene.get('drones',[])
    if len(drones)!=10 or {d.get('id') for d in drones}!=set(range(10)):raise ValueError('Inactive body identities missing/duplicated')
    active=set(active_ids)
    if not active or not active<=set(range(10)):raise ValueError('Invalid active member set')
    result={}
    for d in drones:
        if d['id'] in active:continue
        p=[d.get(k,float('nan')) for k in ('x','y','z')];speed=d.get('speed',float('nan'))
        if not np.isfinite(p+[speed]).all() or not 0<=speed<=.035:raise ValueError('Inactive body not observed stationary')
        result[d['id']]=dict(position=p,observed_speed=speed,received_stamp=received,scene_frame=scene['frames'],world_hash=world_hash,source='unity_scene')
    return result

def reservation_positions(observations,now):
    result={}
    for i,s in observations.items():
        p=np.asarray(s.get('position'),float)
        if s.get('source')!='unity_scene' or p.shape!=(3,) or not np.isfinite(p).all() or not -.05<=now-s.get('received_stamp',-1e9)<=.6 or not 0<=s.get('observed_speed',1e9)<=.035:raise ValueError('Inactive reservation missing/stale/moving')
        result[i]=p.tolist()
    return result
