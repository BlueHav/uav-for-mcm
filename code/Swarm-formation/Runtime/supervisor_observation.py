"""Measured control state can outlive a delayed planner report, never its sensors."""
import numpy as np
from numbers import Real
from static_perception import matching_permission


def recent(value,now,limit):
    return isinstance(value,Real) and not isinstance(value,bool) and np.isfinite(value) and -.05<=now-value<=limit


def measured_execution(state, identity, now, world, commits=()):
    e=state.get('execution',{});o=e.get('observation') or {}
    if (state.get('phase') not in ('TERRAIN_FOLLOW','END_HOLD')
            or not isinstance(world,str) or not world or not e.get('perception_enabled')
            or e.get('error') or not e.get('armed') or e.get('holding')
            or o.get('drone_id')!=identity or o.get('stamp')!=e.get('odom_stamp')
            or not recent(e.get('odom_stamp'),now,.5)
            or not recent(e.get('stop_checked'),now,.15)):return None
    p=np.asarray(o.get('position',[]),float);v=np.asarray(o.get('velocity',[]),float)
    if p.shape!=(3,) or v.shape!=(3,) or not np.isfinite(p).all() or not np.isfinite(v).all():return None
    joint=state.get('terminal_joint',{})
    slots=[s for s in commits if s and s.get('committed')]
    if joint.get('committed'):
        slots.append(dict(packet=dict(run_id=state.get('planning_run_id'),epoch=2,digest=joint.get('digest')),start=joint.get('start')))
    allowed=any(s['packet'].get('run_id')==state.get('planning_run_id') and s['packet'].get('epoch')==2
        and e.get('active')==s['packet'].get('digest') and e.get('active_start')==s.get('start')
        and matching_permission(e,s['packet'],s.get('start'),now,world) for s in slots)
    if not allowed:return None
    # Do not redate stamp, frame ages, map stamps, plan age or landing ACKs.
    result=dict(state,position=p.tolist(),velocity=v.tolist(),speed=float(np.linalg.norm(v)),
                abs=float(p[2]),odom_age=now-o['stamp'],kinematic_stamp=o['stamp'])
    reference=np.asarray(e.get('reference') if e.get('reference') is not None else [],float)
    if reference.shape==(3,3) and np.isfinite(reference).all():
        result['control_reference']=dict(stamp=e['stamp'],position=reference[0].tolist(),
            velocity=reference[1].tolist(),acceleration=reference[2].tolist())
    return result
