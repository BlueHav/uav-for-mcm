"""Validated group geometry; Runtime execution is server-only."""
import copy
import math


def _point(value, dimensions, label):
    if not isinstance(value, (list, tuple)) or len(value) != dimensions:
        raise ValueError(label + ': invalid dimensions')
    if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in value):
        raise ValueError(label + ': nonfinite coordinate')
    return [float(x) for x in value]


def validate_config(config, deployment=False):
    cfg = copy.deepcopy(config)
    center = _point(cfg['region_center'], 2, 'region center')
    radius = cfg['region_radius_m']
    if isinstance(radius, bool) or not isinstance(radius, (int, float)) or not math.isfinite(radius) or not 0 < radius <= 120:
        raise ValueError('region radius must be within the authorized 120 m')
    routes = {}
    for key, route in cfg['routes'].items():
        try:
            route_id = int(key)
        except (TypeError, ValueError):
            raise ValueError('invalid route id')
        if str(route_id) != str(key) or route_id in routes:
            raise ValueError('ambiguous route id')
        item = {name: _point(route[name], 3, name) for name in ('entry', 'handoff', 'end')}
        if math.hypot(item['handoff'][0]-center[0], item['handoff'][1]-center[1]) > radius:
            raise ValueError('handoff outside L region')
        if math.dist(item['entry'], item['handoff']) < 1 or item['entry'][2] <= item['handoff'][2]:
            raise ValueError('invalid descent corridor')
        routes[route_id] = item
    groups = cfg['groups']
    if not isinstance(groups, list) or not groups:
        raise ValueError('groups are required')
    seen_ids, members = set(), set()
    for group in groups:
        gid = group['id']
        if not isinstance(gid, str) or not gid or gid in seen_ids:
            raise ValueError('duplicate or invalid group id')
        seen_ids.add(gid)
        if isinstance(group['route_id'], bool) or not isinstance(group['route_id'], int) or group['route_id'] not in routes:
            raise ValueError('unknown group route')
        if not isinstance(group['members'], list) or not group['members']:
            raise ValueError('empty group')
        for member in group['members']:
            if isinstance(member, bool) or not isinstance(member, int) or member < 0 or member in members:
                raise ValueError('duplicate or invalid member')
            members.add(member)
    if deployment and (len(groups) != 2 or any(len(g['members']) != 1 for g in groups) or members != {0, 1} or {g['route_id'] for g in groups} != {0, 1}):
        raise ValueError('this flight deployment supports only two singleton groups, ids 0 and 1')
    if deployment and any(g['members'][0] != g['route_id'] for g in groups):
        raise ValueError('this flight deployment binds each aircraft to its original endpoint route')
    cfg.update(region_center=center, region_radius_m=float(radius), routes=routes, vehicle_ids=sorted(members))
    return cfg
