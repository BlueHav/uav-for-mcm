"""Explicit low-altitude region geometry; sampled checks are not flight acceptance."""
import math
import numpy as np
from display_sampling import world_height

STAR = np.array([[0., -3.3], [-3.3, 0.], [0., 0.], [3.3, 0.], [0., 3.3]])


def validate_assembly_region(config, world):
    region = config['assembly_region']
    center = np.asarray(region['center'], dtype=float)
    handoffs = np.asarray(region['handoffs'], dtype=float)
    radius = float(region['radius_m'])
    minimum = float(region['minimum_endpoint_distance_m'])
    length = float(region['descent_distance_m'])
    if center.shape != (2,) or handoffs.shape != (2, 2):
        raise ValueError('Assembly region requires center XY and two handoff XY points')
    if not np.isfinite(np.r_[center, handoffs.ravel(), radius, minimum, length]).all():
        raise ValueError('Nonfinite assembly region')
    if radius <= 0 or minimum < 1000 or length <= 0:
        raise ValueError('Invalid assembly radius, endpoint distance or descent distance')
    original = np.array([config['points']['landing'][k] for k in ('x', 'y')])
    if not np.allclose(center, original, atol=1e-6, rtol=0):
        raise ValueError('Assembly region center must preserve configured landing center')
    altitude = float(config['cruise_altitude'])
    agl = float(config['low_arrival_agl'])
    if not np.isfinite([altitude, agl]).all() or agl < 5:
        raise ValueError('Invalid assembly height')
    slots = handoffs[:, None, :] + STAR
    if np.any(np.linalg.norm(slots-center, axis=2) > radius):
        raise ValueError('Assembly slot outside configured region')
    all_slots = slots.reshape(-1, 2)
    separation = np.linalg.norm(all_slots[:, None]-all_slots[None, :], axis=2)
    separation += np.eye(10)*1e9
    if np.min(separation) < 2:
        raise ValueError('Assembly slots violate two metre separation')
    if np.linalg.norm(handoffs[0]-handoffs[1]) > 1000:
        raise ValueError('Assembly group centers exceed one kilometre')
    evidence = []
    for group, name in enumerate(('end', 'end2')):
        end = np.array([config['points'][name][k] for k in ('x', 'y')])
        distances = np.linalg.norm(slots[group]-end, axis=1)
        if np.min(distances) < minimum:
            raise ValueError('Assembly slot is too close to endpoint')
        previous = np.array([config['plan']['cruise_routes'][group][-2][k] for k in ('x', 'y')])
        direction = handoffs[group]-previous
        norm = np.linalg.norm(direction)
        if norm <= length:
            raise ValueError('Insufficient final cruise leg for descent')
        direction /= norm
        target = np.c_[slots[group], world_height(world, slots[group])+agl]
        entry = np.c_[slots[group]-length*direction, np.full(5, altitude)]
        if np.any(target[:, 2] >= altitude):
            raise ValueError('Assembly target is not below cruise altitude')
        # Sample all five body-center corridors at <= 0.5 m spatial intervals.
        samples = np.concatenate([np.linspace(a, b, int(math.ceil(np.linalg.norm(b-a)/.5))+1) for a, b in zip(entry, target)])
        terrain = world_height(world, samples[:, :2])
        if not np.isfinite(samples).all() or not np.isfinite(terrain).all():
            raise ValueError('Assembly descent leaves mapped world')
        clearance = samples[:, 2]-terrain
        if np.min(clearance) < 1.5:
            raise ValueError('Assembly descent terrain clearance below 1.5 m')
        for obj in world['objects']:
            if not {'width','depth','height','base_z','x','y'} <= obj.keys():
                raise ValueError('Unsupported map obstacle geometry')
            theta = np.deg2rad(obj.get('yaw', 0.))
            delta = samples[:, :2]-[obj['x'], obj['y']]
            x = delta[:, 0]*np.cos(theta)+delta[:, 1]*np.sin(theta)
            y = -delta[:, 0]*np.sin(theta)+delta[:, 1]*np.cos(theta)
            hit = ((np.abs(x) <= obj['width']/2+.5) & (np.abs(y) <= obj['depth']/2+.5) &
                   (samples[:, 2] >= obj.get('bottom_z', obj['base_z'])-.5) &
                   (samples[:, 2] <= obj['base_z']+obj['height']+.5))
            if np.any(hit):
                raise ValueError('Assembly descent intersects obstacle '+str(obj.get('id')))
        evidence.append(dict(group=group, minimum_endpoint_distance_m=float(np.min(distances)),
                             minimum_sampled_terrain_clearance_m=float(np.min(clearance)),
                             ground_heights_m=(target[:, 2]-agl).tolist(),
                             entry=entry.tolist(), targets=target.tolist(), samples=len(samples)))
    return dict(scope='sampled_static_geometry_only_not_flight_acceptance', handoffs=handoffs.tolist(),
                center=center.tolist(), radius_m=radius, groups=evidence,
                minimum_slot_separation_m=float(np.min(separation)),
                group_center_distance_m=float(np.linalg.norm(handoffs[0]-handoffs[1])))


def single_descent_height(entry_xy, end_xy, final_z, position_xy, ground_z):
    """Exact accepted single executor height target, including its last 60 m AGL leg."""
    entry, end, position = (np.asarray(q, float) for q in (entry_xy, end_xy, position_xy))
    if any(q.shape != (2,) for q in (entry, end, position)) or not np.isfinite(np.r_[entry, end, position, final_z, ground_z]).all():
        raise ValueError('Invalid single descent input')
    total = float(np.linalg.norm(end-entry))
    if total <= 60:
        raise ValueError('Single descent must be longer than 60 m')
    if np.linalg.norm(end-position) <= 60:
        return 'AGL', 5.
    progress = float(np.clip(np.dot(position-entry, end-entry)/(total*(total-60.)), 0, 1))
    return 'ABS', 270.+(float(final_z)-270.)*progress


def validate_single_descent(config, world, group=0):
    """Sample the executed target curve, not the former straight descent chord."""
    if group not in (0, 1) or isinstance(group, bool):
        raise ValueError('Invalid descent group')
    if float(config['cruise_altitude']) != 270. or float(config['low_arrival_agl']) != 5.:
        raise ValueError('Single executor requires 270 m cruise and 5 m arrival')
    geometry = validate_assembly_region(config, world)
    route = geometry['groups'][group]
    entry = np.asarray(route['entry'][2], float)
    target = np.asarray(route['targets'][2], float)
    total = float(np.linalg.norm(target[:2]-entry[:2]))
    # Explicit samples either side of the ABS/AGL boundary detect its height step.
    along = np.unique(np.r_[np.linspace(0, total, int(math.ceil(total/.5))+1), total-60.-1e-6, total-60., total-60.+1e-6])
    xy = entry[:2]+along[:, None]*(target[:2]-entry[:2])/total
    ground = world_height(world, xy)
    heights = [single_descent_height(entry[:2], target[:2], target[2], p, g) for p, g in zip(xy, ground)]
    z = np.array([h if mode == 'ABS' else g+h for (mode, h), g in zip(heights, ground)])
    if not np.isfinite(np.r_[xy.ravel(), ground, z]).all() or np.min(z-ground) < 1.5:
        raise ValueError('Executed single descent violates terrain clearance')
    for obj in world['objects']:
        theta = np.deg2rad(obj.get('yaw', 0.));delta = xy-[obj['x'], obj['y']]
        x = delta[:, 0]*np.cos(theta)+delta[:, 1]*np.sin(theta)
        y = -delta[:, 0]*np.sin(theta)+delta[:, 1]*np.cos(theta)
        hit = ((np.abs(x) <= obj['width']/2+.5) & (np.abs(y) <= obj['depth']/2+.5) &
               (z >= obj.get('bottom_z', obj['base_z'])-.5) & (z <= obj['base_z']+obj['height']+.5))
        if np.any(hit):
            raise ValueError('Executed single descent intersects obstacle '+str(obj.get('id')))
    return dict(scope='sampled_executed_height_targets_not_actual_flight', group=group,
                entry=entry.tolist(), target=target.tolist(), sample_count=len(xy),
                minimum_terrain_clearance_m=float(np.min(z-ground)),
                maximum_adjacent_target_height_change_m=float(np.max(np.abs(np.diff(z)))),
                profile='ABS 270 to final_z by length-60, last 60 m terrain+5')
