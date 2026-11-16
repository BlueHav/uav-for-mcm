"""Independent actual-motion audit. Execute on the SSH server only."""
import csv
import json
import sys
from pathlib import Path

import numpy as np


def sustained_start(stamps, mask, duration):
    start = None
    previous = None
    for stamp, valid in zip(stamps, mask):
        if not valid or previous is not None and stamp - previous > .100001:
            start = None
        if valid:
            if start is None:
                start = float(stamp)
            if stamp - start >= duration:
                return start
        previous = float(stamp)
    return None


def terrain_height(world, x, y):
    b, h = world['bounds'], world['terrain']
    grid = np.asarray(h['heights']).reshape(h['height'], h['width'])
    u = np.clip((x-b['x_min'])/(b['x_max']-b['x_min'])*(h['width']-1), 0, h['width']-1)
    v = np.clip((b['y_max']-y)/(b['y_max']-b['y_min'])*(h['height']-1), 0, h['height']-1)
    c = np.minimum(np.floor(u).astype(int), h['width']-2)
    r = np.minimum(np.floor(v).astype(int), h['height']-2)
    u, v = u-c, v-r
    a, bb, d, e = grid[r,c], grid[r,c+1], grid[r+1,c], grid[r+1,c+1]
    return np.where(u+v <= 1, a+(bb-a)*u+(d-a)*v, e+(d-e)*(1-u)+(bb-e)*(1-v))


def audit(base):
    manifest = json.loads((base/'test-manifest.json').read_text())
    cfg = manifest['parallel_descent']
    ids = sorted(int(i) for i in cfg['entries'])
    if len(ids) != 2:
        raise ValueError('This actual-flight audit requires exactly two aircraft')
    world = json.loads(Path(('__UAV_PROJECT__/config/maps/GeoMap10km/world.json'.replace("__UAV_PROJECT__", __import__("os").environ["UAV_PROJECT_ROOT"]))).read_text())
    commands = {}
    with (base/'telemetry.jsonl').open() as stream:
        for line in stream:
            if 'central_stage_command' not in line:
                continue
            event = json.loads(line)
            if event.get('event') == 'central_stage_command' and event.get('command',{}).get('_forward_descent'):
                commands.setdefault(int(event['drone_id']), float(event['stamp']))
    result = dict(passed=False, scope='Actual odometry and original terrain, not commanded timing',
                  start_definition='After descent task: >2m projected progress, Z<269.5m, projected velocity>.2m/s and vz<-.05m/s sustained1s',
                  passage_definition='Horizontal radius<=120m and terrain AGL5+/-0.25m sustained0.5s',
                  moving_definition='Both in their measured descent intervals and actual projected forward velocity>.2m/s; holds excluded',
                  coordination_semantics='Two independent group leaders; timing differences and movement fraction are descriptive, not synchronization targets. Safety or formation pauses require separate reason audit.',
                  baseline_note='Old261.009s arrival gap was at exact L; this run uses user-authorized120m region',
                  drones={}, errors=[])
    arrays, directions = {}, {}
    for i in ids:
        rows = list(csv.DictReader((base/('trajectory-drone-%d.csv'%i)).open()))
        a = np.asarray([[float(row[k]) for k in ('stamp','x','y','z','vx','vy','vz')] for row in rows])
        if len(a)<2 or not np.isfinite(a).all() or not np.all(np.diff(a[:,0])>0):
            raise ValueError('Invalid or nonmonotonic actual odometry for %d'%i)
        arrays[i] = a
        entry = np.asarray(cfg['entries'][str(i)], float)
        handoff = np.asarray(cfg['handoffs'][str(i)], float)
        direction = handoff[:2]-entry[:2]
        direction /= np.linalg.norm(direction)
        directions[i] = direction
        progress = (a[:,1:3]-entry[:2]) @ direction
        along_v = a[:,4:6] @ direction
        agl = a[:,3]-terrain_height(world,a[:,1],a[:,2])
        radius = np.linalg.norm(a[:,1:3]-np.asarray(cfg['region_center'])[:2],axis=1)
        if i not in commands:
            result['errors'].append('Missing actual descent command for %d'%i)
            continue
        start = sustained_start(a[:,0], (a[:,0]>=commands[i]) & (progress>2) & (a[:,3]<269.5) & (along_v>.2) & (a[:,6]<-.05), 1.)
        end = None if start is None else sustained_start(a[:,0], (a[:,0]>=start) & (radius<=120.) & (np.abs(agl-5)<=.25), .5)
        d = dict(command_stamp=commands[i],actual_start_stamp=start,actual_region_passage_stamp=end,
                 maximum_odom_gap_s=float(np.max(np.diff(a[:,0]))),entry=entry.tolist(),handoff=handoff.tolist())
        if start is None or end is None:
            result['errors'].append('Missing sustained actual descent or regional AGL5 passage for %d'%i)
        else:
            n = int(np.searchsorted(a[:,0],end))
            d.update(duration_s=end-start,passage_position=a[n,1:4].tolist(),passage_agl_m=float(agl[n]),passage_radius_m=float(radius[n]))
            relevant_gaps = (a[:-1,0]<end) & (a[1:,0]>start)
            if np.any(np.diff(a[:,0])[relevant_gaps]>.100001):
                result['errors'].append('Actual descent odometry gap exceeds 0.1s for %d; cannot certify movement across missing data'%i)
        result['drones'][str(i)] = d
    if not result['errors']:
        ds = [result['drones'][str(i)] for i in ids]
        starts = [d['actual_start_stamp'] for d in ds]
        ends = [d['actual_region_passage_stamp'] for d in ds]
        lo, hi = min(starts), max(ends)
        ts = np.unique(np.concatenate([a[(a[:,0]>lo)&(a[:,0]<hi),0] for a in arrays.values()] + [np.asarray([lo,hi,*starts,*ends])]))
        mid, dt = (ts[:-1]+ts[1:])*.5, np.diff(ts)
        masks = []
        for i,d in zip(ids,ds):
            a = arrays[i]
            speed = np.column_stack([np.interp(mid,a[:,0],a[:,j]) for j in (4,5)]) @ directions[i]
            active = (mid>=d['actual_start_stamp']) & (mid<=d['actual_region_passage_stamp'])
            masks.append(active & (speed>.2))
            d['not_progressing_seconds'] = float(np.sum(dt[active & (speed<=.2)]))
        together = float(np.sum(dt[masks[0]&masks[1]]))
        longest = max(d['duration_s'] for d in ds)
        start_difference, arrival_difference = abs(starts[0]-starts[1]),abs(ends[0]-ends[1])
        result.update(actual_start_difference_s=start_difference,actual_region_passage_difference_s=arrival_difference,
                      interval_overlap_s=max(0.,min(ends)-max(starts)),simultaneous_progress_seconds=together,
                      longest_descent_duration_s=longest,simultaneous_progress_fraction=together/longest,
                      command_start_difference_s=abs(commands[ids[0]]-commands[ids[1]]))
        result['passed'] = bool(together>0)
        result['pass_scope'] = 'Both complete observed regional descent with actual movement overlap; does not alone prove independent scheduling or full-flight safety'
    return result


if __name__ == '__main__':
    base = Path(sys.argv[1])
    report = audit(base)
    (base/'independent-parallel-audit.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
    raise SystemExit(0 if report['passed'] else 1)
