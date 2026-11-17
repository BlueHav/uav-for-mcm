#!/usr/bin/env python3
"""Server-only measured comparison; sampling and scope are explicit."""
import argparse,json,math
from pathlib import Path


def speed_metrics(samples):
    selected=[]
    for stamp,speed in samples:
        if not selected or stamp-selected[-1][0]>=.95:selected.append((stamp,speed))
    variation=distance=duration=slow_seconds=0.;entries=0;moving=False
    for (t0,v0),(t,v) in zip(selected,selected[1:]):
        dt=t-t0
        if dt>2.:moving=False;continue
        duration+=dt;distance+=(v0+v)*dt/2;variation+=abs(v-v0)
        if v>=1.5:moving=True
        if v<.3:
            slow_seconds+=dt
            if moving:entries+=1;moving=False
    return dict(samples_1hz=len(selected),duration_s=duration,estimated_travel_m=distance,
        mean_speed_mps=distance/max(duration,1e-9),speed_total_variation_mps=variation,
        variation_per_km=variation/max(distance/1000.,1e-9),slow_entries=entries,slow_seconds=slow_seconds)


def measure(run):
    run=Path(run)
    candidates=list((run/'Reports').rglob('telemetry.jsonl'))
    if len(candidates)!=1:raise ValueError('Expected one telemetry stream')
    samples={i:[] for i in range(10)};latches={i:[0,0] for i in range(10)}
    takeover={};network_after={};last={};near={};landed={}
    for line in candidates[0].open():
        row=json.loads(line);status=row['status'];stamp=status['stamp']
        if status.get('planner_mode')!='YOPO':continue
        states={int(i):s for i,s in row['states'].items()};last.update(states)
        for i,s in states.items():
            formation=s.get('formation',{});mode=formation.get('star_mode','YOPO')
            if formation.get('star_started'):
                takeover.setdefault(i,dict(stamp=stamp,token=formation.get('star_token'),
                    inference_count=formation.get('star_inference_at_start')))
                base=takeover[i]['inference_count']
                network_after[i]=s.get('network_inferences',0)-base if base is not None else None
            if s['phase']=='LANDED':landed.setdefault(i,stamp)
            if s['phase']!='TERRAIN_FOLLOW':continue
            distance=math.dist(s['position'][:2],s['goal'][:2])
            if distance<30:near.setdefault(i,stamp)
            if mode!='YOPO' or distance<=30:continue
            samples[i].append((stamp,math.hypot(*s['velocity'][:2])))
            latches[i][0]+=s['reason']=='Safety stop latched until velocity settles';latches[i][1]+=1
    members={i:dict(speed_metrics(samples[i]),latched_fraction=latches[i][0]/max(latches[i][1],1),
        near30_to_landed_s=landed[i]-near[i] if i in landed and i in near else None) for i in range(10)}
    return dict(run=run.name,scope='Route TERRAIN_FOLLOW, goal distance >30m, before formation takeover; speed sampled ~1Hz',
        members=members,takeover=takeover,network_inferences_after_takeover=network_after,
        landed_members=sorted(i for i,s in last.items() if s['phase']=='LANDED' and s.get('contact') and not s.get('armed',True)),
        sum_route_variation=sum(x['speed_total_variation_mps'] for x in members.values()),
        sum_route_slow_entries=sum(x['slow_entries'] for x in members.values()),
        sum_route_slow_seconds=sum(x['slow_seconds'] for x in members.values()),
        mean_route_speed=sum(x['estimated_travel_m'] for x in members.values())/max(sum(x['duration_s'] for x in members.values()),1e-9))


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--run',required=True);ap.add_argument('--baseline',required=True)
    args=ap.parse_args();current=measure(args.run);baseline=measure(args.baseline)
    errors=[]
    if current['landed_members']!=list(range(10)):errors.append('Not all ten actually landed')
    if set(current['takeover'])!=set(range(10)):errors.append('Missing all-ten formation takeover')
    if any(x!=0 for x in current['network_inferences_after_takeover'].values()):errors.append('YOPO inference continued after takeover')
    if current['sum_route_variation']>=baseline['sum_route_variation']*.8:errors.append('Route speed variation not improved by 20 percent')
    if current['sum_route_slow_entries']>=baseline['sum_route_slow_entries']*.8:errors.append('Route slow entries not improved by 20 percent')
    if current['mean_route_speed']<baseline['mean_route_speed']*.9:errors.append('Smoothness achieved with more than 10 percent mean-speed loss')
    value=dict(passed=not errors,errors=errors,current=current,baseline=baseline)
    (Path(args.run)/'smooth-audit.json').write_text(json.dumps(value,indent=2)+'\n')
    print(json.dumps(value,indent=2));raise SystemExit(0 if value['passed'] else 1)
