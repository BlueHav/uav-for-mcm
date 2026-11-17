#!/usr/bin/env python3
"""Server-only audit of actual ten-aircraft phase-six telemetry."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from point_mission_core import PointMission

def recorded_release_consistent(was_active,previous_count,edge,previous_triggers=0):
    count=edge['releases'];triggers=edge['triggers']
    return (type(count) is int and type(triggers) is int and
            0<=previous_count<=count<=triggers and previous_triggers<=triggers and
            triggers-count==int(edge['active']) and
            (not was_active or edge['active'] or count>previous_count))

def normal_direction_permission(state,stamp,nominal=6.):
    formation=state.get('formation',{})
    if 'direction_limit' not in formation:
        return (formation.get('member_speed_cap') or 0.)>=nominal-1e-6
    limit=formation.get('direction_limit') or {}
    try:
        return (not limit.get('limited',True) and -.1<=stamp-float(limit.get('stamp',0))<=.6
                and float(limit.get('cap_mps',0))>=nominal-1e-6)
    except (TypeError,ValueError):return False


def observed_order_consumption(state,stamp):
    """Consumer permission and physical response, independent of producer caps."""
    limit=state.get('formation',{}).get('direction_limit') or {}
    try:
        cap=float(limit.get('policy_cap_mps',float('nan')))
        command=np.asarray(limit.get('limited_xy',[]),float)
        consumed=(limit.get('observation_fresh') is True and not limit.get('hard_pause',False)
            and -.1<=stamp-float(limit['stamp'])<=.6
            and 2.-1e-8<=cap<=2.+1e-8 and command.shape==(2,) and np.isfinite(command).all()
            and np.linalg.norm(command)<=2.+1e-6)
        slowed=consumed and np.linalg.norm(state['velocity'][:2])<=2.5
        return bool(consumed),bool(slowed)
    except (KeyError,TypeError,ValueError):return False,False

def distance_audit_errors(groups,events,measured_peaks,gate_peaks,require_cycle=False):
    errors=[]
    if set(groups)!={0,1}:errors.append('Missing both group distance-controller observations')
    if measured_peaks and max(measured_peaks.values())>50.:errors.append('Measured XY link exceeded 50 m')
    if gate_peaks and max(gate_peaks.values())>50.:errors.append('20 Hz controller XY link peak exceeded 50 m')
    if require_cycle and (not any(x['active'] for x in events) or not any(not x['active'] for x in events)):
        errors.append('Distance trigger/release was not exercised')
    return errors


def observed_injection_front_slow(state,stamp,start,end):
    limit=state.get('formation',{}).get('direction_limit') or {}
    try:
        return bool(start is not None and end is not None and start<=stamp<=end+10.
            and limit.get('observation_fresh') is True and not limit.get('hard_pause',False)
            and -.1<=stamp-float(limit['stamp'])<=.6
            and 2.-1e-8<=float(limit['policy_cap_mps'])<=2.100001
            and np.linalg.norm(limit['limited_xy'])<=2.100001
            and np.linalg.norm(state['velocity'][:2])<2.5)
    except (KeyError,TypeError,ValueError):return False


def audit(run,point,require_cohesion=False,require_lag=False,require_formation=False,require_distance_cycle=False):
    report=json.loads((run/'Reports/mission/report.json').read_text())
    result=json.loads((run/'result.json').read_text()) if (run/'result.json').exists() else {}
    model=PointMission(point)
    manifest_path=run/'source-manifest.json'
    manifest=json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
    errors=[];last={};inferences={i:0 for i in range(10)};peaks={i:0. for i in range(10)}
    landing={};own=set();rows=0;swarm_calls=0;failures=set()
    cohesion={};events=[];previous={};release_counts={};trigger_counts={};measured_peaks={};gate_peaks={}
    lag=dict(injected_slow=False,front_slow=False,rear_catchup=False,recovered_within_25m=False,resumed=False)
    shape=dict(groups=[],coefficient_peaks={},culprit_observations={},order_violation_observations={},
        order_response={},policy_floor_violations=0,consumer_policy_floor_violations=0,order_cap_violations=0,
        stale_fallback_observations=0,zero_command_reasons={})
    shape_groups=set();consumer_policy_seen=set()
    injected=[];lag_start=lag_end=release_stamp=None
    if (run/'lag-injection.jsonl').exists():
        injected=[json.loads(line) for line in (run/'lag-injection.jsonl').read_text().splitlines()]
        lag_start=next((x['stamp'] for x in injected if x['active']),None)
        lag_end=next((x['stamp'] for x in injected if not x['active'] and lag_start is not None and x['stamp']>lag_start),None)
    with (run/'Reports/mission/telemetry.jsonl').open() as stream:
        for line in stream:
            row=json.loads(line);rows+=1
            states={int(i):s for i,s in row['states'].items()};last.update(states)
            stamp=row['status']['stamp']
            target_active=None
            for g,c in enumerate(row['status'].get('terminal_cohesion',[])):
                if c is None:continue
                cohesion[g]=c
                formation=c.get('formation')
                if formation:
                    shape_groups.add(g)
                    cost=float(formation['graph_cost'])
                    shape['coefficient_peaks'][g]=max(shape['coefficient_peaks'].get(g,0.),cost)
                    caps=np.asarray(formation['speed_caps'],float)
                    if (caps.shape!=(5,) or not np.isfinite(caps).all() or np.any(caps<2.-1e-8) or np.any(caps>6.+1e-8)):
                        shape['policy_floor_violations']+=1
                    if not np.isfinite(cost) or abs(sum(formation['row_contributions'])-cost)>1e-7:
                        errors.append('Invalid formation coefficient decomposition')
                    for member in formation['culprit_members']:
                        key=str(5*g+member)
                        shape['culprit_observations'][key]=shape['culprit_observations'].get(key,0)+1
                    for order in formation['order_edges']:
                        key='%d:%d-%d'%(g,order['parent'],order['child'])
                        if order['actual_violation']:
                            shape['order_violation_observations'][key]=shape['order_violation_observations'].get(key,0)+1
                            if caps[order['child']]>2.+1e-8:shape['order_cap_violations']+=1
                            response=shape['order_response'].setdefault(key,dict(first_stamp=stamp,
                                consumer_cap_seen=False,physical_slow_seen=False,recovered=False,violation_rows=0))
                            response['violation_rows']+=1;response['last_violation_stamp']=stamp;response['recovered']=False
                            child=states.get(5*g+order['child'],{})
                            consumed,slowed=observed_order_consumption(child,stamp)
                            response['consumer_cap_seen']|=consumed;response['physical_slow_seen']|=slowed
                        elif key in shape['order_response']:
                            response=shape['order_response'][key]
                            response['recovered']=True;response['recovery_stamp']=stamp
                for e in c['edges']:
                    key=(g,e['parent'],e['child']);label='%d:%d-%d'%key
                    if key==(0,0,3):target_active=e['active']
                    gate_peaks[label]=max(gate_peaks.get(label,0.),e['peak_distance_m'])
                    a,b=5*g+e['parent'],5*g+e['child']
                    if a in last and b in last:
                        measured=float(np.linalg.norm(np.array(last[a]['position'][:2])-last[b]['position'][:2]))
                        measured_peaks[label]=max(measured_peaks.get(label,0.),measured)
                    before=previous.get(key,False)
                    if not recorded_release_consistent(before,release_counts.get(key,0),e,trigger_counts.get(key,0)):
                        errors.append('Distance latch release counter inconsistent')
                    if before!=e['active']:
                        events.append(dict(group=g,parent=e['parent'],child=e['child'],active=e['active'],
                            stamp=stamp,observation_distance_m=e['distance_m'],front=e['front'],rear=e['rear'],
                            release_count=e['releases']))
                        # The gate runs at 20 Hz; logs retain every second tick.
                        # The next false observation may already be accelerating
                        # away from the actual <=25 m release between samples.
                        # Counters confirm the unit-tested release transition.
                        # The forced-lag case additionally requires an actual
                        # recorded <=25 m observation of its release.
                        if (key==(0,0,3) and before and not e['active'] and lag_end is not None
                                and stamp>=lag_end and e['distance_m']<=c['resume_m']):
                            lag['recovered_within_25m']=True;release_stamp=stamp
                    # Shape regulation can keep this edge below the latch
                    # trigger throughout. Recovery within 25 m is observable
                    # independently of an actual distance-latch cycle.
                    if (key==(0,0,3) and lag_end is not None and stamp>=lag_end
                            and not e['active'] and e['distance_m']<=c['resume_m']):
                        lag['recovered_within_25m']=True
                        if release_stamp is None:release_stamp=stamp
                    previous[key]=e['active']
                    release_counts[key]=e['releases']
                    trigger_counts[key]=e['triggers']
                    if key==(0,0,3) and e['front']==0 and 0 in last and 3 in last:
                        front,rear=last[0],last[3]
                        rc=rear.get('formation',{}).get('member_speed_cap',0.)
                        rc=0. if rc is None else rc
                        fv=float(np.linalg.norm(front['velocity'][:2]));rv=float(np.linalg.norm(rear['velocity'][:2]))
                        if observed_injection_front_slow(front,stamp,lag_start,lag_end):lag['front_slow']=True
                        if (lag_end is not None and stamp>=lag_end and rc>=5.999
                                and normal_direction_permission(rear,stamp) and rv>fv+.2):lag['rear_catchup']=True
            if lag_start is not None and lag_end is not None and 0 in last and 3 in last:
                leaf=last[3];front=last[0]
                if (lag_start+4.<=stamp<lag_end and abs(leaf.get('formation',{}).get('member_speed_cap',6.)-2.)<1e-6
                        and .5<np.linalg.norm(leaf['velocity'][:2])<2.5):
                    lag['injected_slow']=True
                direction=front.get('formation',{}).get('direction_limit') or {}
                normal_command=('direction_limit' not in front.get('formation',{}) or np.linalg.norm(direction.get('limited_xy',[]))>2.2)
                if (release_stamp is not None and stamp>release_stamp and target_active is False
                        and normal_direction_permission(front,stamp) and normal_command
                        and np.linalg.norm(front['velocity'][:2])>2.2):
                    lag['resumed']=True
            for i,s in states.items():
                direction=s.get('formation',{}).get('direction_limit') or {}
                if s.get('phase') in ('TERRAIN_FOLLOW','END_HOLD') and 'policy_cap_mps' in direction:
                    consumer_policy_seen.add(i)
                    policy=float(direction['policy_cap_mps'])
                    if not direction.get('hard_pause',False) and (not np.isfinite(policy) or not 2.-1e-8<=policy<=6.+1e-8):
                        shape['consumer_policy_floor_violations']+=1
                    if not direction.get('observation_fresh',False):shape['stale_fallback_observations']+=1
                    if np.linalg.norm(direction.get('limited_xy',[]))<.02 and np.linalg.norm(s['velocity'][:2])<.2:
                        reason=s.get('reason','')
                        category=('safety_braking' if 'stop' in reason.lower() or 'brak' in reason.lower() else
                            'sensor_turn' if 'Turning sensor' in reason else 'no_observed_corridor' if 'No observed' in reason else
                            'terminal_settle' if s['phase']=='END_HOLD' else 'other_planner_command')
                        shape['zero_command_reasons'][category]=shape['zero_command_reasons'].get(category,0)+1
                inferences[i]=max(inferences[i],s.get('network_inferences',0))
                peaks[i]=max(peaks[i],float(np.linalg.norm(s['velocity'][:2])))
                swarm_calls=max(swarm_calls,s.get('swarm_calls',0))
                if s.get('failure'):failures.add((i,s['failure']))
                if s.get('ground_cloud_source')=='own' and s['phase']=='TERRAIN_FOLLOW':own.add(i)
            for g in (0,1):
                if g in landing or not any(s.get('phase')=='LAND' for i,s in states.items() if i//5==g):continue
                if all(i in last for i in range(5*g,5*g+5)):
                    xy=np.array([last[i]['position'][:2] for i in range(5*g,5*g+5)])
                    landing[g]=float(np.max(np.linalg.norm(xy-model.landing_slots(g)[:,:2],axis=1)))
    if not result.get('passed') or not report.get('passed'):errors.append('Actual flight did not pass')
    if not model.landed(last):errors.append('Ten measured star touchdown/contact/disarm states not accepted')
    if set(landing)!={0,1} or any(d>=.5 for d in landing.values()):errors.append('Star was not settled before vertical landing')
    if any(n<=0 for n in inferences.values()):errors.append('Missing real network inference for one or more aircraft')
    if own!=set(range(10)):errors.append('Missing own terrain crop evidence for one or more aircraft')
    if swarm_calls:errors.append('Joint swarm planner was called')
    if failures:errors.append('Per-aircraft flight fault recorded')
    for key in ('collision_events','bounds_events'):
        if report.get(key)!=0:errors.append('Unity '+key+' not zero')
    if require_cohesion:
        errors.extend(distance_audit_errors(cohesion,events,measured_peaks,gate_peaks,require_distance_cycle))
    if require_lag and (lag_end is None or not all(lag.values())):errors.append('Slow-injection physical slowdown/catchup/25m recovery/resume evidence incomplete')
    shape['groups']=sorted(shape_groups)
    shape['consumer_policy_members']=sorted(consumer_policy_seen)
    if require_formation:
        if shape_groups!={0,1}:errors.append('Missing formation coefficient evidence for both groups')
        if shape['policy_floor_violations']:errors.append('Formation policy cap outside 2..6 m/s')
        if consumer_policy_seen!=set(range(10)):errors.append('Missing individual policy cap observations')
        if shape['consumer_policy_floor_violations']:errors.append('Consumed formation policy cap outside 2..6 m/s')
        if shape['order_cap_violations']:errors.append('Overtaking child not assigned minimum speed')
        for edge,response in shape['order_response'].items():
            if not response['recovered']:errors.append('Overtaking order never restored: '+edge)
            # Brief turn-dependent projection changes may disappear before a
            # full braking response is possible. Persistent violations must
            # show consumer permission AND measured physical slowdown.
            if response['violation_rows']>=10 and not (response['consumer_cap_seen'] and response['physical_slow_seen']):
                errors.append('Persistent overtaking lacks consumer/physical slowdown: '+edge)
    value=dict(passed=not errors,run_id=report.get('run_id'),run_name=run.name,
        scope='focused real phase-six flight; phases 1-5 not exercised',
        errors=errors,telemetry_rows=rows,network_inferences=inferences,joint_swarm_calls=swarm_calls,
        prelanding_star_max_error_m=landing,actual_peak_horizontal_mps=peaks,
        own_crop_members=sorted(own),failures=sorted(failures),
        collision_events=report.get('collision_events'),bounds_events=report.get('bounds_events'),
        source_hashes=report.get('hashes'),cohesion_measured_xy_peaks_m=measured_peaks,cohesion_gate_xy_peaks_m=gate_peaks,
        runtime_source_manifest_sha256=hashlib.sha256(manifest_path.read_bytes()).hexdigest() if manifest else None,
        player_assembly_sha256=manifest.get('unity/Build/MountainCity_Data/Managed/Assembly-CSharp.dll'),
        cohesion_events=events,lag_evidence=lag if injected else None,formation_speed_evidence=shape)
    value['distance_cycle_exercised']=bool(any(x['active'] for x in events) and any(not x['active'] for x in events))
    (run/'audit.json').write_text(json.dumps(value,indent=2)+'\n')
    return value

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--run',type=Path,required=True)
    ap.add_argument('--point',type=Path,default=Path(__file__).resolve().parents[1]/'config/point.json')
    ap.add_argument('--require-cohesion',action='store_true');ap.add_argument('--require-lag',action='store_true')
    ap.add_argument('--require-formation',action='store_true')
    ap.add_argument('--require-distance-cycle',action='store_true',help='Also require an actual distance latch trigger and release')
    args=ap.parse_args();value=audit(args.run,args.point,args.require_cohesion or args.require_distance_cycle,args.require_lag,args.require_formation,args.require_distance_cycle);print(json.dumps(value,indent=2))
    raise SystemExit(0 if value['passed'] else 1)
