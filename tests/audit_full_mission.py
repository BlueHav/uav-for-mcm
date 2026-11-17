#!/usr/bin/env python3
"""SSH-server-only audit of the complete production ten-aircraft mission."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from point_mission_core import PointMission
from cloud_config import configure_model,CRUISE_MINIMUM,CRUISE_SECONDS


def ordered(actual,required):
    remaining=iter(actual)
    return all(any(value==target for value in remaining) for target in required)


def full_coverage_problems(report):
    errors=[]
    if sorted(report.get('active_ids',[]))!=list(range(10)):errors.append('Full mission must include all ten active members')
    stages=[x.get('stage') for x in report.get('stages',[])]
    if not ordered(stages,['TAKEOFF','AIR_ASSEMBLE','ASSEMBLED','RUNNING','PRELAND','LANDING','LANDED']):
        errors.append('Incomplete or unordered stage-one-to-landing history')
    groups=report.get('group_events',[])
    for g in (0,1):
        phases=[x.get('phase') for x in groups[g]] if len(groups)>g else []
        if not ordered(phases,['WAIT','CLIMB','CRUISE','MOVING_MERGE','DESCENT_ASSEMBLED','TERRAIN','FINAL']):
            errors.append('Incomplete or unordered full group history: '+str(g))
    cruise=report.get('cruise_evidence',{})
    if (not cruise.get('passed') or sorted(cruise.get('checked_members',[]))!=list(range(10))
            or cruise.get('minimum_horizontal_mps',0)<CRUISE_MINIMUM
            or cruise.get('required_straight_seconds',0)<CRUISE_SECONDS):
        errors.append('Missing full all-ten measured cruise evidence')
    for i in range(10):
        member=cruise.get('members',{}).get(str(i),cruise.get('members',{}).get(i,{}))
        if not member.get('passed') or member.get('best_straight_seconds',0)<CRUISE_SECONDS:
            errors.append('Missing qualified cruise member: '+str(i))
    return errors


def audit(run,point):
    run=Path(run);point=Path(point).resolve();root=point.parent.parent
    paths=list((run/'Reports/ten-runs').glob('*/report.json'))
    if len(paths)!=1:raise ValueError('Expected one production full-mission report in this run')
    report_path=paths[0];report=json.loads(report_path.read_text())
    result=json.loads((run/'result.json').read_text());model=PointMission(point);configure_model(model)
    errors=full_coverage_problems(report);last={};rows=0;landing={};own=set();modes=[]
    first_spawn=None;terminal_first={};terminal_peak={};network_first={};network_peak={};global_swarm={}
    policy_members=set();policy_floor_violations=0;link_peaks={};faults=set()
    with report_path.with_name('telemetry.jsonl').open() as stream:
        for line in stream:
            row=json.loads(line);rows+=1;states={int(i):s for i,s in row['states'].items()};last.update(states)
            status=row['status'];mode=status.get('planner_mode')
            if not modes or modes[-1]!=mode:modes.append(mode)
            if first_spawn is None and status['stage']=='TAKEOFF' and set(states)==set(range(10)):
                first_spawn=float(np.max(np.linalg.norm(np.array([states[i]['position'][:2] for i in range(10)])-model.spawn()[:,:2],axis=1)))
            for i,s in states.items():
                global_swarm[i]=max(global_swarm.get(i,0),s.get('swarm_calls',0))
                if s.get('failure'):faults.add((i,s['failure']))
                if mode=='YOPO' and s.get('phase') in ('TERRAIN_FOLLOW','END_HOLD'):
                    calls=s.get('swarm_calls',0);inference=s.get('network_inferences',0)
                    terminal_first.setdefault(i,calls);terminal_peak[i]=max(terminal_peak.get(i,0),calls)
                    network_first.setdefault(i,inference);network_peak[i]=max(network_peak.get(i,0),inference)
                    if s.get('ground_cloud_source')=='own':own.add(i)
                    direction=s.get('formation',{}).get('direction_limit') or {}
                    if 'policy_cap_mps' in direction:
                        policy_members.add(i);cap=float(direction['policy_cap_mps'])
                        if not direction.get('hard_pause',False) and (not np.isfinite(cap) or not 2.-1e-8<=cap<=6.+1e-8):
                            policy_floor_violations+=1
            for g in (0,1):
                if g in landing or not any(s.get('phase')=='LAND' for i,s in states.items() if i//5==g):continue
                if all(i in last for i in range(5*g,5*g+5)):
                    xy=np.array([last[i]['position'][:2] for i in range(5*g,5*g+5)])
                    landing[g]=float(np.max(np.linalg.norm(xy-model.landing_slots(g)[:,:2],axis=1)))
            for g,c in enumerate(status.get('terminal_cohesion',[])):
                if c:
                    for edge in c['edges']:
                        key='%d:%d-%d'%(g,edge['parent'],edge['child'])
                        link_peaks[key]=max(link_peaks.get(key,0.),edge['peak_distance_m'])
    if not result.get('passed') or not report.get('passed'):errors.append('Full production flight did not pass')
    if report.get('errors') or faults:errors.append('Flight fault recorded')
    if report.get('terminal_mode')!='independent_yopo':errors.append('Terminal planner was not independent YOPO')
    if first_spawn is None or first_spawn>=.5:errors.append('Initial measured ten-aircraft spawn not verified')
    if not ordered(modes,['CLOUD','YOPO_WARMUP','YOPO']):errors.append('Complete CLOUD to independent YOPO ownership not observed')
    if set(landing)!={0,1} or any(x>=.5 for x in landing.values()):errors.append('Leading star not settled before vertical landing')
    if not model.landed(last,tuple(range(10))):errors.append('All ten measured star touchdown/contact/disarm states not accepted')
    network_delta={i:network_peak[i]-network_first[i] for i in network_peak}
    swarm_delta={i:terminal_peak[i]-terminal_first[i] for i in terminal_peak}
    if set(network_delta)!=set(range(10)) or any(x<=0 for x in network_delta.values()):errors.append('Missing actual terminal inference for a member')
    if set(swarm_delta)!=set(range(10)) or any(x!=0 for x in swarm_delta.values()):errors.append('Joint Swarm planner called during independent terminal flight')
    if own!=set(range(10)):errors.append('Missing per-aircraft own terrain evidence')
    if policy_members!=set(range(10)) or policy_floor_violations:errors.append('Independent formation speed-floor evidence invalid')
    for key in ('collision_events','bounds_events'):
        if report.get(key)!=0:errors.append('Unity '+key+' not zero')
    manifest_path=run/'source-manifest.json';manifest=json.loads(manifest_path.read_text());source_mismatch=[]
    for name,digest in manifest.items():
        p=root/name
        if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=digest:source_mismatch.append(name)
    if source_mismatch:errors.append('Flight inputs changed since the full run started')
    contract_path=run/'run-contract.json';contract=json.loads(contract_path.read_text()) if contract_path.exists() else None
    if contract and any(contract.get(k) is not False for k in ('phase_skipping','spawn_override','lag_injection')):
        errors.append('Full run contained fixture stage/spawn/injection overrides')
    value=dict(passed=not errors,scope='complete production ten-aircraft mission from stage one',errors=errors,
        run_id=report['run_id'],run_name=run.name,telemetry_rows=rows,
        stages=report['stages'],group_events=report['group_events'],planning_modes=modes,
        initial_spawn_max_xy_error_m=first_spawn,cruise_evidence=report.get('cruise_evidence'),
        terminal_network_inference_delta=network_delta,terminal_joint_swarm_call_delta=swarm_delta,
        whole_mission_swarm_calls=global_swarm,own_crop_members=sorted(own),
        prelanding_star_max_xy_error_m=landing,observed_members=sorted(last),
        landed_members=[i for i in sorted(last) if model.landed(last,(i,))],
        collision_events=report.get('collision_events'),bounds_events=report.get('bounds_events'),
        minimum_body_distance_m=report.get('min_body_distance'),maximum_altitude_m=report.get('max_altitude'),
        terminal_actual_peak_horizontal_mps=report.get('terminal_peak_horizontal_mps'),
        relay_handover_boundaries={i:s['relay']['boundary'] for i,s in last.items() if (s.get('relay') or {}).get('boundary')},
        formation_policy_floor_violations=policy_floor_violations,cohesion_link_peaks_m=link_peaks,
        distance_50m_is_completion_requirement=False,faults=sorted(faults),run_contract=contract,
        flight_elapsed_s=result.get('status',{}).get('flight_elapsed_s'),
        source_manifest_sha256=hashlib.sha256(manifest_path.read_bytes()).hexdigest(),source_mismatches=source_mismatch)
    return value


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--run',type=Path,required=True)
    ap.add_argument('--point',type=Path,default=Path(__file__).resolve().parents[1]/'config/point.json')
    args=ap.parse_args()
    try:value=audit(args.run,args.point)
    except Exception as exc:value=dict(passed=False,scope='full production mission audit',errors=[str(exc)])
    (args.run/'full-audit.json').write_text(json.dumps(value,indent=2)+'\n')
    print(json.dumps(value,indent=2));raise SystemExit(0 if value['passed'] else 1)
