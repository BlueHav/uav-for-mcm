# Server-only analysis of actual finished short-flight evidence; no Runtime imports.
import json,sys
from pathlib import Path
import numpy as np
B=Path(sys.argv[1]);report=json.loads((B/'acceptance-report.json').read_text());events=[json.loads(l) for l in (B/'telemetry.jsonl').read_text().splitlines()];lo=report['crossing_started'];hi=report['crossing_ended'];dec=[e['decision'] for e in events if e['event']=='central_decision' and lo<=e['decision']['stamp']<=hi];a=np.loadtxt(B/'trajectory-drone-1.csv',delimiter=',',skiprows=1);speed=np.linalg.norm(a[:,4:7],axis=1);gates=[json.loads(l) for l in (B/'pair-audit-1.jsonl').read_text().splitlines()];gates=[e for e in gates if e['event']=='central_gate_actual_control'];cycles=[]
for before,d in zip(dec,dec[1:]):
 if before.get('yielding') or not d.get('yielding'):continue
 start=d['stamp'];resume=next((e for e in dec if e['stamp']>start and not e.get('yielding')),None)
 if resume is None:continue
 end=resume['stamp'];indices=np.flatnonzero((a[:,0]>=start)&(a[:,0]<=end));stopidx=next((int(i) for i in indices if speed[i]<.15),None);gate=next((e for e in gates if e['stamp']>=start and not e['allowed']),None)
 x=dict(trigger_stamp=start,trigger_seq=d['seq'],resume_stamp=end,hold_seconds=end-start,trigger_actual_speed=float(np.linalg.norm(d['velocities'][1])),logged_stop_bound_m=d['stop_bounds'][1],predicted_closest_m=d['predicted_closest'],near_trigger_m=d['near_trigger'],actual_initial_separation_m=d['distance'],winner_position_at_release=resume['positions'][0],winner_distance_from_real_crossing_centre_at_release_m=float(np.linalg.norm(np.array(resume['positions'][0])-np.array(resume['conflict']))),actual_gate_latency_s=None if gate is None else gate['stamp']-start)
 if stopidx is not None:
  p=np.array([np.interp(start,a[:,0],a[:,j]) for j in range(1,4)]);x.update(actual_stop_latency_s=float(a[stopidx,0]-start),actual_stop_displacement_m=float(np.linalg.norm(a[stopidx,1:4]-p)),actual_stop_arc_length_m=float(np.linalg.norm(np.diff(np.vstack([p,a[indices[indices<=stopidx],1:4]]),axis=0),axis=1).sum()),actual_stop_speed_mps=float(speed[stopidx]),actual_stop_position=a[stopidx,1:4].tolist())
  later=next((int(i) for i in np.flatnonzero((a[:,0]>end)&(a[:,0]<end+12)) if speed[i]>.25),None);x['actual_resumed_motion_stamp']=None if later is None else float(a[later,0]);x['stop_inside_logged_bound']=bool(x['actual_stop_arc_length_m']<=x['logged_stop_bound_m'])
 cycles.append(x)
allgates=[]
for i in (0,1):
 for line in (B/('pair-audit-%d.jsonl'%i)).read_text().splitlines():
  e=json.loads(line)
  if e['event']=='central_gate_actual_control' and lo<=e['stamp']<=hi:allgates.append(e)
r=dict(passed=bool(report['passed'] and cycles and all(c.get('stop_inside_logged_bound') and c.get('actual_resumed_motion_stamp') for c in cycles)),scope='Actual crossing phase only, current2-aircraft low-speed flight; not full-route acceptance',crossing_started=lo,crossing_ended=hi,cycles=cycles,max_crossing_actual_lease_age_s=max(e['lease_age'] for e in allgates),known_limitations=['Conservative stopping guard caused repeated stop/release cycles while winner moved clear; safe progress was observed, smoothness was not established.'],final_both_landed_disarmed=all(not s['armed'] and s['contact'] and s['phase']=='LANDED' for s in report['final_states'].values()))
(B/'crossing-braking-evidence.json').write_text(json.dumps(r,indent=2));print(json.dumps(dict(passed=r['passed'],cycles=len(cycles),first_cycle=cycles[0] if cycles else None,max_lease_age=r['max_crossing_actual_lease_age_s'],final_both_landed_disarmed=r['final_both_landed_disarmed']),indent=2))
