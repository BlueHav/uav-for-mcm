#!/usr/bin/env python3
"""Summarize recorded measurements without changing or commanding a flight."""
import json,sys
from pathlib import Path
import numpy as np
run=Path(sys.argv[1]);report=json.loads((run/'report.json').read_text())
rows=[]
for line in (run/'telemetry.jsonl').open():
 try:rows.append(json.loads(line))
 except ValueError:pass
low=[r for r in rows if r['status']['stage']=='TERRAIN']
summary=dict(run_id=report['run_id'],scope=report.get('scope'),passed=report['passed'],errors=report['errors'],route=report.get('route'),stages=report['stages'])
if low:
 agl=np.array([[r['states'][str(i)]['agl'] for i in range(10)] for r in low])
 pos=np.array([[r['states'][str(i)]['position'] for i in range(10)] for r in low])
 inference=np.array([[r['states'][str(i)]['network_inferences'] for i in range(10)] for r in low])
 speed=np.array([[r['states'][str(i)]['speed'] for i in range(10)] for r in low])
 start,end=pos[0,:,:2],pos[-1,:,:2];direction=end-start
 cross=pos[:,:,:2]-start[None,:,:]
 lateral=np.abs(cross[:,:,0]*direction[None,:,1]-cross[:,:,1]*direction[None,:,0])/np.maximum(np.linalg.norm(direction,axis=1),1e-6)
 reasons=[s.get('reason','') for r in low for s in r['states'].values()]
 summary['terrain']=dict(seconds=low[-1]['status']['stamp']-low[0]['status']['stamp'],agl_min=float(agl.min()),agl_max=float(agl.max()),agl_abs_error_p95=float(np.percentile(abs(agl-2),95)),min_body_distance=min(r['status']['min_body_distance'] for r in low),max_speed=float(speed.max()),distance_per_member=np.linalg.norm(np.diff(pos,axis=0),axis=2).sum(0).tolist(),max_lateral_deviation=lateral.max(0).tolist(),yopo_inferences=(inference[-1]-inference[0]).tolist(),astar_reason_samples=sum('A*' in r or 'detour' in r for r in reasons),local_corridor_wait_samples=sum('blocked' in r or 'corridor' in r and ('unknown' in r or 'unobserved' in r) for r in reasons))
 timings=[s.get('plan_seconds',0.) for r in low for s in r['states'].values()]
 summary['terrain']['plan_seconds_max']=max(timings)
 summary['terrain']['plan_seconds_p95']=float(np.percentile(timings,95))
if rows:
 summary['final']={i:{k:s.get(k) for k in ('phase','position','agl','speed','armed','contact','failure')} for i,s in rows[-1]['states'].items()}
(run/'terrain-summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(summary,indent=2))
