#!/usr/bin/env python3
import portable_paths
import json,hashlib,zipfile
from pathlib import Path
import numpy as np
ROOT=Path((str(portable_paths.SCENE)));out=[]
for directory in sorted((ROOT/'Reports/ten-runs').iterdir()):
 if not (directory/'module-flight-result.json').exists():continue
 result=json.loads((directory/'module-flight-result.json').read_text());rows=[json.loads(x) for x in (directory/'telemetry.jsonl').read_text().splitlines()]
 valid=[r for r in rows if len(r['states'])==10];band=[r['status']['center_distance'] for r in valid if r['status']['band_active']]
 lat=[s['plan_seconds'] for r in valid for s in r['states'].values()];ages=[max(s['frame_ages'].values()) for r in valid for s in r['states'].values() if s.get('frame_ages')]
 overlap={}
 for stage in ['CLIMB','DESCEND']:
  frames=[r for r in valid if r['status']['stage']==stage];count=0
  for a,b in zip(frames,frames[1:]):
   p=np.array([a['states'][str(i)]['position'] for i in range(10)]).reshape(2,5,3);q=np.array([b['states'][str(i)]['position'] for i in range(10)]).reshape(2,5,3)
   height_change=abs(q[:,:,2].mean()-p[:,:,2].mean());relp=p-p.mean(axis=1,keepdims=True);relq=q-q.mean(axis=1,keepdims=True)
   if height_change>.002 and np.linalg.norm((relq-relp)[:,:,:2])>.002:count+=1
  overlap[stage]=count
 last=valid[-1];summary=dict(run_id=directory.name,scope=result['scope'],passed=result['passed'],elapsed_s=result['elapsed_s'],center_band_min=min(band) if band else None,center_band_max=max(band) if band else None,
 min_body_distance=min(r['status']['min_body_distance'] for r in valid),planner_latency_p50_p95_max=np.percentile(lat,[50,95,100]).tolist(),sensor_age_p50_p95_max=np.percentile(ages,[50,95,100]).tolist(),
 simultaneous_vertical_shape_samples=overlap,landed_ids=[i for i,s in last['states'].items() if s['phase']=='LANDED' and s['contact'] and not s['armed']],max_local_map_bytes=max(s['map']['bytes'] for r in valid for s in r['states'].values()),
 flight_network_inferences={i:s['network_inferences'] for i,s in last['states'].items()},final=last['status'])
 (directory/'acceptance-summary.json').write_text(json.dumps(summary,indent=2));out.append(summary)
print(json.dumps(out,indent=2))
(ROOT/'Reports/ten-acceptance-summaries.json').write_text(json.dumps(out,indent=2))
