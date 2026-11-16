"""Summarize actual runs without treating failed/old landed tests as acceptance."""
import portable_paths
import json
from pathlib import Path
import numpy as np
root=Path((str(portable_paths.SCENE)))
out=[]
for directory in sorted((root/'Reports/ten-runs').iterdir()):
 result=directory/'continuous-result.json'
 if not result.exists():continue
 result=json.loads(result.read_text());rows=[json.loads(line) for line in (directory/'telemetry.jsonl').read_text().splitlines()]
 rows=[r for r in rows if len(r['states'])==10]
 close=[r['status']['center_distance'] for r in rows if r['status']['stage'] in ('ASSEMBLE','REFORM')]
 band=[r['status']['center_distance'] for r in rows if r['status']['band_active']]
 lat=[s['plan_seconds'] for r in rows for s in r['states'].values()]
 summary=dict(run_id=directory.name,passed=result['passed'],scope=result['scope'],elapsed_s=result['elapsed'],
  stages=json.loads((directory/'report.json').read_text())['stages'],assembly_center_max=max(close) if close else None,
  cruise_center_range=[min(band),max(band)] if band else [],min_body_distance=min(r['status']['min_body_distance'] for r in rows),
  final_agl=[s['agl'] for s in result['members'].values()],final_armed=[s['armed'] for s in result['members'].values()],
  planner_latency_p50_p95_max=np.percentile(lat,[50,95,100]).tolist(),
  flight_network_inferences={i:s['network_inferences'] for i,s in result['members'].items()},
  collision_events=result['scene'].get('collision_events'),bounds_events=result['scene'].get('bounds_events'))
 (directory/'continuous-summary.json').write_text(json.dumps(summary,indent=2));out.append(summary)
(root/'Reports/continuous/summaries.json').write_text(json.dumps(out,indent=2))
print(json.dumps(out,indent=2))
