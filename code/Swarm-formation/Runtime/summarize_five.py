#!/usr/bin/env python3
"""Summarize immutable runner result; do not turn failures into acceptance."""
import portable_paths
import json,hashlib,zipfile
from pathlib import Path
import numpy as np
root=Path((str(portable_paths.SCENE)));reports=root/'Reports'
run=Path((root/'Runtime/five-last-run').read_text());result=json.loads((run/'runner-result.json').read_text())
if not result['passed']:raise SystemExit('Cannot summarize a failed run as accepted')
rows=[json.loads(s) for s in (run/'telemetry.jsonl').read_text().splitlines()]
end=next(i for i,r in enumerate(rows) if r['group']['stage']=='COMPLETE');rows=rows[:end+1]
cruise=[r for r in rows if r['group']['stage']=='CRUISE']
shape_errors=[]
for r in cruise:
    actual=np.array([r['states'][str(i)]['position'] for i in range(5)]);target=np.array(r['targets'])
    shape_errors.append(float(np.max(np.linalg.norm((actual-actual[0])-(target-target[0]),axis=1))))
summary=dict(run=str(run),passed=True,duration_seconds=result['report']['stages'][-1]['stamp']-result['report']['stages'][0]['stamp'],
    min_separation=result['report']['min_separation'],final_slot_error=result['report']['final_slot_error'],
    collision_events=result['report']['collision_events'],cruise_distance=result['report']['cruise_distance'],
    cruise_speed_p5_p50_p95=np.percentile([s['speed'] for r in cruise for s in r['states'].values()],[5,50,95]).tolist(),
    cruise_relative_shape_error_max=max(shape_errors),
    plan_seconds_p50_p95_max=np.percentile([s['plan_seconds'] for r in rows for s in r['states'].values()],[50,95,100]).tolist(),
    network_inferences=result['report']['network_inferences'],map_bytes=result['report']['map_bytes'])
(run/'acceptance-summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
sources=list((root/'Runtime').glob('*.py'))+list((root/'Runtime').glob('*.launch'))+list((root/'Runtime').glob('*.rviz'))
sources+=[root/'Assets/TrackerField.cs',root/'Assets/GoalOverlay.cs',root/'install_five.py']
sources+=[Path((str(portable_paths.OPS) + '/bin'))/n for n in ('start.sh','start_five.sh','run_five.sh','check_five.sh','stop.sh','status.sh','show.sh')]
manifest={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
manifest[str(root/'Assets/Resources/TrackerWorld.json')]=hashlib.sha256((root/'Assets/Resources/TrackerWorld.json').read_bytes()).hexdigest()
(reports/'five-source-manifest.json').write_text(json.dumps(manifest,indent=2))
with zipfile.ZipFile(reports/'five-source-snapshot.zip','w',zipfile.ZIP_DEFLATED) as z:
    for p in sources:z.write(p,str(p).lstrip('/'))
with zipfile.ZipFile(reports/'five-evidence.zip','w',zipfile.ZIP_DEFLATED) as z:
    for directory in (reports/'five-runs').iterdir():
        for p in directory.glob('*.json'):z.write(p,'five-runs/'+directory.name+'/'+p.name)
    for name in ('three-mode-regression.log','five-unit-tests.log','single-regression-after-five.json','two-regression-after-five.json','five-source-manifest.json','five-unity-complete.png','five-rviz-complete.png','five-final-ready.json','rebuild.log'):
        p=reports/name
        if p.exists():z.write(p,name)
