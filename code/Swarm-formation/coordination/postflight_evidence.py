import json
from pathlib import Path
import numpy as np
def write_failed(base,report,reason):
 report['passed']=False;report['closed_loop_passed']=False;report.setdefault('errors',[]).append(reason)
 (Path(base)/'acceptance-report.json').write_text(json.dumps(report,indent=2,allow_nan=True))
def read_verified(base,stats):
 base=Path(base)
 if stats['error'] or stats['rejected'] or stats['pending'] or stats['writer_alive'] or stats['submitted']!=stats['written']:raise ValueError('Evidence writer incomplete: '+str(stats))
 history={0:[],1:[]};decisions=[];faults=[];publication_times=[];lines=0
 for number,line in enumerate((base/'telemetry.jsonl').open(),1):
  try:e=json.loads(line)
  except ValueError as exc:raise ValueError('Corrupt telemetry line%d: %s'%(number,exc))
  lines+=1
  if e['event']=='mission_status':history[e['drone_id']].append(e['state'])
  elif e['event']=='central_decision':
   d=e['decision'];d['allow']={int(k):v for k,v in d['allow'].items()};decisions.append(d)
   if 'publish_monotonic' in e:publication_times.append(e['publish_monotonic'])
  elif e['event']=='controller_notice' and e['state'].get('active'):faults.append(e)
 if lines!=stats['written_by_kind']['telemetry']:raise ValueError('Telemetry row count mismatch')
 odom={}
 for i in (0,1):
  try:a=np.loadtxt(base/('trajectory-drone-%d.csv'%i),delimiter=',',skiprows=1).reshape(-1,7)
  except ValueError as exc:raise ValueError('Corrupt odometry%d: %s'%(i,exc))
  if len(a)!=stats['written_by_kind']['csv%d'%i] or not np.isfinite(a).all() or len(a)>1 and np.any(np.diff(a[:,0])<0):raise ValueError('Invalid odometry rows%d'%i)
  odom[i]=a.tolist()
 if not all(history[i] for i in (0,1)):raise ValueError('Missing mission status required for postprocessing')
 if not all(len(odom[i])>=2 for i in (0,1)):raise ValueError('Insufficient odometry required for postprocessing')
 if min(odom[i][-1][0] for i in (0,1))-max(odom[i][0][0] for i in (0,1))<.02:raise ValueError('No common odometry time interval')
 return history,decisions,faults,publication_times,odom
