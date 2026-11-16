import csv,json,math,os,re,threading,time,traceback,subprocess,collections,importlib.util
from evidence_writer import EvidenceWriter
from producer_gate import ProducerGate
from postflight_evidence import read_verified,write_failed
from pathlib import Path
import numpy as np,rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from shared_policy import Policy,CFG
from coordination_config import validate_config
from parallel_descent import GroupScheduler
B=Path(__file__).parent;manifest=json.loads((B/'test-manifest.json').read_text());mode=os.environ.get('SHARED_TEST_MODE','short');rospy.init_node('direct_global_pair_coordinator',disable_signals=True)
lock=threading.RLock();states={};global_states={};scene={};goals={};history={i:collections.deque(maxlen=32) for i in (0,1)};odom={i:collections.deque(maxlen=128) for i in (0,1)};decisions=collections.deque(maxlen=64);last_decision={};faults=[];stage='ready';closing=False;forced_fault=None
configuration=validate_config(manifest['coordination_config'],deployment=True)
group_scheduler=GroupScheduler(configuration);parallel_enabled=False;member_stages={i:'CRUISE' for i in configuration['vehicle_ids']}
writer=EvidenceWriter(B,capacity=512);producer_gate=ProducerGate()
report=dict(passed=False,closed_loop_passed=False,coordination_passed=False,mode=mode,started=time.time(),errors=[],stages=[],thresholds=CFG,architecture='Two group leader proxies with independent group stages and central global-state distance safety. Two independent count1 environment planners; no peer trajectory communication. Future two groups of five are not flight-validated.')
def record(event,**kw):
 if not closing or producer_gate.current_accepted():writer.submit('telemetry',dict(stamp=time.time(),event=event,stage=stage,**kw))

def measured_agl(state,status):
 stamps=[status.get('ground_stamp'),status.get('integrated_ground_stamp'),state['stamp']]
 observed=min(stamps) if all(isinstance(v,(int,float)) and math.isfinite(v) for v in stamps) else float('nan')
 return dict(state,agl=state['p'][2]-status.get('ground',float('nan')),agl_stamp=observed)
def state_cb(m,i):
 s=json.loads(m.data)
 with lock:
  if closing:return
  states[i]=s;history[i].append(s)
 record('mission_status',drone_id=i,state=s)
def odom_cb(m,i):
 p=m.pose.pose.position;v=m.twist.twist.linear;r=[m.header.stamp.to_sec(),p.x,p.y,p.z,v.x,v.y,v.z]
 with lock:
  if closing:return
  global_states[i]=dict(stamp=r[0],p=r[1:4],v=r[4:7],frame=m.header.frame_id);odom[i].append(r)
 writer.submit('csv%d'%i,r)
def scene_cb(m):
 with lock:
  if closing:return
  scene.clear();scene.update(json.loads(m.data))
 record('scene_status',state=dict(scene))
def fault_cb(m,i):
 d=json.loads(m.data)
 if d.get('active'):faults.append(dict(drone_id=i,state=d))
 record('controller_notice',drone_id=i,state=d)
state_cb=producer_gate.wrap(state_cb);odom_cb=producer_gate.wrap(odom_cb);scene_cb=producer_gate.wrap(scene_cb);fault_cb=producer_gate.wrap(fault_cb)
pubs={i:rospy.Publisher('/drone_%d/task'%i,String,queue_size=3) for i in (0,1)};authority=rospy.Publisher('/shared_pair/decision',String,queue_size=1,tcp_nodelay=True)
for i in (0,1):
 rospy.Subscriber('/drone_%d/odom'%i,Odometry,odom_cb,callback_args=i,queue_size=1,tcp_nodelay=True)
 rospy.Subscriber('/drone_%d/mission/status'%i,String,state_cb,callback_args=i,queue_size=30)
 rospy.Subscriber('/drone_%d/controller_fault'%i,String,fault_cb,callback_args=i,queue_size=10)
rospy.Subscriber('/scene/status',String,scene_cb,queue_size=10)
policy=Policy();schedule_gaps=collections.deque(maxlen=2048)
@producer_gate.wrap
def safety_loop():
 global last_decision,forced_fault
 previous=None;deadline=time.monotonic()
 while not closing:
  now=rospy.Time.now().to_sec()
  with lock:
   ss={i:dict(v) for i,v in global_states.items()};gg={i:v.copy() for i,v in goals.items()}
   group_active=parallel_enabled;phases=dict(member_stages);mission={i:dict(v) for i,v in states.items()}
  d=policy.decide(ss,gg,now)
  d.update(speed_cap_3d={i:CFG['command_speed'] for i in (0,1)},cap_generation={i:0 for i in (0,1)})
  if mode=='short' and stage=='short_actual_yield_and_crossing_complete':
   d.update(speed_cap_3d={i:.8 for i in (0,1)},cap_generation={i:1 for i in (0,1)})
  if group_active:
   measured={i:measured_agl(s,mission.get(i,{})) for i,s in ss.items()}
   group=group_scheduler.decide(measured,phases,now,goals=gg)
   d['group_coordination']=group;d['speed_cap_3d']=group['speed_cap_3d'];d['cap_generation']=group['cap_generation']
   for i in (0,1):d['allow'][i]=bool(d['allow'].get(i,False) and group['allow'].get(i,False))
   if group.get('hard_fault'):d.update(hard_fault=group['hard_fault'],reason=group['hard_fault'])
   elif d['reason']=='clear':d['reason']=group.get('soft_reason','independent_group_distance_permissions')
  if writer.error and not forced_fault:forced_fault=writer.error
  if previous is not None and time.monotonic()-previous>CFG['lease'] and not forced_fault:forced_fault='central_deadline_missed'
  if forced_fault:d.update(allow={0:False,1:False},hard_fault=forced_fault,reason=forced_fault)
  authority.publish(String(json.dumps(d)));tick=time.monotonic()
  completed_gap=0 if previous is None else tick-previous
  if completed_gap>CFG['lease'] and not forced_fault:forced_fault='central_publish_deadline_missed'
  with lock:
   if previous is not None:schedule_gaps.append(completed_gap)
   previous=tick;last_decision=d;decisions.append(d)
  record('central_decision',decision=d,publish_monotonic=tick,completed_publish_gap=completed_gap);deadline+=CFG['period']
  after=time.monotonic()
  if after>deadline:deadline=after+CFG['period']
  time.sleep(max(0.,deadline-time.monotonic()))
safety_thread=threading.Thread(target=safety_loop,daemon=True);safety_thread.start()
def current():
 with lock:return {i:dict(s) for i,s in states.items()},dict(scene)
def healthy():
 ss,sc=current()
 for i,s in ss.items():
  if s.get('failure') or s.get('controller_fault'):raise RuntimeError('drone%d failure: %s'%(i,s.get('failure') or s.get('controller_fault')))
  if s.get('armed') and (len(s.get('frame_ages',{}))!=3 or max(s['frame_ages'].values())>.6):raise RuntimeError('drone%d actual sensor stale'%i)
 if writer.error:raise RuntimeError('Reliable evidence failure: '+writer.error)
 if faults:raise RuntimeError('Actual controller watchdog fault')
 if sc.get('collision_events',0) or sc.get('bounds_events',0):raise RuntimeError('Actual Unity collision or bounds event')
 if last_decision.get('hard_fault') and any(s.get('armed') for s in ss.values()):raise RuntimeError('Central safety: '+last_decision['hard_fault'])
 return ss,sc
def wait(label,predicate,timeout):
 global stage
 stage=label;start=time.monotonic();last=0.;deadlock_started=None
 while time.monotonic()-start<timeout:
  ss,sc=healthy()
  blocked=len(ss)==2 and label=='short_actual_yield_and_crossing_complete' and all(s['speed']<.15 for s in ss.values()) and not any(last_decision.get('allow',{}).values())
  if blocked:
   if deadlock_started is None:deadlock_started=time.monotonic();record('possible_static_deadlock',states=ss,decision=last_decision)
   elif time.monotonic()-deadlock_started>10:raise RuntimeError('Static crossing deadlock: both centrally blocked and stopped for 10s')
  else:deadlock_started=None
  if len(ss)==2 and predicate(ss,sc):report['stages'].append(dict(stage=label,stamp=time.time(),states=ss));record('stage_complete',states=ss);print(label,flush=True);return ss
  if time.monotonic()-last>10:print(json.dumps(dict(stage=label,states={i:{k:s.get(k) for k in ['phase','position','speed','reason','failure','network_inferences']} for i,s in ss.items()},decision=last_decision)),flush=True);last=time.monotonic()
  time.sleep(.1)
 raise RuntimeError('Timeout: '+label)
def send(i,data):
 ss,_=current();goal=np.array(ss[i]['position'],float)
 if 'x' in data:goal[:2]=[data['x'],data['y']]
 if 'height' in data:goal[2]=data['height']+(ss[i]['ground'] if data.get('height_mode')=='AGL' else 0)
 if data.get('_forward_descent'):goal[2]=data['_descent_final_z']
 if data.get('_terminal_profile'):goal[2]=configuration['routes'][i]['end'][2]
 with lock:goals[i]=goal
 data['_central_token']='%s-%d-%.6f'%(mode,i,time.time());record('central_stage_command',drone_id=i,command=data,goal=goal.tolist());pubs[i].publish(String(json.dumps(data)));time.sleep(.15)
def reached(ss,i,target):return ss[i]['phase']=='HOLD' and math.dist(ss[i]['position'],target)<.6 and ss[i]['speed']<.2
def screenshot(label):
 try:
  pid=json.loads((B/'pids.json').read_text())['unity'];lines=subprocess.check_output(['wmctrl','-lp'],text=True);window=next(l.split()[0] for l in lines.splitlines() if l.split()[2]==str(pid));path=B/'screenshots'/(label+'.png')
  info=subprocess.check_output(['xwininfo','-root'],text=True);fields={k:int(re.search(r'%s:\s+(-?\d+)'%k,info).group(1)) for k in ['Width','Height']}
  display=os.environ['DISPLAY']+'+0,0'
  subprocess.run(['ffmpeg','-y','-hide_banner','-loglevel','error','-f','x11grab','-video_size','%dx%d'%(fields['Width'],fields['Height']),'-i',display,'-frames:v','1',str(path)],timeout=10,capture_output=True,check=True);record('screenshot',path=str(path))
 except Exception as exc:record('screenshot_error',error=repr(exc))

def member_stage_record(label,i,s):
 report['stages'].append(dict(stage=label,stamp=time.time(),states={i:s}))
 record('member_stage_complete',drone_id=i,member_stage=label,state=s)

def advance_member(i,s,now):
 """One member's mission transitions; other groups affect only the safety permit."""
 global parallel_enabled
 member=member_stages[i]
 limit=300 if member.startswith('WAIT_DISTANCE') else 180 if member.startswith(('CLIMB','LAND_')) else 3000 if member.startswith('CRUISE') else 600 if member.startswith('DESCENT') else 1000
 if member!='LANDED' and now-phase_started[i]>limit:raise RuntimeError('drone%d own phase timeout: %s'%(i,member))
 if member=='WAIT_DISTANCE_SAFE_TO_CLIMB_%d'%i:
  own_ready=s['phase']=='HOLD' and s['armed'] and not s['contact'] and abs(s['agl']-2)<.3 and s['speed']<.2
  if own_ready and not report.setdefault('takeoff_confirmed',{}).get(i):
   report['takeoff_confirmed'][i]=True;member_stage_record('drone%d_takeoff_AGL2'%i,i,s)
  if own_ready and last_decision.get('distance',0)>=300:
   record('central_staged_release',drone_id=i,reason='Measured 3D separation permits safe climb',actual_distance_m=last_decision['distance'],minimum_separation_m=300)
   with lock:member_stages[i]='CLIMB_%d'%i
   phase_started[i]=now
   send(i,dict(op='goto',height_mode='ABS',height=climb,x=manifest['spawns'][i][0],y=manifest['spawns'][i][1],speed=.8,_stage='STAGED_CLIMB_%d'%i))
  return
 if member=='CLIMB_%d'%i:
  if s['phase']=='HOLD' and abs(s['position'][2]-climb)<.3 and s['speed']<.25:
   member_stage_record('drone%d_at_map_Z%d'%(i,climb),i,s)
   with lock:member_stages[i]='CRUISE_%d'%i
   phase_started[i]=now;target=routes[i][indices[i]]
   send(i,dict(op='goto',height_mode='ABS',height=target[2],x=target[0],y=target[1],speed=10.5,_stage='CRUISE_SPLIT_DETOURS'))
  return
 if member=='CRUISE_%d'%i:
  if reached({i:s},i,routes[i][indices[i]]):
   record('cruise_waypoint_reached',drone_id=i,index=indices[i],position=s['position']);indices[i]+=1
   if indices[i]<len(routes[i]):
    target=routes[i][indices[i]];send(i,dict(op='goto',height_mode='ABS',height=target[2],x=target[0],y=target[1],speed=10.5,_stage='CRUISE_SPLIT_DETOURS'))
   else:
    record('member_descent_ready',drone_id=i,state=s);member_stage_record('drone%d_descent_ready'%i,i,s)
    with lock:member_stages[i]='DESCENT_%d'%i;parallel_enabled=True
    phase_started[i]=now;handoff=configuration['routes'][i]['handoff']
    send(i,dict(op='goto',height_mode='ABS',height=climb,x=handoff[0],y=handoff[1],speed=5,_forward_descent=True,_descent_entry=entries[i],_descent_final_z=handoff[2],_stage='DESCENT_%d'%i))
  return
 if member=='DESCENT_%d'%i:
  handoff=configuration['routes'][i]['handoff'];region=last_decision.get('group_coordination',{}).get('region_passed',{})
  own_pass=region.get(i,region.get(str(i),False))
  if own_pass and i not in region_recorded:
   region_recorded.add(i);record('member_region_AGL5_confirmed',drone_id=i,state=s);member_stage_record('drone%d_region_AGL5'%i,i,s)
  settled=(s['phase']=='HOLD' and math.dist(s['position'][:2],handoff[:2])<.5 and abs(s['agl']-5)<.25 and s['speed']<.2)
  if settled:settled_since.setdefault(i,now)
  else:settled_since.pop(i,None)
  if own_pass and now-settled_since.get(i,now)>=.5:
   record('member_handoff_settled',drone_id=i,state=s);member_stage_record('drone%d_handoff_AGL5'%i,i,s)
   target=configuration['routes'][i]['end']
   with lock:member_stages[i]='TERMINAL_E%d'%(i+1)
   phase_started[i]=now
   send(i,dict(op='terrain_follow',height_mode='AGL',height=5,x=target[0],y=target[1],speed=2.5,_terminal_profile=True,_profile_start=handoff[:2],_stage='TERMINAL_E%d'%(i+1)))
  return
 if member=='TERMINAL_E%d'%(i+1):
  target=configuration['routes'][i]['end']
  if s['phase']=='END_HOLD' and math.dist(s['position'][:2],target[:2])<.6 and abs(s['agl']-2)<.3 and s['speed']<.2:
   report.setdefault('endpoint_states',{})[i]=s;member_stage_record('drone%d_actual_endpoint_AGL2'%i,i,s)
   with lock:member_stages[i]='LAND_%d'%i
   phase_started[i]=now;send(i,dict(op='land',_stage='LAND_%d'%i))
  return
 if member=='LAND_%d'%i:
  if s['phase']=='LANDED' and s['contact'] and not s['armed'] and s['speed']<.15:
   member_stage_record('drone%d_landed_disarmed'%i,i,s)
   with lock:member_stages[i]='LANDED'
try:
 wait('ready',lambda ss,sc:sc.get('drone_count')==2 and sc.get('depth_source')=='unity' and sc.get('map_sha256')==manifest['map_hash'] and all(s['phase']=='IDLE' and len(s.get('frame_ages',{}))==3 and max(s['frame_ages'].values())<.6 for s in ss.values()) and all(pub.get_num_connections() for pub in pubs.values()) and last_decision.get('reason')=='clear',180)
 ss,_=current()
 for i in (0,1):
  expected=manifest['spawns'][i].copy();expected[2]+=.083263099193573
  if math.dist(ss[i]['position'],expected)>.05:raise RuntimeError('Actual spawn mismatch')
 report['initial_states']=ss;screenshot('01-ground-start')
 for i in (0,1):send(i,dict(op='takeoff',height_mode='AGL',height=2,speed=.5,_stage='TAKEOFF_AGL2'))
 if mode=='short':wait('both_takeoff_AGL2',lambda ss,sc:all(s['phase']=='HOLD' and s['armed'] and not s['contact'] and abs(s['agl']-2)<.3 and s['speed']<.2 for s in ss.values()),100)
 else:
  wait('drone0_takeoff_AGL2',lambda ss,sc:ss[0]['phase']=='HOLD' and ss[0]['armed'] and not ss[0]['contact'] and abs(ss[0]['agl']-2)<.3 and ss[0]['speed']<.2,100)
  report['takeoff_confirmed']={0:True}
 climb=240 if mode=='short' else 270
 if mode=='short':
  send(0,dict(op='goto',height_mode='ABS',height=climb,x=manifest['spawns'][0][0],y=manifest['spawns'][0][1],speed=.8,_stage='STAGED_CLIMB_0'))
  wait('drone0_at_map_Z%d'%climb,lambda ss,sc:ss[0]['phase']=='HOLD' and abs(ss[0]['position'][2]-climb)<.3 and ss[0]['speed']<.25,180)
  start=manifest['points']['start'];p0=[start['x']-25,start['y']-25,240.]
  send(0,dict(op='goto',height_mode='ABS',height=240,x=p0[0],y=p0[1],speed=1.,_stage='SHORT_WINNER_ENTRY_BEFORE_SECOND_CLIMB'))
  wait('drone0_safe_crossing_entry',lambda ss,sc:reached(ss,0,p0),150)
  send(1,dict(op='goto',height_mode='ABS',height=climb,x=manifest['spawns'][1][0],y=manifest['spawns'][1][1],speed=.8,_stage='STAGED_CLIMB_1'))
  wait('drone1_at_map_Z%d'%climb,lambda ss,sc:ss[1]['phase']=='HOLD' and abs(ss[1]['position'][2]-climb)<.3 and ss[1]['speed']<.25,180)
 S=np.array([manifest['points']['start']['x'],manifest['points']['start']['y']]);L=np.array([manifest['points']['landing']['x'],manifest['points']['landing']['y']]);ends=[np.array([manifest['points'][k]['x'],manifest['points'][k]['y']]) for k in ('end','end2')]
 if mode=='short':
  pre=[np.r_[S+[-25,-25],240],np.r_[S+[25,-25],240]];targets=[np.r_[S+[25,25],240],np.r_[S+[-25,25],240]]
  for i in (0,1):send(i,dict(op='goto',height_mode='ABS',height=240,x=float(pre[i][0]),y=float(pre[i][1]),speed=1.,_stage='SHORT_PREPOSITION'))
  wait('short_safe_crossing_entries',lambda ss,sc:all(reached(ss,i,pre[i]) for i in (0,1)),150)
  policy.declared_conflict=np.r_[S,240.]
  report['crossing_started']=time.time();record('declared_shared_crossing',centre=policy.declared_conflict.tolist())
  for i in (0,1):send(i,dict(op='goto',height_mode='ABS',height=240,x=float(targets[i][0]),y=float(targets[i][1]),speed=1.,_stage='SHORT_SHARED_CROSSING'))
  wait('short_actual_yield_and_crossing_complete',lambda ss,sc:all(reached(ss,i,targets[i]) for i in (0,1)),240);report['crossing_ended']=time.time();screenshot('02-crossing-complete')
 else:
  routes=[[[q['x'],q['y'],270.] for q in r[1:]] for r in manifest['routes']];entries=[]
  for i in (0,1):
   entry=configuration['routes'][i]['entry'];entries.append(entry[:2]);routes[i][-1]=entry.copy()
  report['routes']=routes;report['descent_entries']=entries;indices={0:0,1:0}
  phase_started={i:time.monotonic() for i in (0,1)};settled_since={};region_recorded=set();last=0.
  stage='INDEPENDENT_GROUP_LEADERS'
  with lock:
   member_stages.update({0:'CLIMB_0',1:'WAIT_DISTANCE_SAFE_TO_CLIMB_1'})
  record('independent_group_missions_enabled',configuration=configuration,scope='Each actual drone is its group leader proxy; no cross-group phase/progress barrier')
  send(0,dict(op='goto',height_mode='ABS',height=climb,x=manifest['spawns'][0][0],y=manifest['spawns'][0][1],speed=.8,_stage='STAGED_CLIMB_0'))
  while not all(member_stages[i]=='LANDED' for i in (0,1)):
   ss,sc=healthy();now=time.monotonic()
   for i in (0,1):advance_member(i,ss[i],now)
   if time.monotonic()-last>10:
    print(json.dumps(dict(stage=stage,member_stages=member_stages,states={i:{k:s.get(k) for k in ['phase','position','agl','speed','reason','failure']} for i,s in ss.items()},decision=last_decision)),flush=True);last=time.monotonic()
   time.sleep(.1)
 if mode=='short':
  for i in (0,1):send(i,dict(op='land',_stage='LAND'))
 wait('both_landed_disarmed',lambda ss,sc:all(s['phase']=='LANDED' and s['contact'] and not s['armed'] and s['speed']<.15 for s in ss.values()),180)
 time.sleep(2);ss,sc=healthy();report.update(closed_loop_passed=True,final_states=ss,final_scene=sc);screenshot('04-both-landed')
except BaseException as exc:
 report['errors'].append(str(exc));report['traceback']=traceback.format_exc()
 forced_fault='coordinator_failure:'+str(exc);stage='FAULT_BRAKE_DIAGNOSTIC';record('central_emergency_revoke',reason=forced_fault)
 tail_start=time.monotonic();settled_since=None
 while time.monotonic()-tail_start<10:
  ss,sc=current();stopped=len(ss)==2 and all(s['speed']<.15 and np.linalg.norm(s.get('control_reference',{}).get('velocity',[99,99,99]))<.15 and np.linalg.norm(s.get('control_reference',{}).get('acceleration',[99,99,99]))<.2 for s in ss.values())
  if stopped:
   if settled_since is None:settled_since=time.monotonic()
  else:settled_since=None
  if time.monotonic()-tail_start>=2 and settled_since is not None and time.monotonic()-settled_since>=.5:break
  time.sleep(.1)
 report['failure_braking_tail']=dict(seconds=time.monotonic()-tail_start,actual_stop_settled=settled_since is not None and time.monotonic()-settled_since>=.5,states=current()[0])
 for i in (0,1):
  try:send(i,dict(op='hold',_stage='FAILURE_STOP'))
  except Exception:pass
finally:
 with lock:closing=True
 producer_gate.close();safety_thread.join(2)
 if safety_thread.is_alive() or not producer_gate.wait(2):
  write_failed(B,report,'Producer shutdown incomplete; evidence not parsed');raise SystemExit(1)
 evidence_stats=writer.close();report['evidence_writer']=evidence_stats
 try:history,decisions,faults,publication_times,odom=read_verified(B,evidence_stats)
 except BaseException as exc:write_failed(B,report,'Evidence finalization failed: '+repr(exc));raise SystemExit(1)
 schedule_gaps=np.diff(publication_times).tolist()
 armed=[s['stamp'] for h in history.values() for s in h if s.get('armed')]
 active_lo=min(armed,default=float('inf'));active_hi=max(armed,default=-1.)+.15
 active_central_faults=[d for d in decisions if active_lo<=d['stamp']<=active_hi and d.get('hard_fault')]
 report['active_central_faults']=active_central_faults
 if faults or active_central_faults or forced_fault:report['errors'].append('Controller or central hard fault occurred during this run')
 try:
  with lock:
   report.update(ended=time.time(),duration_seconds=time.time()-report['started'],faults=faults,scheduler_max_interval_s=max(schedule_gaps,default=0),scheduler_p99_interval_s=float(np.percentile(schedule_gaps,99)) if schedule_gaps else None)
   if all(len(odom[i])>1 for i in (0,1)):
    a={i:np.array(odom[i]) for i in (0,1)};tt=np.arange(max(a[i][0,0] for i in (0,1)),min(a[i][-1,0] for i in (0,1)),.02);vv={i:np.column_stack([np.interp(tt,a[i][:,0],a[i][:,j]) for j in range(1,7)]) for i in (0,1)};dist=np.linalg.norm(vv[0][:,:3]-vv[1][:,:3],axis=1)
    report['min_distance_m']=float(dist.min());report['max_distance_m']=float(dist.max());report['max_actual_speed_mps']={i:float(np.linalg.norm(a[i][:,4:7],axis=1).max()) for i in (0,1)}
    status_indices={i:np.maximum(0,np.searchsorted([s['stamp'] for s in history[i]],tt,side='right')-1) for i in (0,1)}
    with (B/'pair-trajectory.csv').open('w',newline='') as f:
     w=csv.writer(f);w.writerow(['stamp','time_s','x0','y0','z0','agl0','speed0','phase0','x1','y1','z1','agl1','speed1','phase1','pair_distance_m'])
     for k,t in enumerate(tt):
      row=[t,t-tt[0]]
      for i in (0,1):
       h=history[i];idx=status_indices[i][k];s=h[idx];v=vv[i][k];row+=list(v[:3])+[float(v[2]-s['ground']),float(np.linalg.norm(v[3:])),s['phase']]
      w.writerow(row+[float(dist[k])])
    proof=[d for d in decisions if mode!='short' or report.get('crossing_started',1e100)<=d['stamp']<=report.get('crossing_ended',-1)]
    yield_samples=[d for d in proof if d.get('yielding') and not d['allow'][1] and d.get('velocities') and np.linalg.norm(d['velocities'][1])>.25]
    resumes=[(x,y) for x,y in zip(proof,proof[1:]) if x.get('yielding') and not y.get('yielding')]
    report['coordination_evidence']=dict(yield_moving_samples=len(yield_samples),released_after_conflict=len(resumes),decisions=len(decisions),mechanism='Independent group leader stages; cross-group caps/holds only for pair distance safety; formation progress control is within groups only; no native peer optimizer claims')
    if mode=='short':report['coordination_passed']=bool(yield_samples and resumes)
    else:
     audit_path=B/'audit_parallel_descent.py'
     spec=importlib.util.spec_from_file_location('actual_parallel_audit',audit_path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
     parallel=module.audit(B);(B/'independent-parallel-audit.json').write_text(json.dumps(parallel,indent=2))
     report['parallel_actual_motion']=parallel;report['coordination_passed']=bool(parallel['passed'])
     if not parallel['passed']:report['errors'].append('Actual parallel-descent acceptance failed')
    report['passed']=bool(report['closed_loop_passed'] and report['coordination_passed'] and 2<=report['min_distance_m']<=report['max_distance_m']<=2000 and max(report['max_actual_speed_mps'].values())<=15.001 and not report['errors'])
   (B/'acceptance-report.json').write_text(json.dumps(report,indent=2))
 except BaseException as exc:
  write_failed(B,report,'Postprocessing failed: '+repr(exc));raise SystemExit(1)

print(json.dumps({k:report.get(k) for k in ['passed','closed_loop_passed','coordination_passed','errors','duration_seconds','min_distance_m','max_distance_m','scheduler_max_interval_s']},indent=2),flush=True)
raise SystemExit(0 if report['passed'] else 1)
