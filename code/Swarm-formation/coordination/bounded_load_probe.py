# Dedicated 120s no-flight test: actual callbacks/writer, independent ROS master.
import ast,collections,gc,json,sys,threading,time,types
from pathlib import Path
import numpy as np,rospy
from std_msgs.msg import String
from producer_gate import ProducerGate
from evidence_writer import EvidenceWriter
from shared_policy import Policy,CFG
S=Path(__file__).parent;fixture=Path(sys.argv[1]);out=Path(sys.argv[2]);out.mkdir(exist_ok=True,parents=True)
latest={};original_counts={0:0,1:0};decision_count=0
for line in (fixture/'telemetry.jsonl').open():
 e=json.loads(line)
 if e['event']=='mission_status':latest[e['drone_id']]=e['state'];original_counts[e['drone_id']]+=1
 if e['event']=='central_decision':decision_count+=1;decision_template=e['decision']
original_odom={i:sum(1 for _ in (fixture/('trajectory-drone-%d.csv'%i)).open())-1 for i in (0,1)}
# Only two immutable JSON templates remain reachable; no retained full history.
templates={i:json.dumps(v) for i,v in latest.items()};decision_template=json.dumps(decision_template);del e
gc.collect(2);rospy.init_node('bounded_coordinator_load_probe',anonymous=True,disable_signals=True)
writer=EvidenceWriter(out,capacity=512);gate=ProducerGate();history={i:collections.deque(maxlen=32) for i in (0,1)};odom={i:collections.deque(maxlen=128) for i in (0,1)};decisions=collections.deque(maxlen=64);received=collections.deque(maxlen=10000);gc_events=collections.deque(maxlen=1024);stops={};counts={0:dict(status=0,odom=0),1:dict(status=0,odom=0)};pressure_decisions=[0]
def receiver(m):received.append((time.monotonic(),time.time()-json.loads(m.data)['stamp']))
rospy.Subscriber('/load_probe/bounded_decision',String,receiver,queue_size=100,tcp_nodelay=True)
env=dict(lock=threading.RLock(),closing=False,producer_gate=gate,writer=writer,stage='BOUNDED_LOAD_ONLY',states={},history=history,global_states={},odom=odom,goals={},policy=Policy(),last_decision={},decisions=decisions,forced_fault=None,authority=rospy.Publisher('/load_probe/bounded_decision',String,queue_size=1,tcp_nodelay=True),rospy=rospy,String=String,schedule_gaps=collections.deque(maxlen=10000),CFG=CFG,time=time,json=json)
tree=ast.parse((S/'coordinator.py').read_text());names=('record','state_cb','odom_cb','safety_loop');nodes=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in names];exec(compile(ast.Module(body=nodes,type_ignores=[]),str(S/'coordinator.py'),'exec'),env)
for name in ('state_cb','odom_cb'):env[name]=gate.wrap(env[name])
start=time.monotonic();duration=120.;threads=[]
def stream(i):
 p=latest[i]['position'];next_tick=time.monotonic();next_status=next_tick
 while time.monotonic()-start<duration and not env['closing']:
  now=time.time();m=types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_sec=lambda n=now:n),frame_id='world'),pose=types.SimpleNamespace(pose=types.SimpleNamespace(position=types.SimpleNamespace(x=p[0],y=p[1],z=p[2]))),twist=types.SimpleNamespace(twist=types.SimpleNamespace(linear=types.SimpleNamespace(x=0.,y=0.,z=0.))))
  env['odom_cb'](m,i);counts[i]['odom']+=1
  if time.monotonic()>=next_status:env['state_cb'](types.SimpleNamespace(data=templates[i]),i);counts[i]['status']+=1;next_status+=.005
  next_tick+=.001;time.sleep(max(0.,next_tick-time.monotonic()))
def pressure():
 next_tick=time.monotonic()
 while time.monotonic()-start<duration and not env['closing']:
  value=json.loads(decision_template);env['record']('allocation_pressure_fixture',value=value);pressure_decisions[0]+=1;next_tick+=.0025;time.sleep(max(0.,next_tick-time.monotonic()))
def gc_callback(phase,info):
 if phase=='start':stops[info['generation']]=time.monotonic()
 else:gc_events.append((info['generation'],time.monotonic()-stops.get(info['generation'],time.monotonic())))
gc.callbacks.append(gc_callback)
for i in (0,1):t=threading.Thread(target=stream,args=(i,),daemon=True);t.start();threads.append(t)
t=threading.Thread(target=pressure,daemon=True);t.start();threads.append(t);safety=threading.Thread(target=env['safety_loop'],daemon=True);safety.start()
checkpoints=[];forced=[]
while time.monotonic()-start<duration:
 time.sleep(2);begin=time.monotonic();gc.collect(2);forced.append(time.monotonic()-begin)
 if len(checkpoints)<2 and all(counts[i]['status']>=(len(checkpoints)+1)*original_counts[i] and counts[i]['odom']>=(len(checkpoints)+1)*original_odom[i] for i in (0,1)) and pressure_decisions[0]>=(len(checkpoints)+1)*decision_count:
  checkpoints.append(dict(multiple=len(checkpoints)+1,elapsed=time.monotonic()-start,counts={i:dict(v) for i,v in counts.items()},pressure_decisions=pressure_decisions[0],max_publish_gap=max(env['schedule_gaps'],default=0)))
env['closing']=True;gate.close()
for t in threads:t.join(2)
safety.join(2);drained=gate.wait(2);stats=writer.close();gc.callbacks.remove(gc_callback);times=[r[0] for r in received];gaps=np.diff(times);maxpub=max(env['schedule_gaps'],default=0);maxrecv=float(max(gaps,default=0));maxage=max((r[1] for r in received),default=999)
report=dict(passed=bool(len(checkpoints)==2 and maxpub<.2 and maxrecv<.2 and maxage<.2 and not env['forced_fault'] and not stats['error'] and not stats['rejected'] and stats['written']==stats['submitted'] and not stats['pending'] and not stats['writer_alive'] and drained and not safety.is_alive()),scope='120s independent-master no-flight pressure; exact callbacks and bounded disk writer. 200status/s/id,1000odom/s/id,400decision-schema allocations/s; explicit full GC every2s. No Runtime/arm/task nodes.',seconds=time.monotonic()-start,original_counts=original_counts,original_odom=original_odom,original_decisions=decision_count,counts=counts,pressure_decisions=pressure_decisions[0],checkpoints=checkpoints,max_publish_gap=maxpub,max_receive_gap=maxrecv,max_receive_age=maxage,max_forced_gc=max(forced,default=0),max_gc=max((r[1] for r in gc_events),default=0),retained_sizes=dict(status={i:len(v) for i,v in history.items()},odom={i:len(v) for i,v in odom.items()},decisions=len(decisions)),writer=stats,forced_fault=env['forced_fault'])
(out/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));rospy.signal_shutdown('No-flight pressure test finished')
