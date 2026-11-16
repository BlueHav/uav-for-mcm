# Dedicated no-flight load diagnostic. No Runtime imports, no arm/task publishers.
import ast,csv,collections,copy,gc,json,sys,threading,time,types
from pathlib import Path
import numpy as np,rospy
from std_msgs.msg import String
from shared_policy import Policy
B=Path(sys.argv[1]);scale=int(sys.argv[2]);sink=sys.argv[3];out=B/('load-%s-%dx'%(sink,scale));out.mkdir(exist_ok=True)
rospy.init_node('coordinator_load_probe',anonymous=True,disable_signals=True)
# Real retained object schemas, without loading any flight node or planner.
history={0:[],1:[]};decisions=[]
for line in (B/'telemetry.jsonl').open():
 e=json.loads(line)
 if e['event']=='mission_status':history[e['drone_id']].append(e['state'])
 if e['event']=='central_decision':decisions.append(e['decision'])
odom={i:np.loadtxt(B/('trajectory-drone-%d.csv'%i),delimiter=',',skiprows=1).tolist() for i in (0,1)}
if scale==2:
 for h in history.values():h+=copy.deepcopy(h)
 for h in odom.values():h+=copy.deepcopy(h)
 decisions+=copy.deepcopy(decisions)
preload=dict(status={i:len(h) for i,h in history.items()},odom={i:len(h) for i,h in odom.items()},decisions=len(decisions))
gc.collect(2)
traces={k:collections.deque(maxlen=100000) for k in ['lock','json','io','gc']};local=threading.local();realjson=json
class Lock:
 def __init__(self):self.r=threading.RLock()
 def __enter__(self):
  start=time.monotonic();self.r.acquire();local.lock_start=(start,time.monotonic());return self
 def __exit__(self,*a):
  end=time.monotonic();start,held=local.lock_start;self.r.release();traces['lock'].append((start,held-start,end-held))
class Sink:
 def __init__(self,name):self.closed=False;self.f=(out/name).open('w',buffering=1) if sink=='disk' else None
 def write(self,text):
  start=time.monotonic()
  if self.f is not None:self.f.write(text)
  traces['io'].append((start,time.monotonic()-start));return len(text)
 def close(self):
  self.closed=True
  if self.f is not None:self.f.close()
def dumps(*args,**kw):
 start=time.monotonic();result=realjson.dumps(*args,**kw);traces['json'].append((start,time.monotonic()-start));return result
jsonproxy=types.SimpleNamespace(dumps=dumps,loads=realjson.loads)
lock=Lock();log=Sink('telemetry.jsonl');raw={i:Sink('odom%d.csv'%i) for i in (0,1)};writers={i:csv.writer(raw[i]) for i in (0,1)}
env=dict(lock=lock,closing=False,log=log,json=jsonproxy,time=time,stage='LOAD_DIAGNOSTIC',states={},history=history,global_states={},odom=odom,writers=writers,goals={},policy=Policy(),last_decision={},decisions=decisions,forced_fault=None,authority=rospy.Publisher('/load_probe/decision',String,queue_size=1),rospy=rospy,String=String,schedule_gaps=[],CFG=dict(period=.05))
tree=ast.parse((B/'coordinator.py').read_text());names=['record','state_cb','odom_cb','safety_loop'];functions=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in names];exec(compile(ast.Module(body=functions,type_ignores=[]),str(Path(__file__).parent/'coordinator.py'),'exec'),env)
gctimes={}
def gc_callback(phase,info):
 t=time.monotonic();generation=info['generation']
 if phase=='start':gctimes[generation]=t
 else:traces['gc'].append((gctimes.get(generation,t),t-gctimes.get(generation,t),generation))
gc.callbacks.append(gc_callback)
start=time.monotonic();threads=[]
def stream(i):
 base=history[i][-1];data=realjson.dumps(base);position=np.array(base['position']);velocity=np.zeros(3);nextodom=time.monotonic();nextstatus=nextodom
 while not env['closing']:
  now=time.time();message=types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_sec=lambda n=now:n),frame_id='world'),pose=types.SimpleNamespace(pose=types.SimpleNamespace(position=types.SimpleNamespace(x=position[0],y=position[1],z=position[2]))),twist=types.SimpleNamespace(twist=types.SimpleNamespace(linear=types.SimpleNamespace(x=0.,y=0.,z=0.))))
  env['odom_cb'](message,i)
  if time.monotonic()>=nextstatus:env['state_cb'](types.SimpleNamespace(data=data),i);nextstatus+=.1
  nextodom+=.02;time.sleep(max(0.,nextodom-time.monotonic()))
for i in (0,1):t=threading.Thread(target=stream,args=(i,),daemon=True);t.start();threads.append(t)
t=threading.Thread(target=env['safety_loop'],daemon=True);t.start();threads.append(t)
forced=[]
for _ in range(3):
 time.sleep(2);begin=time.monotonic();gc.collect(2);forced.append(dict(start=begin,seconds=time.monotonic()-begin))
time.sleep(1);env['closing']=True
for t in threads:t.join(2)
gc.callbacks.remove(gc_callback);log.close()
for f in raw.values():f.close()
def maxitem(name,index):return max((row[index] for row in traces[name]),default=0)
report=dict(scope='No-flight actual callback/schema load replay on independent ROS master; explicit full GC is diagnostic, not proof of historical natural-GC cause',scale=scale,sink=sink,preload=preload,seconds=time.monotonic()-start,forced_full_gc=forced,gc=list(traces['gc']),scheduler_max_gap=max(env['schedule_gaps'],default=0),scheduler_gaps_over_0_2=[v for v in env['schedule_gaps'] if v>.2],max_lock_wait=maxitem('lock',1),max_lock_hold=maxitem('lock',2),max_json_seconds=maxitem('json',1),max_io_seconds=maxitem('io',1))
(out/'result.json').write_text(realjson.dumps(report,indent=2));print(realjson.dumps(report,indent=2));rospy.signal_shutdown('Diagnostic finished')
