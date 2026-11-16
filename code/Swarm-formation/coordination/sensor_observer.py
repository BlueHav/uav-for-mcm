# Ground-only acquisition/transport diagnostic. Never publishes a task or arm.
import json,time,threading,subprocess,os,signal
from pathlib import Path
import rospy
from sensor_msgs.msg import Image,PointCloud2
from geometry_msgs.msg import PoseStamped,PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
B=Path(__file__).parent;rospy.init_node('read_only_stream_observer',disable_signals=True);lock=threading.RLock();latest={};counts={};scene={};statuses={};pids=json.loads((B/'pids.json').read_text());log=(B/'sensor-probe.jsonl').open('w',buffering=1)
def callback(m,key):
 stamp=m.header.stamp.to_sec()
 with lock:latest[key]=dict(header=stamp,received=time.time());counts[key]=counts.get(key,0)+1
for i in (0,1):
 for camera in ('front','up','down'):
  image='depth' if camera=='front' else camera+'_depth';pose='depth_camera_pose' if camera=='front' else camera+'_camera_pose'
  rospy.Subscriber('/drone_%d/%s'%(i,image),Image,callback,callback_args='%d/%s/image'%(i,camera),queue_size=1,buff_size=2**21,tcp_nodelay=True)
  rospy.Subscriber('/drone_%d/%s'%(i,pose),PoseStamped,callback,callback_args='%d/%s/pose'%(i,camera),queue_size=1,tcp_nodelay=True)
 for topic,kind in [('local_ground',PointCloud2),('ground',PointStamped),('odom',Odometry)]:rospy.Subscriber('/drone_%d/%s'%(i,topic),kind,callback,callback_args='%d/%s'%(i,topic),queue_size=1,tcp_nodelay=True)
 def status(m,i=i):
  with lock:statuses[i]=json.loads(m.data)
 rospy.Subscriber('/drone_%d/mission/status'%i,String,status,queue_size=1)
def scene_cb(m):
 with lock:scene.clear();scene.update(dict(received=time.time(),value=json.loads(m.data)))
rospy.Subscriber('/scene/status',String,scene_cb,queue_size=1)
running=True
def stop(*args):
 global running
 running=False
signal.signal(signal.SIGTERM,stop)
start=time.monotonic();lastsystem=0;system={};bad=[]
while running and time.monotonic()-start<7200:
 now=time.time()
 if now-lastsystem>.5:
  system={};lastsystem=now
  for name,pid in pids.items():
   try:
    path=Path('/proc/%d'%pid);raw=(path/'stat').read_text();parts=raw[raw.rindex(')')+2:].split();system[name]=dict(pid=pid,state=parts[0],cpu_ticks=int(parts[11])+int(parts[12]),wchan=(path/'wchan').read_text().strip(),threads=int(parts[17]))
   except OSError:pass
  system['tcp']=subprocess.run(['ss','-tinp','( sport = :10012 or dport = :10012 )'],capture_output=True,text=True,timeout=2).stdout
 with lock:
  ages={k:dict(header_age=now-v['header'],receive_age=now-v['received'],count=counts[k]) for k,v in latest.items()};s={i:{k:v.get(k) for k in ['armed','phase','image_received','pending_depth_count','frame_ages','image_receive_ages','plan_seconds']} for i,v in statuses.items()};row=dict(stamp=now,age=ages,scene=dict(scene),states=s,system=system)
  # Read-only observer permits actual flight; it publishes no commands.
 log.write(json.dumps(row)+'\n')
 if time.monotonic()-start>15 and any(v['header_age']>.6 for k,v in ages.items() if k.endswith('/image')):bad.append(now)
 time.sleep(.1)
report=dict(scope='Read-only actual flight diagnostic; no arm/task publications',stream_stale_observations=len(bad),started=start,ended=time.monotonic());(B/'sensor-probe-report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report));log.close()
