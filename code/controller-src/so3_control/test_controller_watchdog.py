#!/usr/bin/env python3
"""Independent controller heartbeat watchdog SIGSTOP test, isolated ROS11324.
No Unity/oracle; synthetic flat ground. Validates high-altitude brake/hold only.
"""
import portable_paths
import os,sys,time,json,signal,subprocess,math,datetime
import numpy as np
os.environ['ROS_MASTER_URI']='http://127.0.0.1:11324'
import rospy
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Bool,String
from geometry_msgs.msg import PointStamped

if '--publisher' in sys.argv:
 rospy.init_node('watchdog_command_source',anonymous=True)
 state={}
 def update(m):
  p=m.pose.pose.position;v=m.twist.twist.linear;state.update(p=np.array([p.x,p.y,p.z]),v=np.array([v.x,v.y,v.z]))
 rospy.Subscriber('/drone_0/odom',Odometry,update,queue_size=1,tcp_nodelay=True)
 pub=rospy.Publisher('/drone_0/pos_cmd',PositionCommand,queue_size=1)
 arm=rospy.Publisher('/drone_0/arm',Bool,queue_size=1,latch=True)
 while not state or pub.get_num_connections()==0:time.sleep(.02)
 refp=state['p'].copy();refv=np.zeros(3);last=time.monotonic();arm.publish(Bool(True));phase='ASCEND'
 while not rospy.is_shutdown():
  now=time.monotonic();dt=min(.05,max(.001,now-last));last=now;p=state['p'];v=state['v'];desired=np.zeros(3)
  e=20-p[2];desired[2]=np.clip(e*1.2,-1.8,1.8)
  if abs(e)<.1 and abs(v[2])<.15:phase='CRUISE'
  if phase=='CRUISE':desired[0]=10.5
  new=refv.copy();delta=desired[:2]-refv[:2];new[:2]+=delta*min(1,2.5*dt/max(np.linalg.norm(delta),1e-9));new[2]+=np.clip(desired[2]-refv[2],-1.2*dt,1.2*dt)
  acc=(new-refv)/dt;refv=new;refp+=refv*dt
  diff=refp-p
  if np.linalg.norm(diff)>1.5:refp=p+diff*1.5/np.linalg.norm(diff)
  c=PositionCommand();c.header.stamp=rospy.Time.now();c.trajectory_flag=0
  for obj,a in [(c.position,refp),(c.velocity,refv),(c.acceleration,acc)]:obj.x,obj.y,obj.z=map(float,a)
  pub.publish(c);time.sleep(max(.001,.02-(time.monotonic()-now)))
 sys.exit(0)

processes=[];publisher=None;samples=[];faults=[];state={};start=time.monotonic()
try:
 master=subprocess.Popen(['roscore','-p','11324'],stdout=open('/tmp/watchdog-master.log','w'),stderr=subprocess.STDOUT,start_new_session=True);processes.append(master);time.sleep(2)
 launch=subprocess.Popen(['roslaunch','so3_quadrotor_simulator','mission_ros1.launch','count:=1'],stdout=open('/tmp/watchdog-launch.log','w'),stderr=subprocess.STDOUT,start_new_session=True);processes.append(launch)
 rospy.init_node('watchdog_fault_injection',anonymous=True)
 def update(m):
  p=m.pose.pose.position;v=m.twist.twist.linear
  row=[m.header.stamp.to_sec(),time.monotonic()-start,p.x,p.y,p.z,v.x,v.y,v.z];samples.append(row);state.update(p=np.array(row[2:5]),v=np.array(row[5:8]))
 def fault(m):faults.append([time.monotonic()-start,json.loads(m.data)])
 rospy.Subscriber('/drone_0/odom',Odometry,update,queue_size=100,tcp_nodelay=True)
 rospy.Subscriber('/drone_0/controller_fault',String,fault,queue_size=10)
 ground=rospy.Publisher('/drone_0/ground',PointStamped,queue_size=1)
 stale_pub=rospy.Publisher('/drone_0/pos_cmd',PositionCommand,queue_size=1)
 def wait(seconds,stale=False):
  end=time.monotonic()+seconds
  while time.monotonic()<end:
   if state:
    g=PointStamped();g.header.stamp=rospy.Time.now();g.header.frame_id='world';g.point.x=state['p'][0];g.point.y=state['p'][1];ground.publish(g)
   if stale:
    bad=PositionCommand();bad.header.stamp=rospy.Time.now()-rospy.Duration(2.);bad.trajectory_flag=0;stale_pub.publish(bad)
   time.sleep(.05)
 wait(2)
 assert state,'Missing odometry'
 assert not any(x[1]['active'] for x in faults),'Disarmed idle triggered watchdog'
 publisher=subprocess.Popen([sys.executable,__file__,'--publisher'],stdout=open('/tmp/watchdog-publisher.log','w'),stderr=subprocess.STDOUT,start_new_session=True);processes.append(publisher)
 deadline=time.monotonic()+40;steady=None
 while time.monotonic()<deadline:
  wait(.1)
  if np.linalg.norm(state['v'][:2])>=10.4:
   if steady is None:steady=time.monotonic()
   if time.monotonic()-steady>2:break
  else:steady=None
 else:raise AssertionError('Healthy command tracking never reached10.4m/s')
 stop_time=time.monotonic()-start;stop_p=state['p'].copy();os.kill(publisher.pid,signal.SIGSTOP)
 wait(10,stale=True)
 active=[f for f in faults if f[1]['active']]
 assert active,'No watchdog fault after SIGSTOP'
 assert np.linalg.norm(state['v'])<.1,('Failed to stop',state)
 hold_p=state['p'].copy();os.kill(publisher.pid,signal.SIGCONT);wait(2)
 assert np.linalg.norm(state['p']-hold_p)<.03,'Resumed command source cleared sticky fault'
 assert faults[-1][1]['active'],'Fault cleared without explicit reset/rearm'
 # Disarm/rearm clears the latch; resumed fresh source is still alive and sending.
 arm=rospy.Publisher('/drone_0/arm',Bool,queue_size=1)
 until=time.monotonic()+3
 while arm.get_num_connections()==0 and time.monotonic()<until:wait(.1)
 assert arm.get_num_connections()>0,'Rearm test publisher failed to connect'
 arm.publish(Bool(False));wait(.3);arm.publish(Bool(True));wait(.25)
 rearm_cleared=not faults[-1][1]['active']
 segment=[s for s in samples if stop_time<=s[1]<=stop_time+10]
 accel=[]
 for a,b in zip(segment,segment[1:]):
  dt=b[0]-a[0]
  if dt>0:accel.append(np.linalg.norm(np.array(b[5:7])-a[5:7])/dt)
 result=dict(scope='synthetic flat terrain independent heartbeat SIGSTOP; high-altitude only; terrain-aware fault stop NOT validated',fault_latency_s=active[0][0]-stop_time,stop_distance_m=float(np.linalg.norm(hold_p[:2]-stop_p[:2])),stopped_speed_mps=float(np.linalg.norm([s for s in segment if s[1]>stop_time+9][-1][5:8])),peak_actual_xy_accel_mps2=float(max(accel)),max_altitude_error_m=float(max(abs(s[4]-20) for s in segment)),sticky_resume_rejected=True,stale_header_packets_cannot_refresh_heartbeat=True,explicit_rearm_clears=rearm_cleared,faults=faults,samples=samples)
 report=(str(portable_paths.OPS) + '/reports/controller-watchdog-')+datetime.datetime.now().strftime('%Y%m%d-%H%M%S')+'.json'
 with open(report,'w') as f:json.dump(result,f)
 print(json.dumps({k:v for k,v in result.items() if k not in ('samples','faults')},indent=2));print(report)
 assert result['explicit_rearm_clears'],'Explicit rearm did not clear fault'
 assert result['fault_latency_s']<=.4,'Watchdog detection exceeded .4s'
 assert result['peak_actual_xy_accel_mps2']<=3.05,'Physical braking acceleration exceeded 3m/s² plus .05 measurement tolerance'
 assert result['stop_distance_m']<=35,'Jerk-limited stopping corridor exceeded35m at10.5m/s'
finally:
 if publisher:
  try:os.kill(publisher.pid,signal.SIGCONT)
  except ProcessLookupError:pass
 for p in reversed(processes):
  try:os.killpg(p.pid,signal.SIGINT)
  except ProcessLookupError:pass
 for p in reversed(processes):
  try:p.wait(timeout=10)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM)





