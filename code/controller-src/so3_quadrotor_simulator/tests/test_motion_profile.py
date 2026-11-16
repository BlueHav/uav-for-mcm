#!/usr/bin/env python3
"""Synthetic flat-ground controller test only, on isolated master 11322."""
import portable_paths
import os,subprocess,time,json,signal,math,datetime
import numpy as np
os.environ['ROS_MASTER_URI']='http://127.0.0.1:11322'
master=subprocess.Popen(['roscore','-p','11322'],stdout=open('/tmp/profile-master.log','w'),stderr=subprocess.STDOUT,start_new_session=True)
time.sleep(2)
launch=subprocess.Popen(['roslaunch','so3_quadrotor_simulator','mission_ros1.launch','count:=1'],stdout=open('/tmp/profile-launch.log','w'),stderr=subprocess.STDOUT,start_new_session=True)
try:
 import rospy
 from nav_msgs.msg import Odometry
 from std_msgs.msg import Bool
 from geometry_msgs.msg import PointStamped
 from quadrotor_msgs.msg import PositionCommand
 rospy.init_node('controller_profile',anonymous=True)
 state={}; samples=[];phase='IDLE';start=time.monotonic();offset=.083263099193573
 def odom(m):
  p=m.pose.pose.position;v=m.twist.twist.linear
  state.update(p=np.array([p.x,p.y,p.z]),v=np.array([v.x,v.y,v.z]))
  samples.append([time.monotonic()-start,phase,p.x,p.y,p.z,v.x,v.y,v.z])
 rospy.Subscriber('/drone_0/odom',Odometry,odom,queue_size=100,tcp_nodelay=True)
 arm=rospy.Publisher('/drone_0/arm',Bool,queue_size=1,latch=True)
 cmd=rospy.Publisher('/drone_0/pos_cmd',PositionCommand,queue_size=1)
 ground=rospy.Publisher('/drone_0/ground',PointStamped,queue_size=1)
 time.sleep(2)
 assert state,'No odometry'
 refp=state['p'].copy();refv=np.zeros(3);previous=time.monotonic();phase='ASCEND';phase_start=previous
 arm.publish(Bool(True));tick=0
 while not rospy.is_shutdown():
  now=time.monotonic();dt=min(.05,max(.001,now-previous));previous=now;p=state['p'].copy();v=state['v'].copy()
  desired=np.zeros(3)
  if phase=='ASCEND':
   e=150-p[2];desired[2]=np.sign(e)*min(2,math.sqrt(2*1.2*abs(e)))
   if abs(e)<.08:desired[2]=1.5*e
   if abs(e)<.25 and np.linalg.norm(v)<.3:phase='CRUISE';phase_start=now
  elif phase=='CRUISE':
   desired[0]=10.5;desired[2]=np.clip((150-p[2])*1.5,-2,2)
   if now-phase_start>=75:phase='BRAKE';phase_start=now
  elif phase=='BRAKE':
   desired[2]=np.clip((150-p[2])*1.5,-2,2)
   if now-phase_start>4 and np.linalg.norm(v)<.1:phase='DESCEND';phase_start=now
  elif phase=='DESCEND':
   e=offset-p[2];desired[2]=np.sign(e)*min(2,math.sqrt(2*1.2*abs(e)))
   if p[2]<1.:desired[2]=max(-.45,min(.45,1.5*e))
   if abs(e)<.08:desired[2]=1.5*e
   if p[2]<=offset+.001 and abs(v[2])<.01:phase='LANDED';phase_start=now;arm.publish(Bool(False))
  elif phase=='LANDED':
   if now-phase_start>1:break
  if now-start>270:raise RuntimeError('Profile timeout at '+phase)
  new=refv.copy();delta=desired[:2]-refv[:2];norm=np.linalg.norm(delta);new[:2]+=delta*min(1,3*dt/max(norm,1e-9));new[2]+=np.clip(desired[2]-refv[2],-1.5*dt,1.5*dt);new[2]=np.clip(new[2],-2,2)
  refa=(new-refv)/dt;refv=new;refp+=refv*dt
  diff=refp-p;norm=np.linalg.norm(diff)
  if norm>1.5:refp=p+diff*1.5/norm
  if phase in ('CRUISE','BRAKE'):refp[2]=150.
  if phase=='DESCEND':refp[2]=max(offset-.01,refp[2])
  m=PositionCommand();m.header.stamp=rospy.Time.now();m.trajectory_flag=0
  for obj,a in [(m.position,refp),(m.velocity,refv),(m.acceleration,refa)]:obj.x,obj.y,obj.z=map(float,a)
  cmd.publish(m)
  if tick%5==0:
   g=PointStamped();g.header.stamp=rospy.Time.now();g.header.frame_id='world';g.point.x=p[0];g.point.y=p[1];g.point.z=0;ground.publish(g)
  tick+=1;time.sleep(max(.001,.02-(time.monotonic()-now)))
 cruise=[s for s in samples if s[1]=='CRUISE'];steady=[s for s in cruise if s[0]>cruise[0][0]+5]
 streak=best=0.;prev=None
 for s in cruise:
  speed=math.sqrt(s[5]**2+s[6]**2+s[7]**2)
  streak=streak+(s[0]-prev if prev is not None else 0) if speed>=10 else 0;best=max(best,streak);prev=s[0]
 result=dict(scope='synthetic flat terrain controller integration; not full scene flight',complete=phase=='LANDED',elapsed=time.monotonic()-start,cruise_continuous_seconds_ge_10=best,cruise_speed_mean=float(np.mean([np.linalg.norm(s[5:8]) for s in steady])),cruise_abs_z_error_max=float(max(abs(s[4]-150) for s in steady)),max_abs_vz=float(max(abs(s[7]) for s in samples)),max_speed=float(max(np.linalg.norm(s[5:8]) for s in samples)),samples=samples)
 report=(str(portable_paths.OPS) + '/reports/controller-profile-')+datetime.datetime.now().strftime('%Y%m%d-%H%M%S')+'.json';os.makedirs(os.path.dirname(report),exist_ok=True)
 with open(report,'w') as f:json.dump(result,f)
 print(json.dumps({k:v for k,v in result.items() if k!='samples'},indent=2));print(report)
finally:
 for process in (launch,master):os.killpg(process.pid,signal.SIGINT)
 for process in (launch,master):
  try:process.wait(timeout=10)
  except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM)
