import portable_paths
import os, subprocess, time, json, signal
os.environ['ROS_MASTER_URI']='http://127.0.0.1:11322'
base=(str(portable_paths.CONTROLLER))
master=subprocess.Popen(['roscore','-p','11322'],stdout=open('/tmp/ground-test-master.log','w'),stderr=subprocess.STDOUT,start_new_session=True)
time.sleep(2)
launch=subprocess.Popen(['roslaunch','so3_quadrotor_simulator','mission_ros1.launch','count:=2'],stdout=open('/tmp/ground-test-launch.log','w'),stderr=subprocess.STDOUT,start_new_session=True)
try:
 import rospy
 from nav_msgs.msg import Odometry
 from std_msgs.msg import Bool
 from geometry_msgs.msg import PointStamped
 from quadrotor_msgs.msg import PositionCommand
 rospy.init_node('ground_physics_test',anonymous=True)
 state={}; contact={}; history=[[],[]]
 def odom(m,i):
  state[i]=m; history[i].append((time.time(),m.pose.pose.position.z,m.twist.twist.linear.z))
 for i in range(2):
  rospy.Subscriber('/drone_%d/odom'%i,Odometry,lambda m,j=i:odom(m,j))
  rospy.Subscriber('/drone_%d/contact'%i,Bool,lambda m,j=i:contact.update({j:m.data}))
 arms=[rospy.Publisher('/drone_%d/arm'%i,Bool,queue_size=1,latch=True) for i in range(2)]
 resets=[rospy.Publisher('/drone_%d/reset'%i,Bool,queue_size=1) for i in range(2)]
 cmds=[rospy.Publisher('/drone_%d/pos_cmd'%i,PositionCommand,queue_size=1) for i in range(2)]
 grounds=[rospy.Publisher('/drone_%d/ground'%i,PointStamped,queue_size=1) for i in range(2)]
 def run(seconds, targets=None):
  end=time.time()+seconds; k=0
  while time.time()<end:
   for i in range(2):
    if i not in state:continue
    pos=state[i].pose.pose.position
    if k%5==0:
     g=PointStamped();g.header.stamp=rospy.Time.now();g.header.frame_id='world';g.point.x=pos.x;g.point.y=pos.y;g.point.z=0;grounds[i].publish(g)
    if targets and i in targets:
     c=PositionCommand(); c.header.stamp=rospy.Time.now(); c.position.x=45;c.position.y=[-58,-42][i];c.position.z=targets[i];c.trajectory_flag=0;cmds[i].publish(c)
   k+=1;time.sleep(.02)
 run(3)
 floor=.083263099193573
 assert len(state)==2, 'missing odom'
 idle=[state[i].pose.pose.position.z for i in range(2)]
 assert all(abs(z-floor)<.0001 for z in idle),idle
 assert all(contact.values()),contact
 arms[0].publish(True);run(7,{0:2})
 takeoff=state[0].pose.pose.position.z
 assert abs(takeoff-2)<.1,takeoff
 assert abs(state[1].pose.pose.position.z-floor)<1e-4,'peer moved on arm'
 arms[1].publish(True);run(6,{0:2,1:1.5})
 peer_before=state[1].pose.pose.position.z
 resets[0].publish(True);run(2,{1:1.5})
 reset=state[0].pose.pose.position.z;peer_after=state[1].pose.pose.position.z
 assert abs(reset-floor)<1e-4,reset
 assert abs(peer_after-peer_before)<.02,(peer_before,peer_after)
 arms[0].publish(True);run(5,{0:1,1:1.5})
 run(7,{0:floor-.02,1:1.5})
 land=state[0].pose.pose.position.z
 assert contact[0] and abs(land-floor)<1e-4,(land,contact)
 arms[0].publish(False);run(2,{1:1.5})
 assert abs(state[0].pose.pose.position.z-floor)<1e-4
 print(json.dumps({'idle_z':idle,'takeoff_z':takeoff,'reset_z':reset,'peer_before_reset':peer_before,'peer_after_reset':peer_after,'landing_z':land,'contact':contact,'samples':[len(h) for h in history]},indent=2))
finally:
 for p in [launch,master]:
  os.killpg(p.pid,signal.SIGINT)
 for p in [launch,master]:
  try:p.wait(timeout=10)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM)
