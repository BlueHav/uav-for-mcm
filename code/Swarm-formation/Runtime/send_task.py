import sys,json,time,rospy
from std_msgs.msg import String
if len(sys.argv)!=3 or sys.argv[1] not in ('0','1','2','3','4'):raise SystemExit('Usage: run_task.sh DRONE_ID JSON; id 0..4')
value=json.loads(sys.argv[2]);rospy.init_node('mountain_task_client',anonymous=True)
pub=rospy.Publisher('/drone_'+sys.argv[1]+'/task',String,queue_size=1)
end=time.monotonic()+10
while pub.get_num_connections()==0 and time.monotonic()<end:time.sleep(.1)
if pub.get_num_connections()==0:raise SystemExit('Drone task subscriber unavailable; start the requested drone first')
pub.publish(String(json.dumps(value)));time.sleep(.3)
print('Task sent to drone_'+sys.argv[1]+': '+json.dumps(value))
