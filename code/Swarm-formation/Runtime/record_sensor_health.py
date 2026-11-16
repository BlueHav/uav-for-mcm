"""Read-only timing evidence retained with each owned simulation's component logs."""
import json,threading,time,os
from pathlib import Path
import rospy
from std_msgs.msg import String
import portable_paths
rospy.init_node('point_sensor_health_recorder',disable_signals=True)
stream=(Path(portable_paths.SCENE)/'Reports/sensor-health.jsonl').open('w',buffering=1)
lock=threading.Lock();last={}
def record(msg,topic):
 now=time.monotonic()
 with lock:
  if now-last.get(topic,0)<.5:return
  last[topic]=now
  value=json.loads(msg.data)
  if topic.endswith('/mission/status'):
   value={k:value.get(k) for k in ('stamp','drone_id','phase','planner_mode','depth_source','cloud_depth','image_receive_ages','frame_ages','pending_depth_count','pose_cache_count','odom_age','plan_age','plan_seconds','plan_trace','display','failure')}
  elif topic=='/scene/status':value={k:value.get(k) for k in ('frames','depth_source','depth_hz','ground_hz','odom_age_s')}
  stream.write(json.dumps(dict(received=rospy.Time.now().to_sec(),monotonic=now,topic=topic,data=value))+'\n')
topics=['/swarm10/state','/swarm10/status','/sensors/status','/scene/sensor_timing','/scene/view_status','/scene/status','/scene/depth_transport','/scene/depth_sender']+['/drone_%d/mission/status'%i for i in range(10)]
for topic in topics:rospy.Subscriber(topic,String,record,callback_args=topic,queue_size=1)
if os.environ.get('YOPO_TRACE_CONTROL_STALLS')=='1':
 from control_stall_probe import ControlStallProbe
 control_probe=ControlStallProbe(portable_paths.SCENE)
rospy.spin()
