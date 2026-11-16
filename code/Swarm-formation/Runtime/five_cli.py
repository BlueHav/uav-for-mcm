#!/usr/bin/env python3
import json,sys,time
import rospy
from std_msgs.msg import String
rospy.init_node('five_client',anonymous=True)
op=sys.argv[1] if len(sys.argv)>1 else 'status'
if op=='status':
    print(json.dumps(json.loads(rospy.wait_for_message('/swarm5/status',String,timeout=5).data),indent=2));raise SystemExit(0)
if op not in ('start','hold','cancel','land'):raise SystemExit('Use start/status/hold/cancel/land')
pub=rospy.Publisher('/swarm5/command',String,queue_size=1)
end=time.monotonic()+5
while pub.get_num_connections()==0 and time.monotonic()<end:time.sleep(.1)
if not pub.get_num_connections():raise SystemExit('Five-agent coordinator unavailable; start five-mode first')
pub.publish(String(json.dumps(dict(op=op))));time.sleep(.4)
print('Group command sent: '+op)
