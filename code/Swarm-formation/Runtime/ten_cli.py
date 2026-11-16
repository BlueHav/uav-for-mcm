#!/usr/bin/env python3
import argparse,json
import rospy
from std_msgs.msg import String
p=argparse.ArgumentParser();p.add_argument('op',choices=['start','assemble','climb','cruise','descend','hold','cancel','status']);p.add_argument('--altitude',type=float,default=150);p.add_argument('--distance',type=float,default=30);p.add_argument('--x',type=float);p.add_argument('--y',type=float);a=p.parse_args()
if (a.x is None)!=(a.y is None):p.error('--x and --y must be supplied together')
rospy.init_node('ten_cli',anonymous=True)
if a.op=='status':print(rospy.wait_for_message('/swarm10/status',String,timeout=5).data)
else:
 pub=rospy.Publisher('/swarm10/command',String,queue_size=1)
 deadline=rospy.Time.now().to_sec()+5
 while not pub.get_num_connections() and rospy.Time.now().to_sec()<deadline:rospy.sleep(.1)
 if not pub.get_num_connections():raise SystemExit('Ten coordinator not running')
 data=dict(op=a.op,altitude=a.altitude,distance=a.distance)
 if a.x is not None:data.update(x=a.x,y=a.y)
 pub.publish(String(json.dumps(data)));rospy.sleep(.7)
 print(rospy.wait_for_message('/swarm10/status',String,timeout=5).data)
