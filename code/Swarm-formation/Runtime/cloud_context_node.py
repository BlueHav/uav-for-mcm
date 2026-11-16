"""Clip the existing terrain point source; no image/raycast sensor is created."""
import json,time
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2,PointField
from std_msgs.msg import Header,String
from point_mission_core import PointMission
from cloud_context import gradient_bound
from cloud_context_sensor import TerrainContextCache,context_observations
from cloud_config import CONTEXT_SPAN,CONTEXT_RESOLUTION
from sensor_channels import CONTEXT_RATE_HZ,GROUP_LEADERS

def main():
    rospy.init_node('cloud_context');model=PointMission(('__UAV_PROJECT__/config/point.json'.replace("__UAV_PROJECT__", __import__("os").environ["UAV_PROJECT_ROOT"])));slope=gradient_bound(model.world)
    # Keep the source height array cached; all generated observations remain
    # bounded crops around fresh aircraft poses, independent of RViz displays.
    model.world['terrain']['heights']=np.asarray(model.world['terrain']['heights'],float)
    cache=TerrainContextCache(model.world,slope)
    states={}
    # Source identity belongs in the topic; ROS replaces Header.seq on publish.
    pubs=[rospy.Publisher('/drone_%d/terrain_context/'%i+model.hashes['world'],PointCloud2,queue_size=1) for i in range(10)]
    info=rospy.Publisher('/swarm10/cloud_context/source',String,queue_size=1,latch=True)
    info.publish(String(json.dumps(dict(hashes=model.hashes,max_gradient=slope,span_m=CONTEXT_SPAN,resolution_m=CONTEXT_RESOLUTION,height_kind='conservative_terrain_upper_bound'))))
    def odom(msg,i):
        p=msg.pose.pose.position;states[i]=(msg.header.stamp.to_sec(),[p.x,p.y,p.z])
    for i in range(10):rospy.Subscriber('/drone_%d/odom'%i,Odometry,odom,callback_args=i,queue_size=1)
    fields=[PointField('x',0,7,1),PointField('y',4,7,1),PointField('z',8,7,1)]
    # Only the two group leaders are sampled and published; every group member
    # subscribes to its leader's topic and reuses that conservative crop. The
    # 5 Hz source still leaves a full 0.2 s margin under the 0.6 s consumer gate.
    rate=rospy.Rate(CONTEXT_RATE_HZ)
    while not rospy.is_shutdown():
        for i,stamp,points in context_observations(states,cache,lambda:rospy.Time.now().to_sec(),ids=GROUP_LEADERS):
            header=Header(stamp=rospy.Time.from_sec(stamp),frame_id='world')
            pubs[i].publish(PointCloud2(header=header,height=1,width=len(points),fields=fields,is_bigendian=False,point_step=12,row_step=12*len(points),data=points.tobytes(),is_dense=False))
        rate.sleep()

if __name__=='__main__':main()
