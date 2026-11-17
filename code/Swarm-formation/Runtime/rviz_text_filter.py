#!/usr/bin/env python3
"""Presentation-only relay: clear world text, preserve geometry and paths."""
import copy
import rospy
from visualization_msgs.msg import Marker,MarkerArray

def without_world_text(message):
    result=MarkerArray()
    for marker in message.markers:
        if marker.type==Marker.TEXT_VIEW_FACING:
            marker=copy.copy(marker);marker.action=Marker.DELETE
        result.markers.append(marker)
    return result

if __name__=='__main__':
    rospy.init_node('rviz_text_filter')
    pubs=[];subs=[]
    for source in ['/drone_%d/mission/markers'%i for i in range(10)]+['/swarm10/markers']:
        pub=rospy.Publisher('/corner_view'+source,MarkerArray,queue_size=1);pubs.append(pub)
        subs.append(rospy.Subscriber(source,MarkerArray,lambda m,p=pub:p.publish(without_world_text(m)),queue_size=1))
    rospy.spin()
