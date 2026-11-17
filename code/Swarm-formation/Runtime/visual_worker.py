#!/usr/bin/env python3
"""Display-only ROS process. No mission/native-map imports or control output."""
import os
import sys
import time
import numpy as np
import rospy
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Point,PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from cloud_serialization import xyz32
from visualization_msgs.msg import Marker,MarkerArray
from visual_transport import SnapshotMailbox,PathJournalReader


class Renderer:
    def __init__(self,drone_id,count):
        self.id=drone_id;self.count=count;self.ns='/drone_'+str(drone_id)
        self.path_entries=[]
        self.marker_pub=rospy.Publisher(self.ns+'/mission/markers',MarkerArray,queue_size=1)
        self.map_pub=rospy.Publisher('/display'+self.ns+'/local_map',PointCloud2,queue_size=1)
        self.path_pub=rospy.Publisher(self.ns+'/executed_path',Path,queue_size=1,latch=True)
        self.plan_pub=rospy.Publisher(self.ns+'/planned_path',Path,queue_size=1)
        if count==10:self.raw_pub=rospy.Publisher('/display'+self.ns+'/unity_points',PointCloud2,queue_size=1)

    def publish_visuals(self,view):
        from std_msgs.msg import Header
        header=Header(stamp=rospy.Time.from_sec(view.stamp),frame_id='world')
        ids=np.argwhere(view.map.state==view.map.OCCUPIED)
        points=(ids+view.map.origin+.5)*view.map.resolution
        if self.count==10:
            from ten_core import crop
            from ten_cloud import publish_raw
            points=crop(points,view.p);publish_raw(self,header,frames=view.frames)
        self.map_pub.publish(xyz32(header,points[::max(1,len(points)//40000)]))
        path=Path();path.header=header
        for entry in self.path_entries:
            ps=PoseStamped();ps.header=Header(stamp=rospy.Time.from_sec(entry['stamp']),frame_id='world')
            ps.pose.position=Point(*entry['p']);ps.pose.orientation.w=1.;path.poses.append(ps)
        self.path_pub.publish(path)
        plan=Path();plan.header=header
        for p in view.chosen:
            ps=PoseStamped();ps.header=header;ps.pose.position=Point(*map(float,p));ps.pose.orientation.w=1;plan.poses.append(ps)
        self.plan_pub.publish(plan)
        markers=MarkerArray();m=Marker();m.header=header;m.ns=self.ns;m.id=0;m.type=Marker.TEXT_VIEW_FACING;m.action=Marker.ADD
        m.pose.position=Point(*(view.p+np.array([0,0,3])));m.pose.orientation.w=1;m.scale.z=1.2;m.color.a=1;m.color.r=1;m.color.g=1;m.color.b=1
        m.text='%s %s\nABS %.2f AGL %.2f\nv %.2f m/s\n%s'%(self.ns,view.phase,view.p[2],view.p[2]-view.ground,np.linalg.norm(view.v),view.reason)
        markers.markers.append(m)
        model=Marker();model.header=header;model.ns=self.ns;model.id=1;model.type=Marker.MESH_RESOURCE;model.action=Marker.ADD
        model.mesh_resource='package://so3_quadrotor_simulator/config/uav.dae';model.mesh_use_embedded_materials=True
        # IPC arrays deliberately reference immutable bytes. Older SciPy's
        # Cython Rotation reader requires a writable buffer even though this
        # operation is read-only. Copy only this 3x3 matrix in the child.
        model.pose.position=Point(*map(float,view.p));q=Rotation.from_matrix(view.rot.copy()).as_quat()
        model.pose.orientation.x,model.pose.orientation.y,model.pose.orientation.z,model.pose.orientation.w=map(float,q)
        model.scale.x=model.scale.y=model.scale.z=2. if self.count==10 else 1.
        model.color.r=model.color.g=model.color.b=model.color.a=1.;markers.markers.append(model)
        arrow=Marker();arrow.header=header;arrow.ns=self.ns;arrow.id=2;arrow.type=Marker.ARROW;arrow.action=Marker.ADD
        arrow.pose.orientation.w=1.
        arrow.points=[Point(*map(float,view.p)),Point(*map(float,view.p+view.v))];arrow.scale.x=.12;arrow.scale.y=.24;arrow.scale.z=.35
        arrow.color.g=1.;arrow.color.b=1.;arrow.color.a=1.;markers.markers.append(arrow)
        goal=Marker();goal.header=header;goal.ns=self.ns;goal.id=3;goal.type=Marker.SPHERE;goal.action=Marker.DELETE if self.count==10 else Marker.ADD
        goal.pose.position=Point(*map(float,view.goal));goal.pose.orientation.w=1.;goal.scale.x=goal.scale.y=goal.scale.z=.8
        goal.color.r=1.;goal.color.g=.6;goal.color.a=1.;markers.markers.append(goal)
        for index,points in enumerate(view.last_candidates):
            candidate=Marker();candidate.header=header;candidate.ns=self.ns+'/candidates';candidate.id=index;candidate.type=Marker.LINE_STRIP;candidate.action=Marker.ADD
            candidate.pose.orientation.w=1.;candidate.scale.x=.05;candidate.color.r=.5;candidate.color.g=.7;candidate.color.b=1.;candidate.color.a=.4
            candidate.points=[Point(*map(float,point)) for point in points];candidate.lifetime=rospy.Duration(1.5);markers.markers.append(candidate)
        if self.count==2 and view.peer:
            pair=Marker();pair.header=header;pair.ns='pair';pair.id=self.id;pair.type=Marker.LINE_STRIP;pair.action=Marker.ADD
            pair.pose.orientation.w=1.;pair.scale.x=.15;pair.color.r=1.;pair.color.g=.6;pair.color.a=1.
            pair.points=[Point(*map(float,view.p)),Point(*view.peer['position'])];markers.markers.append(pair)
            m.text+='\n3D separation %.2f m (%s)'%(view.distance or 0.,'active 30..80' if view.band else 'phase exemption')
        self.marker_pub.publish(markers)

def main():
    root,drone_id,count,parent=sys.argv[1:5]
    drone_id,count,parent=int(drone_id),int(count),int(parent)
    # Strip inherited mission ROS remappings: this process owns a distinct node.
    rospy.init_node('mission_display_'+str(drone_id),argv=[sys.argv[0]],anonymous=False)
    renderer=Renderer(drone_id,count)
    journal=PathJournalReader(root+'/path',unlimited=count==10 and drone_id in (2,7))
    try:
        with SnapshotMailbox(root+'/view') as mailbox:
            while not rospy.is_shutdown() and os.getppid()==parent:
                view=mailbox.take()
                if view is None:time.sleep(.02);continue
                renderer.path_entries=journal.read_through(view.sequence)
                try:renderer.publish_visuals(view)
                except Exception as exc:rospy.logerr_throttle(1,'Display error: %s',repr(exc))
    finally:journal.close()


if __name__=='__main__':main()
