#!/usr/bin/env python3
"""Independent scene sensor; raw odometry is its only live input.

No planner or native local-map imports. Ground publication has a separate ROS
timer, so incomplete peers and expensive raycasts cannot gate ground samples.
"""
import hashlib
import json
import os
from pathlib import Path
import sys
import threading
import time
import numpy as np
from visual_transport import SnapshotMailbox

CAMERAS=('front','up','down')
# Symmetric ROS-body enclosure of every original TrackerDroneMesh vertex.
# TrackerField installs that mesh with identity rotation, scale 1 and no offset.
PEER_HALF_EXTENTS=np.array([.2463711053133011,.24637119472026825,.0970769003033638])
MAX_POSE_AGE=.1
MAX_PEER_SKEW=.1

def preview_enabled(drone_id):return drone_id in (2,7)

def aligned_peers(states,own_id,count):
    stamp=states[own_id]['stamp'];result=[]
    for identity in range(count):
        if identity==own_id:continue
        peer=states.get(identity)
        if peer is None:raise ValueError('missing peer odometry: %d'%identity)
        dt=stamp-peer['stamp']
        if not np.isfinite(dt) or abs(dt)>MAX_PEER_SKEW:raise ValueError('peer odometry skew: %d'%identity)
        p=np.asarray(peer['position'],float);v=np.asarray(peer['velocity'],float);rotation=np.asarray(peer['rotation'],float)
        if p.shape!=(3,) or v.shape!=(3,) or rotation.shape!=(3,3) or not np.isfinite(np.r_[p,v,rotation.ravel()]).all():raise ValueError('invalid peer pose')
        # Enclose the full possible displacement since its real sample. The
        # scalar expansion remains conservative for any body orientation.
        inflation=np.linalg.norm(v)*abs(dt)
        result.append(dict(id=identity,position=p+v*dt,rotation=rotation,half_extents=PEER_HALF_EXTENTS+inflation))
    return result

def build_frame(scene,states,own_id,count,map_hash,sequence,now,monotonic=time.monotonic):
    """Render one exact pose snapshot; no clock can rewrite its sample stamp."""
    from cloud_depth_geometry import camera_rotation,PreparedPeers
    start=monotonic();own=states.get(own_id)
    if own is None or not 0<=now-own['stamp']<=MAX_POSE_AGE:raise ValueError('own odometry missing/stale/future')
    peers=PreparedPeers(aligned_peers(states,own_id,count));images={};rotations={};stats={};peer_masks={}
    for camera in CAMERAS:
        images[camera]=scene.render(own['position'],own['rotation'],camera,peers=peers)
        peer_masks[camera]=getattr(scene,'peer_mask',np.zeros(images[camera].shape,bool)).copy()
        rotations[camera]=camera_rotation(own['rotation'],camera);stats[camera]=dict(scene.stats)
    finish=monotonic()
    if finish-start>.3 or now-own['stamp']+finish-start>.6:raise ValueError('render expired')
    skew=max((abs(own['stamp']-state['stamp']) for i,state in states.items() if i!=own_id),default=0.)
    return dict(map_hash=map_hash,own_id=own_id,sequence=sequence,stamp=own['stamp'],started=start,finished=finish,
                position=own['position'],rotations=rotations,images=images,peer_masks=peer_masks,stats=stats,peer_max_skew=skew)

def load_world(point_path,expected_hash):
    path=Path(point_path);config=json.loads(path.read_text(encoding='utf-8-sig'))
    directory=Path(config['paths']['map'])
    if not directory.is_absolute():directory=path.parent/directory
    data={}
    for key in ('world','metadata'):
        raw=(directory/config['paths']['map_files'][key]).read_bytes();digest=hashlib.sha256(raw).hexdigest()
        if digest!=config['map_fingerprints'][key] or (key=='world' and digest!=expected_hash):raise ValueError('map fingerprint mismatch: '+key)
        data[key]=raw
    return json.loads(data['world'])

class GroundSensor:
    """Vectorized TerrainSampler.Height with local-ground unknown boundaries."""
    def __init__(self,world):
        self.bounds=world['bounds'];terrain=world['terrain'];self.width=int(terrain['width']);self.rows=int(terrain['height'])
        self.heights=np.asarray(terrain['heights'],np.float32).reshape(self.rows,self.width);self.origin=None;self.points=None
    def height(self,xy):
        xy=np.asarray(xy,np.float32);b=self.bounds
        known=(xy[:,0]>=b['x_min'])&(xy[:,0]<=b['x_max'])&(xy[:,1]>=b['y_min'])&(xy[:,1]<=b['y_max'])
        u=np.clip((xy[:,0]-np.float32(b['x_min']))/np.float32(b['x_max']-b['x_min'])*np.float32(self.width-1),0,self.width-1)
        v=np.clip((np.float32(b['y_max'])-xy[:,1])/np.float32(b['y_max']-b['y_min'])*np.float32(self.rows-1),0,self.rows-1)
        c=np.minimum(np.floor(u).astype(int),self.width-2);r=np.minimum(np.floor(v).astype(int),self.rows-2)
        u-=c.astype(np.float32);v-=r.astype(np.float32)
        a=self.heights[r,c];bb=self.heights[r,c+1];d=self.heights[r+1,c];e=self.heights[r+1,c+1]
        height=np.where(u+v<=1,a+(bb-a)*u+(d-a)*v,e+(d-e)*(1-u)+(bb-e)*(1-v))
        return np.where(known,height,np.nan).astype(np.float32)
    def sample(self,position,velocity):
        lead=np.asarray(velocity[:2],np.float32)*2;length=np.linalg.norm(lead)
        if length>10:lead*=10/length
        origin=np.floor((np.asarray(position[:2],np.float32)+lead)/.5)*.5-30
        if self.origin is None or not np.array_equal(origin,self.origin):
            x,y=np.meshgrid(origin[0]+np.arange(121,dtype=np.float32)*.5,origin[1]+np.arange(121,dtype=np.float32)*.5)
            xy=np.c_[x.ravel(),y.ravel()];self.points=np.c_[xy,self.height(xy)].astype(np.float32);self.origin=origin
        return self.points

def main():
    root,identity,count,parent,expected=sys.argv[1:];identity=int(identity);count=int(count);parent=int(parent)
    import rospy
    from scipy.spatial.transform import Rotation
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PointStamped,Point
    from sensor_msgs.msg import Image,PointCloud2,PointField
    from std_msgs.msg import Header,String
    from cloud_depth_geometry import CloudDepthScene
    rospy.init_node('cloud_depth_%d'%identity,anonymous=False)
    # Validate the actual player's mode before registering any ground producer.
    # A capable player may have been started in unity mode in another terminal.
    from depth_source import validate_scene_source
    scene_status=json.loads(rospy.wait_for_message('/scene/status',String,timeout=15).data)
    validate_scene_source(scene_status,'pointcloud',count)
    world=load_world(('__UAV_PROJECT__/config/point.json'.replace("__UAV_PROJECT__", __import__("os").environ["UAV_PROJECT_ROOT"])),expected);ground=GroundSensor(world)
    states={};lock=threading.Lock();ground_status={};namespace='/drone_%d/'%identity
    ground_pub=rospy.Publisher(namespace+'ground',PointStamped,queue_size=1)
    local_pub=rospy.Publisher(namespace+'local_ground',PointCloud2,queue_size=1)
    preview=rospy.Publisher(namespace+'depth',Image,queue_size=1) if preview_enabled(identity) else None
    fields=[PointField('x',0,7,1),PointField('y',4,7,1),PointField('z',8,7,1)]
    def odom(msg,i):
        p=msg.pose.pose.position;v=msg.twist.twist.linear;q=msg.pose.pose.orientation
        try:
            state=dict(stamp=msg.header.stamp.to_sec(),position=np.array([p.x,p.y,p.z]),velocity=np.array([v.x,v.y,v.z]),rotation=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix())
            if not np.isfinite(np.r_[state['stamp'],state['position'],state['velocity']]).all():return
        except ValueError:return
        with lock:
            if i not in states or state['stamp']>states[i]['stamp']:states[i]=state
    subscriptions=[rospy.Subscriber('/drone_%d/odom'%i,Odometry,odom,callback_args=i,queue_size=1,tcp_nodelay=True) for i in range(count)]
    def publish_ground(_):
        with lock:own=states.get(identity)
        if own is None or not 0<=rospy.Time.now().to_sec()-own['stamp']<=MAX_POSE_AGE:return
        header=Header(stamp=rospy.Time.from_sec(own['stamp']),frame_id='world');p=own['position']
        points=ground.sample(p,own['velocity']);height=float(ground.height(p[None,:2])[0])
        ground_pub.publish(PointStamped(header=header,point=Point(p[0],p[1],height)))
        local_pub.publish(PointCloud2(header=header,height=121,width=121,fields=fields,is_bigendian=False,point_step=12,row_step=121*12,data=points.tobytes(),is_dense=False))
        ground_status.update(stamp=own['stamp'],published=rospy.Time.now().to_sec())
    timer=rospy.Timer(rospy.Duration(.1),publish_ground)
    try:
        # Depth-only CPU triangle projection preserves the reference geometry
        # while leaving CPU headroom for the ten control/planning processes.
        scene=CloudDepthScene(world,backend='raster');sequence=0;last_stamp=-float('inf');deadline=time.monotonic()
        with SnapshotMailbox(root+'/frames') as output,SnapshotMailbox(root+'/health') as health:
            while not rospy.is_shutdown() and os.getppid()==parent:
                start=time.monotonic();status=dict(state='running',stamp=rospy.Time.now().to_sec(),ground=dict(ground_status),sequence=sequence)
                with lock:snapshot=dict(states)
                own=snapshot.get(identity)
                try:
                    if own is None or not 0<=status['stamp']-own['stamp']<=MAX_POSE_AGE:raise ValueError('own odometry missing/stale/future')
                    if own['stamp']<=last_stamp:raise ValueError('no new own pose')
                    value=build_frame(scene,snapshot,identity,count,expected,sequence+1,status['stamp'])
                    if not 0<=rospy.Time.now().to_sec()-own['stamp']<=.6:raise ValueError('render expired')
                    sequence=value['sequence']
                    status.update(sequence=sequence,frame_stamp=value['stamp'],published=output.put(value),render_ms=(value['finished']-value['started'])*1000,
                                  peer_max_skew=value['peer_max_skew'],stats=value['stats']);last_stamp=own['stamp']
                    if preview is not None:
                        header=Header(stamp=rospy.Time.from_sec(own['stamp']),frame_id=namespace.strip('/')+'/camera_frame')
                        preview.publish(Image(header=header,height=270,width=480,encoding='32FC1',is_bigendian=0,step=1920,data=value['images']['front'].tobytes()))
                except ValueError as exc:status['error']=str(exc)
                health.put(status)
                deadline+=.1;remaining=deadline-time.monotonic()
                if remaining>0:time.sleep(remaining)
                else:deadline=time.monotonic()
    finally:timer.shutdown()

if __name__=='__main__':main()
