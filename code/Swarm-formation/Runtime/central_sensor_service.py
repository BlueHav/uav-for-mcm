#!/usr/bin/env python3
"""Single sensor ingress and fair distribution. Map terrain is display only.

Unity lossless depth is supported here. The existing pointcloud depth producer
remains phase-owned by mission_node; it is not represented as a verified source.
Odometry/contact/controller feedback do not pass through this service.
"""
import json
import math
import os
import re
import socket
import threading
import time
import numpy as np
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header, String
from central_sensor_core import IsolatedFrameReader, LatestScheduler
from depth_transport import HEADER, ProtocolError
from sensor_channels import (GROUP_COUNT, group_cloud_raw, group_cloud_topic,
                             group_ground_raw, group_ground_topic)

CAMERAS=('front','up','down')


def bind_ingress(port=10013):
    # Bind before ROS registration: a duplicate must not evict the running node.
    server=socket.socket()
    try:
        server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        server.bind(('127.0.0.1',port));server.listen(1);server.settimeout(.5)
        return server
    except Exception:
        server.close();raise


class CentralSensorService:
    def __init__(self,server):
        self.server=server
        self.depth_source=os.environ.get('YOPO_DEPTH_SOURCE','unity')
        self.depth_enabled=self.depth_source=='unity'
        thresholds={('depth',d,c):.6 for d in range(10) for c in range(3)} if self.depth_enabled else {}
        if self.depth_enabled:thresholds.update({(kind,d):.6 for d in range(10) for kind in ('ground','local_ground')})
        thresholds.update({('terrain',name):6. for name in ('near','far','status')})
        # One leader cloud per group is reused by all five members. The cloud is
        # sampled at 3 Hz; a 1.5 s window only guards against a dead producer.
        thresholds.update({('group_ground',g):.6 for g in range(GROUP_COUNT)})
        thresholds.update({('group_cloud',g):1.5 for g in range(GROUP_COUNT)})
        self.scheduler=LatestScheduler(thresholds)
        self.wake=threading.Event();self.stop=threading.Event()
        self.stats_lock=threading.Lock();self.connection_lock=threading.RLock()
        self.connected=False;self.connections=0;self.errors=0;self.generation=0
        self.counts=[0]*30;self.wire_bytes=0;self.raw_bytes=0
        self.max_age=0.;self.max_publish_ms=0.;self.last_report=time.monotonic()
        self.pubs={};self.subscriptions=[]
        for d in (range(10) if self.depth_enabled else ()):
            for c,name in enumerate(CAMERAS):
                names=('depth','depth_camera_pose','camera_info') if c==0 else (name+'_depth',name+'_camera_pose',name+'_camera_info')
                self.pubs['depth',d,c]=tuple(rospy.Publisher('/drone_%d/%s'%(d,n),kind,queue_size=1,tcp_nodelay=True)
                                            for n,kind in zip(names,(Image,PoseStamped,CameraInfo)))
            for kind,msg_type in (('ground',PointStamped),('local_ground',PointCloud2)):
                key=(kind,d)
                self.pubs[key]=rospy.Publisher('/drone_%d/%s'%(d,kind),msg_type,queue_size=1,tcp_nodelay=True)
                self.subscriptions.append(rospy.Subscriber('/sensors/raw/drone_%d/%s'%(d,kind),msg_type,self.receive_ros,
                                                           callback_args=key,queue_size=1,buff_size=4000000,tcp_nodelay=True))
        if self.depth_enabled:
            for group in range(GROUP_COUNT):
                for kind,msg_type,topic in (('group_ground',PointStamped,group_ground_topic(group)),
                                            ('group_cloud',PointCloud2,group_cloud_topic(group))):
                    key=(kind,group)
                    self.pubs[key]=rospy.Publisher(topic,msg_type,queue_size=1,latch=True,tcp_nodelay=True)
                    raw=group_ground_raw(group) if kind=='group_ground' else group_cloud_raw(group)
                    self.subscriptions.append(rospy.Subscriber(raw,msg_type,self.receive_ros,
                                                               callback_args=key,queue_size=1,buff_size=4000000,tcp_nodelay=True))
        for name in ('near','far','status'):
            key=('terrain',name);kind=String if name=='status' else PointCloud2
            suffix='status' if name=='status' else 'terrain_'+name
            self.pubs[key]=rospy.Publisher('/swarm10/display/'+suffix,kind,queue_size=1,latch=True)
            self.subscriptions.append(rospy.Subscriber('/sensors/raw/swarm10/display/'+suffix,kind,self.receive_ros,
                                                       callback_args=key,queue_size=1,buff_size=16000000))
        self.health=rospy.Publisher('/sensors/status',String,queue_size=1,latch=True)
        self.compat_health=rospy.Publisher('/scene/depth_transport',String,queue_size=1)
        self.timer=rospy.Timer(rospy.Duration(1.),self.report)
        # Tiny, freshness-critical ground heights bypass the shared dispatch
        # queue: a 175 KB cloud must never delay a 0.2 KB height sample.
        self.ground_latest={};self.ground_published={}
        self.ground_thread=threading.Thread(target=self.dispatch_ground,name='ground-fast',daemon=True)
        self.reader_thread=threading.Thread(target=self.receive_depth,name='sensor-ingress',daemon=True)
        self.dispatch_thread=threading.Thread(target=self.dispatch,name='sensor-dispatch',daemon=True)
        rospy.on_shutdown(self.close)

    def start(self):
        self.reader_thread.start();self.dispatch_thread.start();self.ground_thread.start()
        rospy.loginfo('Central sensor service ready; Unity depth=%s; raw ground and display routing enabled',self.depth_enabled)

    def receive_ros(self,msg,key):
        try:
            if key==('terrain','status'):
                data=json.loads(msg.data)
                if not isinstance(data,dict) or len(msg.data)>65536:raise ValueError('terrain status shape')
                if data.get('display_only') is not True or data.get('source_kind')!='map_terrain_sample_not_radar':
                    raise ValueError('terrain provenance')
                stamp=float(data['stamp'])
            else:
                stamp=msg.header.stamp.to_sec()
                ids=re.findall(r'(?:^|/)drone_(\d+)(?:/|$)',msg.header.frame_id)
                if key[0] in ('depth','ground','local_ground') and any(int(v)!=key[1] for v in ids):raise ValueError('drone identity')
                if key[0] in ('ground','group_ground'):
                    if not all(math.isfinite(x) for x in (msg.point.x,msg.point.y,msg.point.z)):raise ValueError('ground values')
                else:
                    limit=16000000 if key[0]=='terrain' else 4000000
                    if (len(msg.data)>limit or msg.height<1 or msg.point_step<=0 or msg.width<0
                        or msg.row_step<msg.width*msg.point_step or len(msg.data)!=msg.height*msg.row_step):
                        raise ValueError('cloud shape/bounds')
                    fields={f.name:f for f in msg.fields}
                    if any(n not in fields or fields[n].datatype!=7 or fields[n].count!=1
                           or not 0<=fields[n].offset<=msg.point_step-4 for n in ('x','y','z')):
                        raise ValueError('cloud xyz fields')
                    if key[0] in ('local_ground','group_cloud') and (msg.header.frame_id!='world' or msg.is_bigendian!=0
                        or msg.point_step!=12 or msg.row_step!=msg.width*12
                        or any(fields[n].offset!=i*4 for i,n in enumerate(('x','y','z')))):
                        raise ValueError('local_ground Unity xyz32 layout')
                    for name in ('x','y','z'):
                        values=np.ndarray((msg.height,msg.width),dtype='>f4' if msg.is_bigendian else '<f4',
                                          buffer=msg.data,offset=fields[name].offset,
                                          strides=(msg.row_step,msg.point_step)) if msg.width else np.empty(0)
                        if key[0] in ('local_ground','group_cloud') and name=='z':
                            if np.isinf(values).any() or (np.isnan(values).any() and msg.is_dense):
                                raise ValueError('ground unknown values require is_dense=false')
                        elif not np.isfinite(values).all():raise ValueError('cloud xyz values')
                    if key[0]=='terrain' and msg.header.frame_id!='world':raise ValueError('terrain frame')
            if key[0]=='ground':
             self.ground_latest[key]=(msg,stamp);return
            if self.scheduler.submit(key,msg,stamp,rospy.Time.now().to_sec(),time.monotonic()):self.wake.set()
        except (ValueError,TypeError,KeyError,OverflowError) as exc:
            self.scheduler.reject(key,str(exc))

    def receive_depth(self):
        while not self.stop.is_set() and not rospy.is_shutdown():
            try:conn,_=self.server.accept()
            except socket.timeout:continue
            except OSError:break
            if not self.depth_enabled:
                conn.close();continue
            with self.connection_lock:
                self.generation+=1
                self.scheduler.clear('depth')
                with self.stats_lock:self.connections+=1;self.connected=True
            try:
                with conn:
                    conn.settimeout(.5);conn.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
                    reader=IsolatedFrameReader()
                    while not self.stop.is_set() and not rospy.is_shutdown():
                        frame,key,reason=reader.read(conn,now=lambda:rospy.Time.now().to_sec())
                        if frame is None:
                            if key is not None:self.scheduler.reject(key,reason)
                            with self.stats_lock:self.errors+=1
                            continue
                        h=frame.header
                        if self.scheduler.submit(key,(self.generation,frame),h.sec+h.nsec*1e-9,rospy.Time.now().to_sec(),time.monotonic(),h.sequence):
                            self.wake.set()
            except (EOFError,OSError,ProtocolError) as exc:
                with self.stats_lock:self.errors+=1
                rospy.logwarn_throttle(2.,'Sensor depth connection reset: %s',str(exc))
            except Exception as exc:
                with self.stats_lock:self.errors+=1
                rospy.logerr_throttle(2.,'Sensor depth ingress error: %s',str(exc))
            finally:
                with self.connection_lock:
                    self.generation+=1
                    with self.stats_lock:self.connected=False
                    self.scheduler.clear('depth')

    def publish_depth(self,key,frame):
        h=frame.header;name=CAMERAS[h.camera]
        header=Header(seq=h.sequence & 0xffffffff,stamp=rospy.Time(h.sec,h.nsec),
                      frame_id='drone_%d/%s'%(h.drone,'camera_frame' if h.camera==0 else name+'_camera_frame'))
        pose=PoseStamped(header=Header(seq=header.seq,stamp=header.stamp,frame_id='world'))
        pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=h.pose[:3]
        pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w=h.pose[3:]
        image=Image(header=header,width=h.width,height=h.height,encoding='32FC1',is_bigendian=0,step=h.width*4,data=frame.data)
        # Wire v1 calibration is fixed by the Unity capture contract.
        focal=240. if h.camera==0 else 77.94228634059948
        info=CameraInfo(header=header,width=h.width,height=h.height,distortion_model='plumb_bob',
                        K=[focal,0,240,0,focal,135,0,0,1],R=[1,0,0,0,1,0,0,0,1],
                        P=[focal,0,240,0,0,focal,135,0,0,0,1,0])
        image_pub,pose_pub,info_pub=self.pubs[key]
        pose_pub.publish(pose);image_pub.publish(image);info_pub.publish(info)
        with self.stats_lock:
            self.counts[h.drone*3+h.camera]+=1
            self.wire_bytes+=HEADER.size+h.payload_length;self.raw_bytes+=h.raw_length
            self.max_age=max(self.max_age,rospy.Time.now().to_sec()-header.stamp.to_sec())

    def dispatch_ground(self):
        """Publish the newest ground height for each drone at up to 50 Hz."""
        while not self.stop.is_set() and not rospy.is_shutdown():
            try:
                for key,(msg,stamp) in list(self.ground_latest.items()):
                    if self.ground_published.get(key)==stamp:continue
                    self.pubs[key].publish(msg)
                    self.ground_published[key]=stamp
            except Exception as exc:
                with self.stats_lock:self.errors+=1
                rospy.logerr_throttle(2.,'Ground fast path failed: %s',repr(exc))
            self.wake.wait(.01)

    def dispatch(self):
        while not self.stop.is_set() and not rospy.is_shutdown():
            self.wake.clear()
            # Serial dispatch across streams is round robin. ROS publishers have
            # independent queue_size=1 async queues, so one slow subscriber cannot
            # hold the network ingress or allocate an unbounded backlog.
            item=self.scheduler.take(rospy.Time.now().to_sec(),time.monotonic())
            if item is None:self.wake.wait(.01);continue
            key,value=item;started=time.monotonic()
            try:
                if key[0]=='depth':
                    # Serialize with disconnect invalidation; an already taken old
                    # value cannot escape after a reconnect cleared the slots.
                    with self.connection_lock:
                        generation,frame=value
                        if not self.connected or generation!=self.generation:continue
                        self.publish_depth(key,frame)
                else:self.pubs[key].publish(value)
                elapsed=(time.monotonic()-started)*1000.
                self.scheduler.published(key,time.monotonic(),elapsed)
                if key[0]=='depth':
                    with self.stats_lock:self.max_publish_ms=max(self.max_publish_ms,elapsed)
            except Exception as exc:
                self.scheduler.reject(key,'publish: '+str(exc))
                with self.stats_lock:self.errors+=1
                rospy.logerr_throttle(2.,'Sensor dispatch failed for %s: %s',str(key),str(exc))

    def report(self,_):
        mono=time.monotonic();now=rospy.Time.now().to_sec()
        streams=self.scheduler.status(now,mono)
        with self.stats_lock:
            compat=dict(window_s=mono-self.last_report,frames=self.counts[:],wire_bytes=self.wire_bytes,
                        raw_bytes=self.raw_bytes,max_age_s=self.max_age,max_publish_ms=self.max_publish_ms,
                        errors=self.errors,connections=self.connections,connected=self.connected)
            self.counts=[0]*30;self.wire_bytes=self.raw_bytes=0;self.max_age=self.max_publish_ms=0.;self.last_report=mono
        status=dict(stamp=now,service='central_sensor_service',depth_source=self.depth_source,
                    supported_depth_source='unity',unity_depth_enabled=self.depth_enabled,
                    pointcloud_depth='existing_mission_owned_compatibility_path',
                    ground_source='unity_position_samples' if self.depth_enabled else 'existing_mission_owned_bypass',terrain_source_kind='map_terrain_sample_not_radar',
                    terrain_display_only=True,control_local_map_source='per_drone_samples_only' if self.depth_enabled else 'existing_mission_owned_bypass',
                    queue_capacity_per_stream=1,streams=streams,depth_transport=compat,
                    fresh_depth_streams=sum(s['fresh'] for s in streams if s['stream'][0]=='depth'),
                    dropped=sum(s['dropped'] for s in streams),queue=sum(s['queue'] for s in streams))
        self.compat_health.publish(String(data=json.dumps(compat)))
        self.health.publish(String(data=json.dumps(status)))

    def close(self):
        self.stop.set();self.wake.set()
        self.server.close()


def main():
    # Alternate port exists only for isolated server-side protocol tests.
    server=bind_ingress(int(os.environ.get('YOPO_SENSOR_TEST_PORT','10013')))
    try:
        rospy.init_node('central_sensor_service')
        service=CentralSensorService(server);service.start();rospy.spin()
    finally:server.close()


if __name__=='__main__':main()
