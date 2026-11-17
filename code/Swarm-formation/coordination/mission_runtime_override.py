#!/usr/bin/env python3
"""Sensor-only YOPO mission node. One process and final command writer per drone.

Full scene data must never be imported here. Ground comes only from current
Local sensor measurements; obstacles only from the three depth cameras.
"""
import portable_paths
import os, sys, json, time, math, threading, collections
import faulthandler, signal
import numpy as np
import cv2, torch, rospy
from scipy.spatial.transform import Rotation
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from quadrotor_msgs.msg import PositionCommand
from config.config import cfg
from test_yopo_ros import YopoNet
from policy.yopo_network import YopoNetwork
from policy.state_transform import StateTransform
from policy.primitive import LatticePrimitive
from local_map import RollingLocalMap,_carve,visible_detour_waypoint,route_corridor_clear
from sensor_inbox import LatestMeasurement
from inference_device import select_device
from mission_core import *


class Mission:
    process_output=YopoNet.process_output

    def __init__(self):
        rospy.init_node('mission',anonymous=False)
        self.id=int(rospy.get_param('~drone_id',0));self.count=int(rospy.get_param('~count',1))
        from depth_source import depth_source
        self.depth_source=depth_source();self.cloud_depth=None;self.cloud_preview_stamps={}
        self.cloud_mission=self.count==10 and os.environ.get('YOPO_CLOUD_MISSION')=='1'
        self.planning_mode='CLOUD' if self.cloud_mission else 'YOPO'
        self.planner_epoch=0;self.image_callbacks=0;self.image_received=0;self.preview_images={};self.yopo_ready=False;self.image_subscriptions=[]
        if self.count==10 and _carve is None:raise RuntimeError('Ten-agent mode requires ray_carve.so; run setup.sh in the project root')
        if self.count==10:rospy.loginfo('Native sensor ray carving loaded')
        self.ns='/drone_'+str(self.id);self.lock=threading.RLock();self.maplock=threading.RLock()
        from yopo_input_preview import YopoInputPreview
        self.input_preview=YopoInputPreview(self.id)
        self.p=None;self.v=np.zeros(3);self.rot=np.eye(3);self.odom_stamp=0.;self.ground_stamp=0.;self.ground=float('nan')
        self.frames={};self.camera_pose={};self.pending_depth={};self.ground_cloud=LatestMeasurement();self.context_cloud=LatestMeasurement();self.context_source_ok=False;self.contact=False;self.armed=False
        self.phase='IDLE';self.mode='ABS';self.height=150.;self.speed=10.5;self.reason='Ground idle; motors disarmed'
        self.goal=np.array([45.,-50.,150.]);self.hold=None;self.refp=None;self.refv=np.zeros(3);self.refa=np.zeros(3)
        self.desired=np.zeros(3);self.yaw=0.;self.heading=0.;self.auto=False;self.failure=None
        self.group_command_token='';self.landing_check=False;self.landing_check_stamp=0.
        self.stop_pending=False
        self.phase_start=time.monotonic();self.progress_stamp=self.phase_start;self.progress_p=None
        self.last_plan=0.;self.last_tick=time.monotonic();self.last_status=0.;self.last_vis=0.;self.last_safe=0.
        self.map=RollingLocalMap(size=(60.,60.,60.) if self.count==10 else (60.,60.,30.),depth_stride=12 if self.count==10 else 8 if self.count==5 else 4);self.rejects=collections.Counter();self.inferences=0
        if self.cloud_mission:
            from cloud_map import LayeredCloudMap
            self.map=LayeredCloudMap()
            with open(('__UAV_PROJECT__/config/point.json'.replace("__UAV_PROJECT__", __import__("os").environ["UAV_PROJECT_ROOT"]))) as stream:self.context_hash=json.load(stream)['map_fingerprints']['world']
        self.cruise=CruiseGate();self.peer=None;self.peer_stamp=0.;self.distance=None;self.band=False
        self.chosen=np.empty((0,3));self.last_candidates=[]
        self.rendezvous=False;self.rendezvous_ready=False;self.dwell=None;self.swarm_result=None;self.swarm_calls=0;self.last_swarm=0.
        self.detour=None;self.last_detour=-float('inf')
        self.controller_fault=None;self.peer_controller_fault=None
        self.braking=False
        cfg['train']=False;cfg['velocity']=6.0
        self.height_px=cfg['image_height'];self.width=cfg['image_width'];self.visualize=False
        self.state_transform=StateTransform();self.lattice_primitive=LatticePrimitive.get_instance()
        self.device=select_device(os.environ.get('YOPO_INFERENCE_DEVICE','auto'),torch.cuda.is_available())
        if self.device=='cpu':torch.set_num_threads(1)
        self.policy=None
        if not self.cloud_mission:self.load_yopo_policy()
        self.executor=None
        if self.cloud_mission:
            from execution_bridge import ExecutionBridge
            self.executor=ExecutionBridge(self)
        self.ctrl=rospy.Publisher(self.ns+('/execution/input' if self.executor else '/pos_cmd'),PositionCommand,queue_size=1)
        self.arm_pub=rospy.Publisher(self.ns+'/arm',Bool,queue_size=1,latch=True)
        self.status=rospy.Publisher(self.ns+'/mission/status',String,queue_size=1,latch=True)
        from visual_transport import DisplayProcess
        self.display=DisplayProcess(self.id,self.count)
        rospy.on_shutdown(self.display.close)
        self.arm_pub.publish(Bool(False))
        rospy.Subscriber(self.ns+'/odom',Odometry,self.odom,queue_size=1,tcp_nodelay=True)
        rospy.Subscriber(self.ns+'/ground',PointStamped,self.on_ground,queue_size=1)
        rospy.Subscriber(self.ns+'/local_ground',PointCloud2,self.on_ground_cloud,queue_size=1)
        if self.cloud_mission:
            rospy.Subscriber('/swarm10/cloud_context/source',String,self.on_context_source,queue_size=1)
            rospy.Subscriber(self.ns+'/terrain_context/'+self.context_hash,PointCloud2,self.on_context_cloud,queue_size=1)
        rospy.Subscriber(self.ns+'/contact',Bool,lambda m:setattr(self,'contact',m.data),queue_size=1)
        rospy.Subscriber(self.ns+'/task',String,self.task,queue_size=1)
        rospy.Subscriber(self.ns+'/goal',PoseStamped,self.legacy_goal,queue_size=1)
        rospy.Subscriber(self.ns+'/reset',Bool,self.reset,queue_size=1)
        rospy.Subscriber(self.ns+'/controller_fault',String,self.on_controller_fault,queue_size=1)
        rospy.Subscriber('/scene/collision_detail',String,self.on_collision,queue_size=10)
        self.subscribe_yopo_images()
        if self.count==2:
            from mission_swarm import SwarmGuard
            self.guard=SwarmGuard(self.id);self.native_reference=False;self.last_native_pub=0.;self.swarm_reason='bootstrap'
            rospy.Subscriber('/drone_'+str(1-self.id)+'/mission/status',String,self.on_peer,queue_size=1)
            rospy.Subscriber('/drone_'+str(1-self.id)+'/controller_fault',String,self.on_controller_fault,callback_args=True,queue_size=1)
            from traj_utils.msg import PolyTraj
            self.traj_pub=rospy.Publisher(self.ns+'/planning/broadcast_traj_send',PolyTraj,queue_size=1)
            rospy.Subscriber('/drone_'+str(1-self.id)+'/planning/broadcast_traj_send',PolyTraj,self.on_peer_traj,queue_size=1)
            self.peer_traj=None
        self.five=None
        if self.count in (5,10):
            if not self.cloud_mission:
                for _ in range(3):self.infer(np.full((270,480),80.,np.float32),np.zeros(3),np.zeros(3),np.eye(3),np.array([5.,0.,0.]))
            self.inferences=0
            from formation_follower import FormationFollower
            self.five=FormationFollower(self) if self.count==5 else __import__('ten_follower').TenFollower(self)
        self.cloud_planner=None;self.handoff=None;self.terminal_client=None
        if self.cloud_mission:
            from yopo_handoff import YopoHandoff
            self.handoff=YopoHandoff(self)
            from terminal_stream_client import ContinuousTerminalClient
            self.terminal_client=ContinuousTerminalClient(self,clock=lambda:rospy.Time.now().to_sec())
            self.terminal_client.independent_environment=True
            self.terminal_client.executor_stopping=True
            if self.id in (2,7):
                from terminal_proposal_worker import LeaderProposalWorker
                self.terminal_client.proposal_worker=LeaderProposalWorker()
            rospy.on_shutdown(self.terminal_client.close)
            if self.id in (2,7):
                from cloud_leader import CloudLeaderPlanner
                self.cloud_planner=CloudLeaderPlanner(('__UAV_PROJECT__/code/Swarm-formation/native/cloud_swarm_native'.replace("__UAV_PROJECT__", __import__("os").environ["UAV_PROJECT_ROOT"])))
                rospy.on_shutdown(self.cloud_planner.close)
            rospy.Subscriber('/swarm10/planning_mode',String,self.on_planning_mode,queue_size=1)
        self.trace_control_stalls=os.environ.get('YOPO_TRACE_CONTROL_STALLS')=='1'
        if self.trace_control_stalls:faulthandler.register(signal.SIGUSR1,all_threads=True)
        self.plan_timer=rospy.Timer(rospy.Duration(.1),self.plan)
        self.ctrl_timer=rospy.Timer(rospy.Duration(.02),self.control)
        if self.terminal_client is not None:
            self.terminal_safety_timer=rospy.Timer(rospy.Duration(.04),self.terminal_safety_tick)
            self.coordination_pub=rospy.Publisher(self.ns+'/terminal/coordination',String,queue_size=1)
            self.coordination_timer=rospy.Timer(rospy.Duration(.05),self.coordination_tick)
        # Map-count telemetry can take longer than a command heartbeat budget.
        # Keep it on its own timer so the sole 50 Hz writer can keep running.
        self.status_timer=rospy.Timer(rospy.Duration(.1),self.status_tick)
        if self.depth_source=='pointcloud':
            from cloud_depth_process import CloudDepthProcess
            self.cloud_depth=CloudDepthProcess(self.id,self.count,clock=lambda:rospy.Time.now().to_sec())
            rospy.on_shutdown(self.cloud_depth.close)
            self.cloud_depth_timer=rospy.Timer(rospy.Duration(.02),self.on_cloud_depth)
        rospy.loginfo('MISSION_READY ns=%s sensor-only local map; native YOPO geometry retimed independently',self.ns)

    def coordination_tick(self,_):
        value=self.terminal_client.coordination_status(rospy.Time.now().to_sec())
        self.coordination_pub.publish(String(json.dumps(value,separators=(',',':'),allow_nan=False)))

    def load_yopo_policy(self):
        if self.planning_mode=='CLOUD':raise RuntimeError('YOPO loading forbidden in cloud mode')
        from io import BytesIO
        policy=YopoNetwork()
        # Python file reads release the GIL during disk I/O. The observed
        # UntypedStorage.from_file mmap path did not, starving the 50 Hz writer.
        # Deserialize from memory on CPU, then transfer the completed model.
        with open(str(portable_paths.WEIGHTS),'rb') as checkpoint:
            weights=torch.load(BytesIO(checkpoint.read()),map_location='cpu',weights_only=True)
        policy.load_state_dict(weights)
        policy=policy.to(self.device);policy.eval()
        self.policy=policy
        self.yopo_ready=True

    def on_planning_mode(self,msg):
        try:
            data=json.loads(msg.data)
            with self.maplock:
                before=self.planner_epoch
                self.handoff.receive(data,rospy.Time.now().to_sec())
                if self.planner_epoch!=before:
                    worker=getattr(self.terminal_client,'proposal_worker',None)
                    if worker is not None:worker.invalidate()
                    self.five.reference=None;self.five.active=False;self.desired[:]=0.;self.hold=self.p.copy()
                    self.set_phase('HOLD','Planning handoff; waiting for current epoch reference')
        except (ValueError,TypeError) as exc:self.reason='Invalid planning handoff: '+str(exc)

    def cloud_reference(self,now,mono):
        planner=self.cloud_planner
        if planner is None:return None
        ref=self.five.reference or {}
        permitted=(self.armed and self.planning_mode=='CLOUD' and self.phase=='FORMATION' and self.five.active and ref.get('stage')=='CRUISE'
                   and not self.failure and not self.stop_pending and not self.braking and not self.five.blocked
                   and not self.five.safety_paused and not ref.get('safety_pause') and self.id%5 not in ref.get('paused_members',[])
                   and mono-self.last_plan<.3 and -.1<=now-self.odom_stamp<=.5)
        if not permitted:
            if planner.active is not None or planner.next_plan is not None:planner.cancel()
            return None
        result=planner.reference(now)
        measured_reference=planner.reference(self.odom_stamp)
        if result is not None and measured_reference is not None and np.linalg.norm(measured_reference[0]-self.p)>1.5:
            planner.cancel();self.fail('Cloud polynomial tracking error exceeds 1.5 m');return None
        return result

    def terminal_reference(self,now):
        client=getattr(self,'terminal_client',None)
        if client is None or self.planning_mode!='YOPO' or not (self.five.reference or {}).get('terminal_joint'):return None
        ref=self.five.reference
        if ref.get('safety_pause') or getattr(self,'id',-1)%5 in ref.get('paused_members',[]) or getattr(self.five,'blocked',False) or getattr(self,'braking',False):
            if getattr(client,'committed',False):client.abort('Joint safety pause/braking')
            return None
        if not getattr(self.five,'active',True) or getattr(self.five,'suspended',False):return None
        return client.reference(now)

    def subscribe_yopo_images(self):
        # Receipt is always enabled. CLOUD callbacks retain only the latest raw
        # preview; decoding, projection and YOPO remain phase-owned below.
        if getattr(self,'depth_source','unity')=='pointcloud' or self.image_subscriptions:return
        for camera in ('front','up','down'):
            topic='depth' if camera=='front' else camera+'_depth'
            self.image_subscriptions.append(rospy.Subscriber(self.ns+'/'+topic,Image,self.on_depth,callback_args=camera,queue_size=1,buff_size=4000000))
            pose_topic='depth_camera_pose' if camera=='front' else camera+'_camera_pose'
            self.image_subscriptions.append(rospy.Subscriber(self.ns+'/'+pose_topic,PoseStamped,self.on_camera_pose,callback_args=camera,queue_size=10))

    def on_cloud_depth(self,_):
        # The sensor process does rendering. This short callback takes at most
        # one bounded mailbox value and never waits for the map/control lock.
        frame=self.cloud_depth.take()
        if frame is None:
            if self.cloud_depth.error:rospy.logerr_throttle(2.,'Pointcloud depth input: %s',self.cloud_depth.error)
            return
        self.image_received+=3
        self.cloud_preview_stamps={camera:frame['stamp'] for camera in ('front','up','down')}
        if self.planning_mode=='CLOUD' or self.p is None:return
        handoff=getattr(self,'handoff',None)
        if handoff is not None and handoff.enabled_stamp is not None and frame['stamp']<=handoff.enabled_stamp:return
        # Keep the source mask inside the same immutable pose/frame tuple;
        # the next callback cannot pair an old image with a newer mask.
        frames={camera:(frame['stamp'],frame['images'][camera],(frame['position'],frame['rotations'][camera],frame.get('peer_masks',{}).get(camera))) for camera in ('front','up','down')}
        self.frames=frames
        self.image_callbacks+=3

    def odom(self,m):
        p=m.pose.pose.position;v=m.twist.twist.linear;q=m.pose.pose.orientation
        with self.lock:
            self.p=np.array([p.x,p.y,p.z]);self.v=np.array([v.x,v.y,v.z]);self.rot=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
            self.odom_stamp=m.header.stamp.to_sec()
            if self.refp is None:
                self.refp=self.p.copy();self.hold=self.p.copy();self.goal=self.p.copy();self.yaw=math.atan2(self.rot[1,0],self.rot[0,0])
    def on_ground(self,m):
        self.ground_stamp=m.header.stamp.to_sec()
        if self.map.origin is None:self.ground=float(m.point.z)
    def on_ground_cloud(self,m):
        self.ground_cloud.put(m)
        if getattr(self,'trace_control_stalls',False):
            self.ground_receive_trace=dict(header=m.header.stamp.to_sec(),received=rospy.Time.now().to_sec(),monotonic=time.monotonic())
    def on_context_source(self,m):
        try:
            data=json.loads(m.data)
            self.context_source_ok=data.get('hashes',{}).get('world')==self.context_hash and data.get('height_kind')=='conservative_terrain_upper_bound'
        except (ValueError,TypeError):self.context_source_ok=False
    def on_context_cloud(self,m):
        if self.context_source_ok and m.header.frame_id=='world':
            self.context_cloud.put(m)
            if getattr(self,'trace_control_stalls',False):
                self.context_receive_trace=dict(header=m.header.stamp.to_sec(),received=rospy.Time.now().to_sec(),monotonic=time.monotonic())
    def on_camera_pose(self,m,camera):
        if getattr(self,'planning_mode','YOPO')=='CLOUD':return
        p=m.pose.position;q=m.pose.orientation
        key=(camera,m.header.stamp.to_nsec())
        pose=(np.array([p.x,p.y,p.z]),Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix())
        self.camera_pose[key]=pose
        pending=self.pending_depth.pop(key,None)
        if pending is not None:self.frames[camera]=(m.header.stamp.to_sec(),pending,pose)
        while len(self.camera_pose)>40:self.camera_pose.pop(next(iter(self.camera_pose)))
    def on_depth(self,m,camera):
        self.image_received=getattr(self,'image_received',0)+1
        if not hasattr(self,'preview_images'):self.preview_images={}
        self.preview_images[camera]=m
        if getattr(self,'planning_mode','YOPO')=='CLOUD':return
        if self.p is None:return
        self.image_callbacks=getattr(self,'image_callbacks',0)+1
        if m.encoding=='32FC1':d=np.frombuffer(m.data,dtype='>f4' if m.is_bigendian else '<f4').reshape(m.height,m.step//4)[:,:m.width].copy()
        elif m.encoding=='16UC1':d=np.frombuffer(m.data,dtype='>u2' if m.is_bigendian else '<u2').reshape(m.height,m.step//2)[:,:m.width].astype(float)/1000.
        else:return
        key=(camera,m.header.stamp.to_nsec());pose=self.camera_pose.get(key)
        if pose is not None:self.frames[camera]=(m.header.stamp.to_sec(),d,pose)
        else:
            self.pending_depth[key]=d
            while len(self.pending_depth)>12:self.pending_depth.pop(next(iter(self.pending_depth)))
    def on_peer(self,m):
        try:self.peer=json.loads(m.data);self.peer_stamp=time.time()
        except (ValueError,TypeError):pass
    def on_peer_traj(self,m):self.peer_traj=m
    def on_controller_fault(self,m,peer=False):
        try:
            value=json.loads(m.data)
            setattr(self,'peer_controller_fault' if peer else 'controller_fault',value if value.get('active') else None)
        except (ValueError,TypeError):pass
    def on_collision(self,m):
        try:
            value=json.loads(m.data)
            if int(value.get('drone_id',value.get('id',-1)))==self.id and self.armed:self.fail('Unity collision event: '+m.data[:300])
        except (ValueError,TypeError):pass
    def reset(self,m):
        if not m.data:return
        if getattr(self,'cloud_mission',False):
            self.reason='Cloud mission reset requires a new coordinated run';return
        with self.lock:
            self.arm(False);self.auto=False;self.failure=None;self.refp=None;self.refv[:]=0;self.desired[:]=0;self.cruise=CruiseGate();self.band=False;self.rendezvous_ready=False;self.rendezvous=False
            self.map=RollingLocalMap(size=(60.,60.,60.) if self.count==10 else (60.,60.,30.),depth_stride=12 if self.count==10 else 8 if self.count==5 else 4);self.frames={};self.set_phase('IDLE','Independent reset received')
            if self.count==2:self.guard.reset_fault();self.native_reference=False
    def arm(self,enabled):
        self.armed=bool(enabled);self.arm_pub.publish(Bool(self.armed))
    def set_phase(self,phase,reason=''):
        if phase!=self.phase:rospy.loginfo('%s %s -> %s: %s',self.ns,self.phase,phase,reason)
        self.phase=phase;self.reason=reason;self.phase_start=time.monotonic();self.dwell=None;self.progress_stamp=time.monotonic()
        self.progress_p=None if self.p is None else self.p.copy()
    def fail(self,reason):
        if self.failure:return
        self.failure=reason;self.auto=False;self.hold=self.p.copy();self.set_phase('FAULT_HOLD',reason)
        try:
            if self.count==10:
                from ten_core import failure_directory
                directory=str(failure_directory((str(portable_paths.SCENE)),(self.five.group_status or {}).get('run_id')))
            else:
                with open((str(portable_paths.SCENE) + '/Runtime/last-run')) as stream:directory=stream.read().strip()
            np.savez_compressed(os.path.join(directory,'drone_%d_failure_input.npz'%self.id),state=self.map.state,origin=self.map.origin,
                ground=self.map.ground,position=self.p,velocity=self.v,goal=self.goal,depth=self.frames.get('front',(None,np.empty(0)))[1])
        except (OSError,ValueError):pass
    def legacy_goal(self,m):
        if m.header.frame_id not in ('','world','map'):
            self.reason='Rejected legacy goal: frame must be world';return
        self.task(String(json.dumps(dict(op='goto',height_mode='ABS',height=m.pose.position.z,x=m.pose.position.x,y=m.pose.position.y))))
    def task(self,m):
        if self.p is None:return
        try:
            raw=json.loads(m.data)
            if self.count==10 and raw.get('op') in ('hold','cancel'):
                self.stop_pending=True
                worker=getattr(getattr(self,'terminal_client',None),'proposal_worker',None)
                if worker is not None:worker.invalidate()
            with self.maplock:t=validate_task(raw,self.p,self.map.ground_at)
        except (ValueError,TypeError,KeyError) as exc:
            self.reason='Rejected task: '+str(exc);rospy.logwarn(self.reason);return
        with self.maplock,self.lock:
            if self.count==10 and t['op'] in ('hold','cancel'):
                # A reference-stop callback can wait behind the authoritative
                # tokened hold. It must not erase its landing acknowledgment.
                if raw.get('_reference_stop') and getattr(self,'group_command_token',''):
                    self.stop_pending=False;return
                self.group_command_token=raw.get('_phase_token','');self.landing_check=False
                if self.five:self.five.active=False;self.five.reference=None;self.five.suspended=True
                if not self.armed and self.contact:self.stop_pending=False;return
                if self.contact and np.linalg.norm(self.v)<.15:
                    self.auto=False;self.arm(False);self.set_phase('LANDED','Stop received after stationary ground contact');self.stop_pending=False;return
            if self.count==10 and t['op']=='land':
                token=raw.get('_phase_token')
                if not token or token!=getattr(self,'group_command_token',None) or self.phase!='HOLD' or not getattr(self,'landing_check',False) or not 0<=rospy.Time.now().to_sec()-self.landing_check_stamp<=.6 or self.sensor_ready() is not None:
                    self.reason='Landing command rejected: current coordinated readiness required';return
            if getattr(self,'five',None) and self.five.group_running() and not raw.get('_formation_internal',False):
                if t['op'] not in ('hold','cancel','land'):
                    self.reason='Use the five-agent group interface during formation';return
                self.five.active=False;self.five.reference=None;self.five.interrupted=True
            if t['op'] in ('takeoff','mission') and (not self.contact or np.linalg.norm(self.v)>.2):
                self.reason='Takeoff requires stationary ground contact';return
            if t['op'] in ('goto','terrain_follow') and not self.armed:
                self.reason='Rejected airborne target: take off first';return
            if self.failure and not self.contact:
                self.reason='Rejected task: airborne fault is latched; stopping must finish';return
            op=t['op'];self.failure=None;self.auto=op=='mission';self.mode=t['height_mode'];self.height=t['height'];self.speed=t['speed']
            self.goal=np.array([t['x'],t['y'],self.height]);self.hold=self.p.copy();self.refp=self.p.copy();self.refv=self.v.copy()
            if op in ('hold','cancel'):
                self.auto=False;self.set_phase('HOLD','User '+op);self.stop_pending=False;return
            if op=='land':self.mode='AGL';self.height=BODY_OFFSET;self.set_phase('LAND','Landing requested');return
            if op in ('takeoff','mission'):
                if not self.contact or np.linalg.norm(self.v)>.2:
                    self.reason='Takeoff requires stationary ground contact';self.auto=False;return
                if self.count==2:self.guard.reset_fault();self.native_reference=False
                self.height=150. if op=='mission' else altitude_target(t['height_mode'],t['height'],self.ground);self.mode='ABS';self.goal=self.p.copy();self.goal[2]=self.height
                self.arm(True);self.cruise=CruiseGate();self.band=False;self.rendezvous_ready=False;self.rendezvous=False;self.set_phase('TAKEOFF','Physical motor spool-up');return
            if not self.armed:
                self.reason='Rejected airborne target: take off first';return
            self.pending_next='TERRAIN_FOLLOW' if op=='terrain_follow' else 'GOTO'
            here_target=self.height if self.mode=='ABS' else self.ground+self.height
            if abs(here_target-self.p[2])>.4:
                self.set_phase('VERTICAL_GOTO','Observe and change altitude before horizontal goal')
            else:self.set_phase(self.pending_next,'Explicit '+self.mode+' target')

    def sensor_ready(self):
        now=rospy.Time.now().to_sec()
        if now-self.odom_stamp>.5:return 'odometry stale'
        if now-self.ground_stamp>.6:return 'local ground stale'
        if getattr(self,'planning_mode','YOPO') in ('CLOUD','YOPO_WARMUP'):
            if self.map.ground_stamp is None or not -.1<=now-self.map.ground_stamp<=.6:return 'integrated ground stale'
            if getattr(self,'cloud_mission',False):
                context=getattr(self.map,'context',None)
                if not self.context_source_ok or context is None or context.ground_stamp is None or not -.1<=now-context.ground_stamp<=.6:return 'terrain stopping context missing/stale'
            return None
        for c in ('front','up','down'):
            if c not in self.frames or now-self.frames[c][0]>.6:return c+' depth stale'
        return None

    def infer(self,depth,p,v,rotation,goal,source_stamp=None):
        if getattr(self,'planning_mode','YOPO')=='CLOUD':raise RuntimeError('YOPO inference forbidden in cloud mode')
        client=getattr(self,'terminal_client',None)
        yielding=bool(client and getattr(client,'rolling',False) and client.committed)
        acceleration=self.refa.copy()
        if yielding:
            owner=(self.planning_mode,self.planner_epoch,self.phase,self.goal.copy())
            # No map reads or writes occur in policy inference. Let the
            # independent map/stop monitor run while the GPU is working.
            self.maplock.release()
        try:
            image=cv2.resize(depth,(self.width,self.height_px),interpolation=cv2.INTER_NEAREST)
            image=np.nan_to_num(image,nan=0.,posinf=20.,neginf=0.);image=np.clip(image,.04,20.)/20.
            obs=np.r_[rotation.T@v,rotation.T@acceleration,rotation.T@(goal-p)].astype(np.float32)[None,:]
            obs=self.state_transform.prepare_input_cpu(self.state_transform.normalize_obs_cpu(obs))
            input_array=np.ascontiguousarray(image[None,None],dtype=np.float32)
            with torch.inference_mode():pred,scores=self.policy(torch.from_numpy(input_array).to(self.device),torch.from_numpy(obs).to(self.device))
            self.input_preview.publish(input_array[0,0],source_stamp)
            ends,scores=self.process_output(pred.cpu().numpy(),scores.cpu().numpy(),True)
        finally:
            if yielding:self.maplock.acquire()
        self.inferences+=1
        if yielding and (self.failure or (self.planning_mode,self.planner_epoch,self.phase)!=owner[:3] or not np.array_equal(self.goal,owner[3])):
            raise RuntimeError('YOPO proposal ownership changed during inference')
        return rotation@ends.reshape(-1,3,3).transpose(0,2,1),scores

    def terminal_checkpoint(self):
        client=getattr(self,'terminal_client',None)
        if client and getattr(client,'rolling',False) and client.committed:
            # Read-only candidate searches need bounded proof intervals, not
            # a redundant full brake integration for every terrain sample.
            now=rospy.Time.now().to_sec()
            if not client.stop_inside or not -.05<=now-client.stop_checked<.04:self.terminal_safety_tick(None)
        return not getattr(self,'failure',None)

    def update_mission(self,p,v):
        elapsed=time.monotonic()-self.phase_start
        if self.count==10 and self.phase=='HOLD' and self.contact and np.linalg.norm(v)<.15:
            self.arm(False);self.set_phase('LANDED','Coordinated stop: ground contact settled');return
        if self.count==10 and self.phase=='END_HOLD' and ((self.five.group_status or {}).get('stage')=='TERRAIN' or (self.five.reference or {}).get('workflow')=='point-v1') and abs(p[2]-self.ground-self.height)>.25:
            self.set_phase('TERRAIN_FOLLOW','Updated terminal ground requires AGL correction')
        if self.phase=='GOTO' and np.linalg.norm(p-self.goal)<.5 and np.linalg.norm(v)<.3:
            self.hold=p.copy();self.set_phase('HOLD','Three-dimensional goal reached; hover')
        if self.phase=='TAKEOFF' and p[2]-self.ground>.7:self.set_phase('CLIMB','Climbing to ABS 150 m')
        if self.phase in ('TAKEOFF','CLIMB') and abs(p[2]-self.height)<.25 and np.linalg.norm(v)<.3:
            if self.auto:
                self.goal=np.array([1495.,-5.,150.]) if self.count==1 else np.array([1525.,-10.,150.]) if self.id==0 else np.array([1470.,-8.,150.])
                if self.count==2:
                    self.goal=np.array([100.,-58.,150.]) if self.id==0 else np.array([50.,-42.,150.]);self.rendezvous=True
                self.set_phase('CRUISE','High cruise; speed gate uses measured odometry')
            else:self.hold=self.p.copy();self.set_phase('HOLD','Takeoff complete')
        if self.phase=='CRUISE':
            self.cruise.update(rospy.Time.now().to_sec(),np.linalg.norm(v))
            if self.count==2 and self.rendezvous:
                if np.linalg.norm(p-self.goal)<.05 and np.linalg.norm(v)<.08 and rospy.Time.now().to_sec()-getattr(self,'last_native_pub',0)<.4:self.rendezvous_ready=True
                if self.rendezvous_ready and self.peer and (self.peer.get('rendezvous_ready') or self.peer.get('band_active')):
                    self.rendezvous=False;self.band=True
                    self.goal=np.array([1525.,-10.,150.]) if self.id==0 else np.array([1470.,-8.,150.])
            elif np.linalg.norm(p[:2]-self.goal[:2])<1 and np.linalg.norm(v)<.4:
                if self.cruise.best<60:return self.fail('Cruise did not achieve continuous 60 s measured speed >=10 m/s')
                self.hold=p.copy();self.set_phase('BRAKE','Hover at descent point')
        if self.phase=='BRAKE' and time.monotonic()-self.phase_start>2 and np.linalg.norm(v)<.25:
            if self.count==2 and (not self.peer or self.peer.get('phase') not in ('BRAKE','DESCENT')):return
            self.mode='AGL';self.height=2.;self.hold=p.copy();self.set_phase('DESCENT','Observed downward column; descending to AGL 2 m')
        if self.phase=='DESCENT' and abs(p[2]-self.ground-2)<.15 and abs(v[2])<.15:
            if self.count==2 and (not self.peer or self.peer.get('phase') not in ('DESCENT','TERRAIN_FOLLOW') or abs(self.peer.get('agl',999)-2)>.3):return
            self.goal=np.array([1745.,-50.,2.]) if self.count==1 else np.array([1737. if self.id==0 else 1753.,-50.,2.])
            self.mode='AGL';self.set_phase('TERRAIN_FOLLOW','Terrain measured locally; AGL 2 m; horizontal obstacle bypass')
        if self.count==2 and self.phase=='TERRAIN_FOLLOW' and (p[0]>1690 or self.peer and self.peer.get('phase') in ('END_APPROACH','END_HOLD','LAND','LANDED')):
            self.band=False;self.set_phase('END_APPROACH','Sequential terminal approach; distance exemption')
        if self.phase in ('TERRAIN_FOLLOW','END_APPROACH') and np.linalg.norm(p[:2]-self.goal[:2])<.5 and np.linalg.norm(v)<.3 and (self.count!=10 or (abs(p[2]-self.ground-self.height)<.2 and not self.braking and np.linalg.norm(v)<.15)):
            self.hold=p.copy();self.band=False;self.set_phase('END_HOLD','Terminal hover; distance exemption starts')
        if self.phase=='END_HOLD' and self.count!=10 and time.monotonic()-self.phase_start>3:
            if self.count==2 and self.id==1 and (not self.peer or self.peer.get('phase')!='LANDED'):return
            self.height=BODY_OFFSET;self.mode='AGL';self.set_phase('LAND','Slow final descent to body contact')
        if self.phase=='LAND' and self.contact and np.linalg.norm(v)<.15:
            if self.dwell is None:self.dwell=time.monotonic()
            elif time.monotonic()-self.dwell>1:
                self.arm(False);self.auto=False;self.set_phase('LANDED','Ground contact confirmed; motors stopped')
        elif self.phase=='LAND':self.dwell=None
        if self.phase=='GOTO' and np.linalg.norm(p-self.goal)<.4 and np.linalg.norm(v)<.3:
            self.hold=p.copy();self.set_phase('HOLD','Goal reached')
        if self.phase=='VERTICAL_GOTO':
            target=self.height if self.mode=='ABS' else self.ground+self.height
            if abs(p[2]-target)<.15 and abs(v[2])<.15:
                self.goal[2]=target;self.hold=p.copy()
                self.set_phase(self.pending_next,'Requested altitude established; continue to horizontal goal')

    def vertical_plan(self,p,v):
        target=self.height if self.mode=='ABS' else self.ground+self.height
        error=target-p[2];sign=np.sign(error)
        # During ground support, a spherical envelope would falsely intersect the
        # ground. Check the actual narrow vertical swept column above body top.
        distance=min(10.,abs(error)+1.)
        heights=p[2]+sign*np.arange(.6,distance+.1,.25)
        points=np.tile(p,(len(heights),1));points[:,2]=heights
        if sign<0:points=points[points[:,2]>self.ground+.6]
        safe=self.map.points_clear(points,radius=.35) if len(points) else np.ones(1,bool)
        free=distance if np.all(safe) else max(0.,float(np.flatnonzero(~safe)[0])*.25)
        approach=braking_speed(abs(error),.3,0. if self.phase=='LAND' else .02,1.)
        vz=sign*min(VERTICAL_SPEED,approach,braking_speed(free,.2,.3,1.2))
        if self.phase=='LAND':vz=sign*min(.45,abs(error)*1.5,abs(vz))
        if abs(error)<.08:vz=sign*min(abs(error)*1.5,abs(vz))
        self.desired=np.r_[limit_norm((self.hold[:2]-p[:2])*1.2,1.),vz]
        self.reason='Vertical local corridor verified' if free>.5 else 'Vertical corridor unknown/occupied; waiting'

    def horizontal_plan(self,p,v):
        checkpoint=getattr(self,'terminal_checkpoint',lambda:True)
        if not checkpoint():return
        escaping=getattr(self,'buffer_escape_active',False);self.buffer_escape_active=False
        if self.count==10 and self.mode=='AGL' and self.phase=='TERRAIN_FOLLOW' and not self.map.points_clear([p],radius=.35)[0]:
            settled=brake_settled(v,self.refv,self.refa,full_3d=True)
            if settled or escaping and max(np.linalg.norm(v),np.linalg.norm(self.refv))<.4:
                target=self.map.buffer_escape_target(p,self.goal,agl=self.height)
                if target is not None:
                    self.desired=limit_norm((target-p)*1.2,.3);self.buffer_escape_active=True
                    self.chosen=np.linspace(p,target,21);self.reason='Measured low-speed retreat from voxel margin'
                    self.heading=math.atan2(self.desired[1],self.desired[0]);return
        route_radius=.5 if self.count==10 and self.mode=='AGL' else .35
        navigation_goal=self.goal.copy()
        if self.mode=='AGL':
            goal_ground=self.map.ground_at(*self.goal[:2])
            navigation_goal[2]=goal_ground+self.height if np.isfinite(goal_ground) else p[2]
            if np.isfinite(goal_ground):self.goal[2]=navigation_goal[2]
        if self.detour is not None and np.linalg.norm(self.goal[:2]-p[:2])<28. and np.isfinite(navigation_goal).all() and route_corridor_clear(self.map,p,navigation_goal,radius=route_radius):
            self.detour=None
        if self.detour is not None:
            if np.linalg.norm(p[:2]-self.detour[-1,:2])<.8:self.detour=None
            else:
                waypoint=visible_detour_waypoint(self.map,p,self.detour,radius=route_radius)
                if waypoint is None:
                    self.detour=None;self.desired[:]=0.;self.try_detour(p);return
                navigation_goal=waypoint
        delta=navigation_goal[:2]-p[:2];remaining=float(np.linalg.norm(delta))
        if remaining<.05:self.desired[:]=0;return
        direct=delta/remaining;self.heading=math.atan2(direct[1],direct[0])
        if self.count==2 and self.phase=='END_APPROACH' and self.id==1 and self.peer and self.peer.get('phase')!='LANDED' and p[0]>1690:
            self.desired[:]=0;self.reason='Waiting for drone_0 terminal landing';return
        if remaining<3.:
            target=np.r_[navigation_goal[:2],self.height if self.mode=='ABS' else self.map.ground_at(*navigation_goal[:2])+self.height]
            samples=np.linspace(p,target,max(3,int(remaining/.15)+1))[1:]
            if route_corridor_clear(self.map,p,target,radius=route_radius):
                self.desired[:2]=direct*min(self.speed,1.,remaining*.8)
                self.desired[2]=np.clip((target[2]-p[2])*1.2,-VERTICAL_SPEED,VERTICAL_SPEED)
                self.chosen=samples;self.reason='Observed terminal approach';self.last_safe=time.monotonic();return
            self.desired[:]=0;self.reason='Terminal corridor unobserved/occupied';return
        # Rotate to observe before proceeding; retain braking reference while yawing.
        yaw_error=math.atan2(math.sin(self.heading-self.yaw),math.cos(self.heading-self.yaw))
        if abs(yaw_error)>.65:
            self.desired[:]=0;self.reason='Turning sensor toward local route';return
        frame=self.frames['front'];world,scores=self.infer(frame[1],p,v,self.rot,navigation_goal,source_stamp=frame[0])
        if not checkpoint():return
        directions=[]
        for idx in np.argsort(scores):
            vec=world[idx,:2,0];norm=np.linalg.norm(vec)
            if norm>.01 and np.dot(vec/norm,direct)>.05:directions.append((vec/norm,float(scores[idx]),int(idx)))
        # Add goal-directed ray only when a YOPO candidate supports that direction.
        if any(np.dot(d,direct)>.97 for d,_,_ in directions):directions.insert(0,(direct,-.1,-1))
        candidates=[];best=None
        for direction,score,index in directions:
            if not checkpoint():return
            ds=np.arange(.75,min(35.,remaining)+.1,.5)
            if not len(ds):ds=np.array([remaining])
            xyz=np.c_[p[:2]+ds[:,None]*direction,np.full(len(ds),self.height)]
            if self.mode=='AGL':xyz[:,2]=self.map.ground_at_many(xyz[:,:2])+self.height
            valid=self.map.points_clear(xyz,radius=route_radius)
            invalid=np.flatnonzero(~valid);end=int(invalid[0]) if len(invalid) else len(ds)
            if end<2:
                self.rejects[self.map.reject_reason(xyz[:max(1,end+1)]) or 'short_corridor']+=1;continue
            free=ds[end-1];usable=xyz[:end];safe_speed=min(self.speed,braking_speed(free),math.sqrt(max(0.,2*ACC_XY*max(0,remaining-1.))))
            slope=0.
            if self.mode=='AGL':
                gradient=np.diff(usable[:,2])/.5
                slope=float(np.max(np.abs(gradient),initial=0))
                future_limit=.8/np.maximum(np.abs(gradient),.01)
                curvature=np.abs(np.diff(usable[:,2],n=2))/.25
                if len(curvature):future_limit[1:]=np.minimum(future_limit[1:],np.sqrt(.25/np.maximum(curvature,1e-4)))
                available=np.maximum(0.,ds[:end-1]-max(np.linalg.norm(v[:2]),1.)*.3-1.)
                approach_limit=np.sqrt(future_limit**2+2*ACC_XY*available)
                safe_speed=min(safe_speed,float(np.min(future_limit,initial=self.speed)))
            metric=safe_speed*max(0,np.dot(direction,direct))-.03*score-.25*(1-np.dot(direction,direct))
            candidates.append(usable)
            if best is None or metric>best[0]:best=(metric,direction,safe_speed,usable,slope,index)
        self.last_candidates=candidates
        if best is None:
            self.desired[:]=0;self.reason='No observed safe YOPO corridor';self.try_detour(p);return
        _,direction,speed,points,slope,index=best
        self.chosen=points;self.desired[:2]=direction*speed
        if self.mode=='AGL':
            self.desired[2]=self.terrain_velocity(p,v)
        else:self.desired[2]=np.clip((self.height-p[2])*1.5,-VERTICAL_SPEED,VERTICAL_SPEED)
        self.reason='YOPO candidate %d; observed %.1f m; speed limit %.2f'%(index,np.linalg.norm(points[-1,:2]-p[:2]),speed)
        self.last_safe=time.monotonic()
        detour_threshold=min(3.,self.speed*.75) if self.count==10 else 3.
        if speed<detour_threshold and self.detour is None:self.try_detour(p)

    def try_detour(self,p):
        if self.mode!='AGL' or time.monotonic()-self.last_detour<1.:return
        checkpoint=getattr(self,'terminal_checkpoint',lambda:True)
        if not checkpoint():return
        self.last_detour=time.monotonic()
        direction=limit_norm(self.goal[:2]-p[:2],1.);side=np.array([-direction[1],direction[0]])
        remaining=np.linalg.norm(self.goal[:2]-p[:2])
        ten=getattr(self,'count',10)==10;radius=.5 if ten else .35
        candidates=[]
        if remaining<28:
            target=np.r_[self.goal[:2],self.map.ground_at(*self.goal[:2])+self.height]
            candidates.append(target)
        for forward,lateral in ((22,0),(20,10),(20,-10),(12,15),(12,-15),(10,0),(2,8),(2,-8),(0,12),(0,-12),(-1,3),(-1,-3),(0,2),(0,-2)):
            forward=min(forward,remaining)
            xy=p[:2]+direction*forward+side*lateral
            target=np.r_[xy,self.map.ground_at(*xy)+self.height]
            candidates.append(target)
        cursor=getattr(self,'detour_candidate_cursor',0)%len(candidates) if ten else 0
        for offset in range(len(candidates)):
            index=(cursor+offset)%len(candidates);target=candidates[index]
            if not np.isfinite(target).all() or not np.all(self.map.points_clear([target],radius=radius)):continue
            # One bounded search per cycle; unreachable far-side targets must
            # not monopolize the planner or starve observed near-side routes.
            self.detour_candidate_cursor=(index+1)%len(candidates)
            path=self.map.local_detour(p,target,agl=self.height,radius=radius,max_expansions=1200 if ten else 3000,search_budget=.045 if ten else None)
            if not checkpoint():return
            if path is not None and len(path)>3:
                self.detour=path;self.reason='Observed local A* bypass selected; YOPO follows local waypoints';return
            if ten:return

    def swarm_check(self,p,v):
        self.guard.agl=self.height if self.mode=='AGL' and self.phase in ('TERRAIN_FOLLOW','FAULT_HOLD','HOLD') else None
        active=self.band
        stop_reason=self.failure
        peer_traj=self.peer_traj
        if active and self.phase=='HOLD':stop_reason='User hold/cancel; coordinated braking'
        if active and self.peer and self.peer.get('phase') in ('FAULT_HOLD','HOLD'):
            stop_reason='Neighbor entered hold/fault; coordinated braking'
        if active and getattr(self,'peer_controller_fault',None):
            # A controller watchdog executes its own brake, not the last native
            # polynomial. Do not certify distance using that obsolete forecast.
            peer_traj=None;stop_reason='Neighbor controller watchdog; native commitment invalid'
        if active and getattr(self,'controller_fault',None):stop_reason='Own controller watchdog active'
        if active and stop_reason:
            result=self.guard.stop(p,v,self.refa,self.map,self.peer,peer_traj,stop_reason,rospy.Time.now().to_sec())
        else:result=self.guard.update(p,v,self.refa,self.desired,self.map,self.peer,peer_traj,active,rospy.Time.now().to_sec())
        self.native_reference=False
        if result.get('traj') is not None and not getattr(self,'controller_fault',None):
            self.traj_pub.publish(result['traj']);self.last_native_pub=rospy.Time.now().to_sec();self.swarm_calls+=1
        self.native_reference=bool(result.get('executable') and result.get('certified') and result.get('reference') is not None and not getattr(self,'controller_fault',None))
        self.swarm_result=result
        self.swarm_reason=result['reason']
        if active:
            self.desired=result['velocity'].copy() if self.native_reference else np.zeros(3)
            if not result.get('accepted'):self.rejects['swarm_'+result['reason']]+=1
            if result.get('fault'):self.fail('Swarm guard: '+result.get('fault_reason',result['reason']))

    def terrain_velocity(self,p,v):
        here=self.map.ground_at(*p[:2])
        speed=np.linalg.norm(v[:2])
        ahead=self.map.ground_at(*(p[:2]+v[:2]/max(speed,1e-9)*.25))
        if not np.isfinite(here+ahead):return 0.
        feed=(ahead-here)/.25*speed
        return float(np.clip(feed+2.*(here+self.height-p[2]),-VERTICAL_SPEED,VERTICAL_SPEED))

    def shape_desired(self,target):
        """Optional pre-integration shaping; the default preserves baseline."""
        return target

    def final_sweep(self,p,v):
        """Certify a complete delayed stop from measured velocity."""
        client=getattr(self,'terminal_client',None)
        if client is not None and self.planning_mode=='YOPO' and (self.five.reference or {}).get('terminal_joint'):
            if getattr(client,'independent_environment',False):self.terminal_safety_tick(None)
            else:client.monitor(rospy.Time.now().to_sec())
            return
        if getattr(self,'braking',False):
            if not brake_settled(v,getattr(self,'refv',v),self.refa,full_3d=self.count==10):
                self.desired[:]=0.;self.reason='Safety stop latched until velocity settles';return
            self.braking=False
        if self.count==10:
            terrain=self.terrain_velocity if self.mode=='AGL' and self.phase=='TERRAIN_FOLLOW' and not getattr(self.five,'safety_paused',False) and not self.five.blocked else None
            if terrain is None:
                # Same pointwise stop integration without small NumPy operations
                # monopolizing the planning timer at cruise/catch-up speeds.
                from terminal_stopping import actual_stop_paths
                paths=actual_stop_paths(p,v,self.refa,self.desired)
            else:
                paths=ten_stop_envelopes(p,v,self.refa,self.desired,terrain_velocity=terrain)
            escaping=getattr(self,'buffer_escape_active',False) and self.mode=='AGL' and self.phase=='TERRAIN_FOLLOW'
            clear_paths=[self.map.escape_points_clear(path,p) if escaping else self.map.points_clear(path,radius=.35) for path in paths]
            if not escaping and self.mode=='AGL' and self.phase=='TERRAIN_FOLLOW':
                # A physically clear stop at the voxel boundary leaves no room
                # for tracking error. Keep the route's 0.5 m margin at both stop
                # endpoints, while allowing a swept-clear departure out of it.
                endpoints=self.map.points_clear([path[-1] for path in paths],radius=.5)
                for clear,endpoint in zip(clear_paths,endpoints):clear[-1]&=endpoint
            if not all(np.all(clear) for clear in clear_paths):
                index=next(i for i,clear in enumerate(clear_paths) if not np.all(clear));point=paths[index][np.flatnonzero(~clear_paths[index])[0]]
                offsets=np.array([(a,b,c) for a in (-1,0,1) for b in (-1,0,1) for c in (-1,0,1)])*.5
                cells,inside=self.map._indices(point+offsets);values=np.zeros(len(cells),int);values[inside]=self.map.state[tuple(cells[inside].T)]
                self.stop_debug=dict(stamp=rospy.Time.now().to_sec(),position=p.tolist(),velocity=v.tolist(),desired=self.desired.tolist(),blocked_point=point.tolist(),cell_states=values.tolist(),peers={str(i):s['position'] for i,s in self.five.peers.items()})
                self.braking=True;self.desired[:]=0.;self.rejects['final_swept_reference']+=1
                self.reason='Actual/watchdog stop corridor blocked; controlled braking'
            return
        position=p.copy();velocity=v.copy();acceleration=self.refa.copy();points=[]
        duration=max(np.linalg.norm(v[:2])/ACC_XY+2*ACC_XY/JERK_XY,abs(v[2])/1.2+1.2)+.5
        for step in range(max(5,int(np.ceil(duration/.1)))):
            target=self.desired.copy() if step<3 else np.zeros(3)
            trial_target=self.shape_desired(target)
            trial_velocity,_=smooth_velocity(velocity,acceleration,trial_target,.1,vertical_speed=7. if self.count==10 else VERTICAL_SPEED)
            if self.mode=='AGL' and self.phase in ('TERRAIN_FOLLOW','END_APPROACH'):
                target[2]=self.terrain_velocity(position,velocity)
            # Certify the slower watchdog brake (1.2 m/s²) even with fast ten-mode control.
            target=self.shape_desired(target)
            next_velocity,acceleration=smooth_velocity(velocity,acceleration,target,.1,vertical_speed=7. if self.count==10 else VERTICAL_SPEED)
            position=position+(velocity+next_velocity)*.05
            velocity=next_velocity;points.append(position.copy())
        if not np.all(self.map.points_clear(points,radius=.35)):
            self.braking=True
            self.desired[:]=0.;self.rejects['final_swept_reference']+=1
            self.reason='Final accelerated reference blocked; controlled braking'
            if self.count==2:self.native_reference=False
            return False
        return True

    def plan(self,_):
        if self.p is None:return
        begin=time.monotonic()
        self.previous_plan_profile=getattr(self,'plan_profile',{})
        self.plan_profile={}
        self.plan_stage='map_lock';self.plan_stage_started=begin
        try:
            with self.maplock:
                self.plan_profile['map_lock']=time.monotonic()-self.plan_stage_started
                # The independent timer cannot enter while this cycle owns the
                # map. Recompute real certificates only at complete boundaries;
                # RLock re-entry retains the same mode/ownership gates.
                checkpoint=self.terminal_safety_tick if getattr(getattr(self,'terminal_client',None),'committed',False) else None
                if checkpoint is not None:checkpoint(None)
                self.plan_stage='update_pose';self.plan_stage_started=time.monotonic()
                p,v=self.p.copy(),self.v.copy();self.map.update_pose(p,v)
                self.plan_profile['update_pose']=time.monotonic()-self.plan_stage_started
                self.plan_stage='ground_take';self.plan_stage_started=time.monotonic()
                cloud=self.ground_cloud.take()
                if cloud is not None:
                    self.plan_stage='integrate_ground';self.plan_stage_started=time.monotonic()
                    pts=np.frombuffer(cloud.data,dtype='<f4').reshape(-1,cloud.point_step//4)[:,:3]
                    self.map.integrate_ground(pts,cloud.header.stamp.to_sec())
                    self.plan_profile['integrate_ground']=time.monotonic()-self.plan_stage_started
                    if checkpoint is not None:checkpoint(None)
                if hasattr(self.map,'integrate_context'):
                    self.plan_stage='context_take';self.plan_stage_started=time.monotonic()
                    context=self.context_cloud.take()
                    if context is not None:
                        self.plan_stage='integrate_context';self.plan_stage_started=time.monotonic()
                        points=np.frombuffer(context.data,dtype='<f4').reshape(-1,context.point_step//4)[:,:3]
                        self.map.integrate_context(points,context.header.stamp.to_sec());self.map.context_hash=self.context_hash
                        self.plan_profile['integrate_context']=time.monotonic()-self.plan_stage_started
                        if checkpoint is not None:checkpoint(None)
                warm=getattr(self,'handoff',None)
                image_map=warm.map if warm is not None and self.planning_mode=='YOPO_WARMUP' else self.map
                if image_map is not self.map:
                    self.plan_stage='warmup_update_pose';self.plan_stage_started=time.monotonic()
                    image_map.update_pose(p,v)
                    self.plan_profile['warmup_update_pose']=time.monotonic()-self.plan_stage_started
                    if cloud is not None:
                        self.plan_stage='warmup_integrate_ground';self.plan_stage_started=time.monotonic()
                        image_map.integrate_ground(pts,cloud.header.stamp.to_sec())
                        self.plan_profile['warmup_integrate_ground']=time.monotonic()-self.plan_stage_started
                if checkpoint is not None:checkpoint(None)
                warmup_image_ground=None
                def warmup_refresh_due(layer):
                    stamp=getattr(layer,'ground_stamp',None)
                    return stamp is None or not math.isfinite(stamp) or rospy.Time.now().to_sec()-stamp>=.2
                def refresh_warmup_inputs():
                    nonlocal warmup_image_ground
                    # Keep the active terrain map fresh at each complete image
                    # boundary. The future image map gets only the final latest
                    # ground below, without repeating its work per camera.
                    # Avoid rebuilding a still-fresh terrain volume at every
                    # image boundary. Leave its latest input queued until needed;
                    # only true received headers refresh either certificate.
                    if getattr(self,'planning_mode',None)!='YOPO_WARMUP':return
                    if hasattr(self.map,'integrate_context') and warmup_refresh_due(getattr(self.map,'context',None)):
                        latest_context=self.context_cloud.take()
                        if latest_context is not None:
                            self.plan_stage='warmup_latest_context';self.plan_stage_started=time.monotonic()
                            points=np.frombuffer(latest_context.data,dtype='<f4').reshape(-1,latest_context.point_step//4)[:,:3]
                            self.map.integrate_context(points,latest_context.header.stamp.to_sec())
                            self.map.context_hash=self.context_hash
                            self.plan_profile['warmup_latest_context']=self.plan_profile.get('warmup_latest_context',0.)+time.monotonic()-self.plan_stage_started
                    latest=self.ground_cloud.take() if warmup_refresh_due(self.map) else None
                    if latest is not None:
                        self.plan_stage='warmup_latest_ground';self.plan_stage_started=time.monotonic()
                        points=np.frombuffer(latest.data,dtype='<f4').reshape(-1,latest.point_step//4)[:,:3]
                        stamp=latest.header.stamp.to_sec()
                        self.map.integrate_ground(points,stamp)
                        warmup_image_ground=(points,stamp)
                        self.plan_profile['warmup_latest_ground']=self.plan_profile.get('warmup_latest_ground',0.)+time.monotonic()-self.plan_stage_started
                for name in ([] if getattr(self,'planning_mode',None)=='CLOUD' else list(self.frames)):
                    # Arrival callbacks replace immutable tuples. Pick each
                    # camera immediately before processing, not an old batch.
                    frame=self.frames[name]
                    stamp,d,pose=frame
                    if stamp>getattr(self,'integrated_'+name,0):
                        image_begin=time.monotonic();self.plan_stage='image_'+name;self.plan_stage_started=image_begin
                        focal=240. if name=='front' else 77.94228634059948
                        focus=[v,self.desired,self.refv]
                        if getattr(self,'terminal_client',None) is not None and self.planning_mode=='YOPO':focus.append(self.terminal_client.focus)
                        source_args={'peer_mask':pose[2]} if len(pose)>2 and pose[2] is not None else {}
                        image_map.integrate_depth(d,pose[0],pose[1],fx=focal,fy=focal,cx=240.,cy=135.,stamp=stamp,dense_center=self.count==10 and name!='front',focus_directions=focus if self.count==10 else None,**source_args);setattr(self,'integrated_'+name,stamp)
                        if warm is not None:warm.integrated[name]=stamp
                        self.plan_profile['image_'+name]=time.monotonic()-image_begin
                        refresh_warmup_inputs()
                        if checkpoint is not None:checkpoint(None)
                self.plan_stage='map';self.plan_stage_started=time.monotonic()
                if getattr(self,'planning_mode',None)=='YOPO_WARMUP':
                    refresh_warmup_inputs()
                    if warmup_image_ground is not None and image_map is not self.map:
                        self.plan_stage='warmup_final_image_ground';self.plan_stage_started=time.monotonic()
                        image_map.integrate_ground(*warmup_image_ground)
                        self.plan_profile['warmup_final_image_ground']=time.monotonic()-self.plan_stage_started
                self.plan_stage='map_finish';self.plan_stage_started=time.monotonic()
                if image_map is not self.map:image_map.known_body_volume(p)
                self.map.known_body_volume(p)
                measured=self.map.ground_at(*p[:2])
                if np.isfinite(measured):self.ground=measured
                self.plan_profile['map']=time.monotonic()-begin
                if checkpoint is not None:checkpoint(None)
                self.plan_stage='prepare';self.plan_stage_started=time.monotonic()
                if self.five:self.five.prepare()
                self.plan_profile['prepare']=time.monotonic()-self.plan_stage_started
                self.plan_stage='mission';self.plan_stage_started=time.monotonic()
                if self.phase not in ('IDLE','LANDED'):
                    if self.controller_fault:self.fail('Controller watchdog: '+self.controller_fault.get('reason','unknown'))
                    stale=self.sensor_ready()
                    client=getattr(self,'terminal_client',None)
                    if (stale and stale!='odometry stale' and client is not None
                            and client.perception_allowed(rospy.Time.now().to_sec())):stale=None
                    if stale:
                        self.desired[:]=0;self.reason=stale
                        if self.count==2:self.native_reference=False
                        if self.five and self.five.group_running():self.fail('Five-agent sensor timeout: '+stale)
                    else:
                        self.update_mission(p,v)
                        # Terminal ownership survives a missing/stopped group
                        # reference: that means brake, never independent flight.
                        joint=getattr(self,'terminal_client',None) is not None and self.planning_mode=='YOPO'
                        if joint and self.phase in ('TERRAIN_FOLLOW','END_HOLD'):self.five.plan_terrain(p,v)
                        elif self.phase in ('TAKEOFF','CLIMB','DESCENT','LAND','VERTICAL_GOTO'):self.vertical_plan(p,v)
                        elif self.phase=='TERRAIN_FOLLOW' and self.count==10:self.five.plan_terrain(p,v)
                        elif self.phase in ('CRUISE','TERRAIN_FOLLOW','GOTO','END_APPROACH'):self.horizontal_plan(p,v)
                        elif self.phase=='FORMATION':self.five.plan(p,v)
                        elif self.phase=='FAULT_HOLD':self.desired[:]=0
                        elif self.phase in ('HOLD','BRAKE','END_HOLD'):
                            self.desired=limit_norm((self.hold-p)*1.2,1.5)
                        if self.phase in ('CRUISE','TERRAIN_FOLLOW','GOTO') and not joint:
                            if self.count==10 and getattr(self.five,'active',False) and getattr(self.five,'safety_paused',False):
                                self.progress_p=p.copy();self.progress_stamp=time.monotonic()
                            if self.progress_p is None or np.linalg.norm(p-self.progress_p)>.5:
                                self.progress_p=p.copy();self.progress_stamp=time.monotonic()
                            stagnant=time.monotonic()-self.progress_stamp
                            if stagnant>5:
                                self.reason+='; local detour requested'
                                if self.mode=='AGL':
                                    self.try_detour(p)
                                    path=self.detour
                                    if path is not None and len(path)>2:
                                        waypoint=visible_detour_waypoint(self.map,p,path,lookahead=4,radius=.5 if self.count==10 else .35)
                                        if waypoint is not None:
                                            self.detour=path;delta=waypoint[:2]-p[:2];self.desired[:2]=limit_norm(delta,.6)
                                            self.desired[2]=self.terrain_velocity(p,v)
                                            self.heading=math.atan2(delta[1],delta[0])
                                            if self.count==10:self.five.blocked=False
                            if stagnant>(30 if self.count==10 else 10):self.fail('No safe local progress')
                self.plan_stage='sweep';self.plan_stage_started=time.monotonic()
                if self.phase in ('CRUISE','TERRAIN_FOLLOW','GOTO','END_APPROACH','FORMATION') or self.count==10 and self.phase=='END_HOLD':self.final_sweep(p,v)
                self.plan_stage='safety';self.plan_stage_started=time.monotonic()
                if self.five:self.five.safety(p,v)
                self.plan_profile['safety']=time.monotonic()-self.plan_stage_started
                if self.count==10 and getattr(self.five,'active',False) and getattr(self.five,'safety_paused',False):self.desired[:]=0.
                if self.count==10:
                    self.landing_check=False
                    if self.phase=='HOLD' and (self.five.group_status or {}).get('stage') in ('PRELAND','LANDING'):
                        self.landing_check=self.sensor_ready() is None and landing_column_ready(self.map,p,v,self.ground)
                        self.landing_check_stamp=rospy.Time.now().to_sec()
                if self.count==2:
                    if self.band and self.sensor_ready() is not None:self.fail('Sensor stale; coordinated stop')
                    self.swarm_check(p,v)
                self.last_plan=time.monotonic()
            # Heavy display work belongs to a separate process and GIL.
            self.plan_stage='visuals';self.plan_stage_started=time.monotonic()
            if begin-self.last_vis>1:self.publish_visuals();self.last_vis=begin
            self.plan_profile['visuals']=time.monotonic()-self.plan_stage_started
        except Exception as exc:
            rospy.logerr_throttle(1,'Plan error: %s',repr(exc));self.desired[:]=0;self.reason='Planner exception: '+repr(exc)
        self.plan_seconds=time.monotonic()-begin
        self.plan_profile['total']=self.plan_seconds
        self.plan_stage='idle';self.plan_stage_started=time.monotonic()

    def control(self,_):
        if self.p is None:return
        executor=getattr(self,'executor',None)
        executed=executor.synchronize(rospy.Time.now().to_sec()) if executor is not None else None
        now=time.monotonic();dt=min(.05,max(.001,now-self.last_tick));self.last_tick=now
        cloud_reference=self.cloud_reference(rospy.Time.now().to_sec(),now) if getattr(self,'cloud_planner',None) is not None else None
        terminal_reference=self.terminal_reference(rospy.Time.now().to_sec()) if getattr(self,'terminal_client',None) is not None else None
        if self.map.origin is not None:
            ground_now=self.map.ground_at(*self.p[:2])
            if np.isfinite(ground_now):self.ground=ground_now
        if executed is not None:
            self.refp,self.refv,self.refa=(a.copy() for a in executed)
        elif not self.armed:
            self.refp=self.p.copy();self.refv[:]=0;self.refa[:]=0
        elif cloud_reference is not None:
            self.refp,self.refv,self.refa=(a.copy() for a in cloud_reference)
        elif terminal_reference is not None:
            self.refp,self.refv,self.refa=(a.copy() for a in terminal_reference)
        elif self.count==2 and self.native_reference and now-self.last_plan<.6:
            reference=self.guard.reference(rospy.Time.now().to_sec())
            if reference is not None:
                self.refp=reference['position'];self.refv=reference['velocity'];self.refa=reference['acceleration']
                if np.linalg.norm(self.refp-self.p)>1.5:
                    self.native_reference=False;self.fail('Swarm polynomial tracking error exceeds 1.5 m')
            else:
                self.native_reference=False;self.fail('Swarm polynomial finite horizon expired')
        else:
            desired=self.desired.copy()
            joint=getattr(self,'terminal_client',None) is not None and self.planning_mode=='YOPO' and self.phase in ('TERRAIN_FOLLOW','END_HOLD')
            if joint:desired[:]=0. # No AGL/detour fallback before a common start or after cancellation.
            if getattr(self,'cloud_planner',None) is not None and self.planning_mode=='CLOUD' and (self.five.reference or {}).get('stage')=='CRUISE':desired[:]=0.
            if not joint and self.mode=='AGL' and self.phase in ('TERRAIN_FOLLOW','END_APPROACH') and self.sensor_ready() is None and not self.braking and not (self.count==10 and (self.five.blocked or getattr(self.five,'safety_paused',False))):
                desired[2]=self.terrain_velocity(self.refp,self.refv)
            if self.failure or getattr(self,'stop_pending',False) or now-self.last_plan>.6 or rospy.Time.now().to_sec()-self.odom_stamp>.5:desired[:]=0
            if self.count==10 and getattr(self.five,'active',False) and getattr(self.five,'safety_paused',False):desired[:]=0.
            desired=self.shape_desired(desired)
            limits=dict(vertical_speed=7.,vertical_acc=2.5,vertical_jerk=8.) if self.count==10 else {}
            new,self.refa=smooth_velocity(self.refv,self.refa,desired,dt,**limits)
            self.refp+=(self.refv+new)*dt*.5;self.refv=new
            # Prevent stale trajectory windup without replacing actual dynamics.
            difference=self.refp-self.p
            if np.linalg.norm(difference)>1.5:self.refp=self.p+limit_norm(difference,1.5)
            if self.phase=='LAND':self.refp[2]=max(self.ground+BODY_OFFSET-.01,self.refp[2])
        yaw_error=math.atan2(math.sin(self.heading-self.yaw),math.cos(self.heading-self.yaw));yaw_dot=np.clip(yaw_error/dt,-.6,.6);self.yaw+=yaw_dot*dt
        msg=PositionCommand();msg.header.stamp=rospy.Time.now();msg.header.frame_id='world';msg.trajectory_flag=msg.TRAJECTORY_STATUS_EMPTY
        for obj,arr in ((msg.position,self.refp),(msg.velocity,self.refv),(msg.acceleration,self.refa)):
            obj.x,obj.y,obj.z=map(float,arr)
        msg.yaw=self.yaw;msg.yaw_dot=yaw_dot;self.ctrl.publish(msg)
        # Replace one complete snapshot; telemetry must not mix arrays from
        # different control ticks while this writer advances its reference.
        self.last_control_reference=dict(stamp=msg.header.stamp.to_sec(),
            position=[msg.position.x,msg.position.y,msg.position.z],
            velocity=[msg.velocity.x,msg.velocity.y,msg.velocity.z],
            acceleration=[msg.acceleration.x,msg.acceleration.y,msg.acceleration.z])
        if executor is not None:
            actual=executor.reference(rospy.Time.now().to_sec())
            if actual is not None:
                self.last_control_reference=dict(stamp=executor.state['stamp'],position=actual[0].tolist(),
                    velocity=actual[1].tolist(),acceleration=actual[2].tolist())

    def status_tick(self,_):
        if self.p is not None:self.publish_status()

    def terminal_safety_tick(self,_):
        client=getattr(self,'terminal_client',None)
        if client is None or not client.committed or self.failure:return
        # Use the last complete environment while fusion owns the mutable map.
        # Each dynamic stopping proof still uses fresh measured PVA and expires
        # after150ms; the frozen integrated sensor times retain600ms expiry.
        acquired=self.maplock.acquire(False)
        independent=getattr(client,'independent_environment',False)
        if not acquired and not independent:return
        try:
            # Publish the complete cropped observation even when the dynamic
            # checker is busy. It can recheck a newer clear snapshot without
            # throwing away its still-fresh measured stopping computation.
            if acquired and independent and (_ is None or client.map_observation is None):
                client.capture_observation(rospy.Time.now().to_sec())
            # hold/cancel/PRELAND revoke formation ownership under this lock.
            # A historical commitment must not inspect the later landing path.
            if self.planning_mode!='YOPO' or self.phase not in ('TERRAIN_FOLLOW','END_HOLD'):return
            if not self.five.active or self.five.suspended or not (self.five.reference or {}).get('terminal_joint'):return
            if not client.stop_lock.acquire(False):return
            try:
                client.monitor(rospy.Time.now().to_sec())
                executor=getattr(self,'executor',None)
                if executor is not None:executor.publish_proof(client,rospy.Time.now().to_sec())
            finally:client.stop_lock.release()
        except Exception as exc:
            client.abort('Stopping check exception: '+repr(exc))
        finally:
            if acquired:self.maplock.release()

    def publish_status(self):
        stats=self.map.stats()
        # Finish slow map telemetry before taking a fresh, paired odometry
        # snapshot. Never label a pre-scan position with a post-scan timestamp.
        with self.lock:
            p=self.p.copy();v=self.v.copy();odom_stamp=self.odom_stamp
            stamp=rospy.Time.now().to_sec()
            reference=getattr(self,'last_control_reference',None)
            if reference is None:reference=dict(position=p.tolist(),velocity=v.tolist(),stamp=odom_stamp)
        self.distance=None if not self.peer else float(np.linalg.norm(p-np.array(self.peer['position'])))
        value=dict(stamp=stamp,drone_id=self.id,phase=self.phase,height_mode=self.mode,target_height=self.height,inference_device=getattr(self,'device','unknown'),
            position=p.tolist(),velocity=v.tolist(),abs=float(p[2]),ground=self.ground,ground_stamp=self.ground_stamp,integrated_ground_stamp=self.map.ground_stamp,agl=float(p[2]-self.ground),speed=float(np.linalg.norm(v)),
            control_reference=reference,
            goal=self.goal.tolist(),at_goal=bool(np.linalg.norm(p-self.goal)<.5 and np.linalg.norm(v)<.4),armed=self.armed,contact=self.contact,
            reason=self.reason,failure=self.failure,cruise_seconds=self.cruise.best,speed_1s=self.cruise.mean,band_active=self.band,
            pair_distance=self.distance,rendezvous_ready=self.rendezvous_ready,network_inferences=self.inferences,rejections=dict(self.rejects),map=stats,
            plan_seconds=getattr(self,'plan_seconds',0.),swarm_calls=self.swarm_calls,swarm_reason=getattr(self,'swarm_reason',None),
            controller_fault=self.controller_fault,swarm_action=(self.swarm_result or {}).get('action'),
            safety_failure=(self.swarm_result or {}).get('safety_failure',False),peer_commitment_assumed=(self.swarm_result or {}).get('peer_commitment_assumed',False))
        if self.five:
            value['formation']=self.five.status()
            value['odom_age']=stamp-odom_stamp
            value['plan_age']=time.monotonic()-self.last_plan
            value['landing_ready']=bool(self.landing_check)
            value['landing_check_stamp']=self.landing_check_stamp
            value['landing_ack_token']=self.group_command_token
            value['stop_debug']=getattr(self,'stop_debug',None)
            value['approach_ack']=getattr(self.five,'approach_ack',None)
            value['approach_check_stamp']=getattr(self.five,'approach_check_stamp',0.)
            value['approach_goal_clear']=getattr(self.five,'approach_goal_clear',False)
        value['frame_ages']={c:rospy.Time.now().to_sec()-f[0] for c,f in self.frames.items()}
        value.update(planner_mode=getattr(self,'planning_mode','YOPO'),planner_epoch=getattr(self,'planner_epoch',0),
                     image_callbacks=getattr(self,'image_callbacks',0),image_subscribers=len(getattr(self,'image_subscriptions',[])),yopo_ready=getattr(self,'yopo_ready',False),
                     image_received=getattr(self,'image_received',0),image_receive_ages={k:stamp-m.header.stamp.to_sec() for k,m in list(getattr(self,'preview_images',{}).items())},
                     image_usage='preview_only' if getattr(self,'planning_mode','YOPO')=='CLOUD' else 'planning')
        if getattr(self,'handoff',None) is not None:
            value.update(handoff_ready=self.handoff.ready(stamp),handoff_error=self.handoff.error,image_enable_stamp=self.handoff.enabled_stamp,
                         integrated_image_stamps=self.handoff.integrated,planning_run_id=self.handoff.run_id)
        planner=getattr(self,'cloud_planner',None)
        value['cloud_planning']=dict(calls=planner.calls,rejected=planner.rejected,reason=planner.reason,last_native_ms=planner.last_ms) if planner is not None else dict(calls=0)
        value['pending_depth_count']=len(self.pending_depth)
        value['pose_cache_count']=len(self.camera_pose)
        if hasattr(self,'display'):value['display']=self.display.status()
        if getattr(self,'cloud_depth',None) is not None:
            value['depth_source']='pointcloud';value['cloud_depth']=self.cloud_depth.status()
            value['image_receive_ages']={k:stamp-v for k,v in self.cloud_preview_stamps.items()}
        if getattr(self,'trace_control_stalls',False):
            trace_now=time.monotonic();started=getattr(self,'plan_stage_started',trace_now)
            value['plan_trace']=dict(enabled=True,stage=getattr(self,'plan_stage','idle'),started=started,
                                     age=max(0.,trace_now-started),profile=dict(getattr(self,'plan_profile',{})),
                                     previous_profile=dict(getattr(self,'previous_plan_profile',{})),
                                     ground_receive=getattr(self,'ground_receive_trace',None),
                                     context_receive=getattr(self,'context_receive_trace',None),
                                     ground_integrated=getattr(self.map,'ground_stamp',None),
                                     context_integrated=getattr(getattr(self.map,'context',None),'ground_stamp',None))
        if getattr(self,'terminal_client',None) is not None:value['terminal_joint']=self.terminal_client.status(stamp)
        if getattr(self,'executor',None) is not None:value['execution']=self.executor.state
        self.status.publish(String(json.dumps(value,allow_nan=True)))

    def visual_snapshot(self):
        from types import SimpleNamespace
        # Copy bounded arrays only while holding the map lock. Raster scanning,
        # depth projection, ROS serialization and publication happen afterward.
        with self.maplock:
            grid=SimpleNamespace(state=self.map.state.copy(),origin=self.map.origin.copy(),
                                 resolution=self.map.resolution,OCCUPIED=self.map.OCCUPIED)
            with self.lock:
                p,v,rot=self.p.copy(),self.v.copy(),self.rot.copy()
            frames={name:(stamp,d.copy(),(pose[0].copy(),pose[1].copy()))
                    for name,(stamp,d,pose) in list(self.frames.items())}
            peer=None if self.peer is None else dict(self.peer,position=list(self.peer['position']))
            return SimpleNamespace(map=grid,p=p,v=v,rot=rot,frames=frames,peer=peer,
                                   phase=self.phase,ground=self.ground,reason=self.reason,goal=self.goal.copy(),
                                   chosen=self.chosen.copy(),last_candidates=[p.copy() for p in self.last_candidates],
                                   distance=self.distance,band=self.band,
                                   stamp=rospy.Time.now().to_sec(),
                                   run_id=(self.five.group_status or {}).get('run_id') if self.count in (5,10) else None)

    def publish_visuals(self):
        # Only bounded snapshot admission runs here. The display child owns all
        # scans, depth projection, ROS serialization and historical path builds.
        try:return self.display.submit(self.visual_snapshot())
        except Exception as exc:
            rospy.logwarn_throttle(5,'Display submission skipped: %s',repr(exc))
            return False

if __name__=='__main__':
    Mission();rospy.spin()
