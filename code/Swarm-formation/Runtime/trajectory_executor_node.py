#!/usr/bin/env python3
"""Sole 50 Hz command publisher, isolated from image fusion and inference."""
import json
import threading
import time
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Bool, String
from trajectory_execution import TrajectoryExecutor


class ExecutorNode:
    def __init__(self):
        rospy.init_node('trajectory_executor')
        identity=int(rospy.get_param('~drone_id'))
        self.terminal_only=bool(rospy.get_param('~terminal_only',False))
        self.output_owned=False
        from depth_source import depth_source,validate_player
        perception=None
        source=depth_source()
        if source=='pointcloud' or self.terminal_only:
            import portable_paths
            from point_mission_core import PointMission
            from static_perception import StaticSceneGuard,PerceptionPermission
            if source=='pointcloud':validate_player(portable_paths.SCENE,source)
            model=PointMission(('__UAV_PROJECT__/config/point.json'.replace("__UAV_PROJECT__", __import__("os").environ["UAV_PROJECT_ROOT"])))
            perception=PerceptionPermission(StaticSceneGuard(model.hashes['world'],source=source))
        self.core=TrajectoryExecutor(identity,require_authority=True,local_stopping=True,
                                     clock=lambda:rospy.Time.now().to_sec(),recover_source_gaps=True,perception=perception);self.lock=threading.RLock()
        self.ns='/drone_%d'%identity;self.last_status=0.;self.last_publish=None;self.max_gap=0.
        self.output=rospy.Publisher(self.ns+'/pos_cmd',PositionCommand,queue_size=1)
        self.status=rospy.Publisher(self.ns+'/execution/status',String,queue_size=1)
        self.peer_status=rospy.Publisher(self.ns+'/execution/peer_state',String,queue_size=1)
        rospy.Subscriber(self.ns+'/odom',Odometry,self.odom,queue_size=1,tcp_nodelay=True)
        rospy.Subscriber(self.ns+'/arm',Bool,self.arm,queue_size=1,tcp_nodelay=True)
        rospy.Subscriber(self.ns+'/execution/input',PositionCommand,self.forward,queue_size=1,tcp_nodelay=True)
        rospy.Subscriber(self.ns+'/execution/command',String,self.command,queue_size=1,tcp_nodelay=True)
        rospy.Subscriber(self.ns+'/execution/proof',String,self.proof,queue_size=1,tcp_nodelay=True)
        rospy.Subscriber(self.ns+'/execution/authority',String,self.authority,queue_size=1,tcp_nodelay=True)
        rospy.Subscriber('/swarm10/group_%d/execution_commit'%(identity//5),String,self.formation_commit,queue_size=1,tcp_nodelay=True)
        rospy.Subscriber(self.ns+'/controller_fault',String,self.controller_fault,queue_size=1,tcp_nodelay=True)
        if perception:rospy.Subscriber('/scene/status',String,self.scene,queue_size=1,tcp_nodelay=True)
        self.timer=rospy.Timer(rospy.Duration(.02),self.control)
        # Measurement heartbeat. Peers must see every body before the first
        # command exists, otherwise peer freshness and command flow form a
        # startup cycle (takeoff -> peer stale -> no command -> watchdog).
        if not self.terminal_only:self.peer_timer=rospy.Timer(rospy.Duration(.05),self.peer_heartbeat)

    def peer_heartbeat(self,_):
        try:payload=json.dumps(self.core.peer_state(),separators=(',',':'),allow_nan=False)
        except Exception:return
        self.peer_status.publish(String(payload))

    def odom(self,msg):
        p=msg.pose.pose.position;v=msg.twist.twist.linear
        with self.lock:self.core.observe([p.x,p.y,p.z],[v.x,v.y,v.z],msg.header.stamp.to_sec())

    def scene(self,msg):
        try:
            value=json.loads(msg.data)
            with self.lock:self.core.perception.guard.observe(value,rospy.Time.now().to_sec())
        except (ValueError,TypeError):pass

    def arm(self,msg):
        with self.lock:
            self.core.arm(msg.data)
            if not msg.data:self.output_owned=False

    def forward(self,msg):
        values=[[x.x,x.y,x.z] for x in (msg.position,msg.velocity,msg.acceleration)]
        with self.lock:self.core.forward(values,msg.header.stamp.to_sec(),msg.yaw)

    def controller_fault(self,msg):
        try:
            value=json.loads(msg.data)
            if value.get('active'):
                with self.lock:self.core.fail('Controller fault: '+str(value.get('reason')),rospy.Time.now().to_sec())
        except (ValueError,TypeError):pass

    def command(self,msg):
        with self.lock:
            now=rospy.Time.now().to_sec()
            try:
                value=json.loads(msg.data);kind=value['kind']
                if kind=='stop':self.core.fail(value.get('reason','Mission stop'),now)
                elif kind=='hold':self.core.hold(value.get('reason','Coordinated hold'),now)
                elif kind=='release':
                    self.core.release(now)
                    self.output_owned=False
                elif kind=='stage':
                    if self.core.active and self.core.active['digest']==value['packet']['digest']:return
                    self.core.stage(value['packet'],value['start'],now,value.get('predecessor'))
                    if value.get('commit'):self.core.commit(value['packet']['digest'],now)
                elif kind=='commit':
                    if self.core.active and self.core.active['digest']==value['digest']:return
                    self.core.commit(value['digest'],now)
            except (ValueError,KeyError,TypeError) as exc:
                # A malformed/late stage cannot cancel an already valid active
                # trajectory. It is reported, never silently counted as ready.
                self.rejection=str(exc)

    def proof(self,msg):
        try:
            value=json.loads(msg.data)
            with self.lock:
                now=rospy.Time.now().to_sec()
                slot=self.core.prepared
                perception=self.core.perception
                nearing_switch=(self.terminal_only and perception is not None and slot is not None
                                and not slot['committed'] and 0.<slot['start']-now<=.55)
                before=perception.describe(slot,now) if nearing_switch else None
                self.core.refresh(value,now)
                if nearing_switch:
                    after=perception.describe(slot,now)
                    # Report fresh launch evidence without waiting for the
                    # next periodic status from the slowest member.
                    if before!=after and (after is None or perception.scheduled_launch_ready(slot,now)):
                        self.publish_status(now)
        except (ValueError,KeyError,TypeError):pass

    def authority(self,msg):
        try:
            value=json.loads(msg.data)
            with self.lock:self.core.authorize(value,rospy.Time.now().to_sec())
        except (ValueError,KeyError,TypeError):pass

    def formation_commit(self,msg):
        try:
            value=json.loads(msg.data)
            with self.lock:
                now=rospy.Time.now().to_sec()
                if self.terminal_only:
                    if (value.get('kind')!='formation_commit' or value.get('group')!=self.core.identity//5
                            or value.get('epoch')!=2 or not -.05<=now-value['stamp']<=.35):
                        return
                    self.core.authorize(value,now)
                self.core.commit_from_coordinator(value,now,allow_initial=self.terminal_only)
        except (ValueError,KeyError,TypeError):pass

    def control(self,_):
        with self.lock:
            now=rospy.Time.now().to_sec()
            slot=self.core.prepared
            if self.terminal_only and self.core.armed and slot and slot['committed'] and now>=slot['start']:
                self.output_owned=True
            if not self.terminal_only or self.output_owned:
                reference=self.core.sample(now)
                if reference is not None:
                    msg=PositionCommand();msg.header.stamp=rospy.Time.from_sec(now);msg.header.frame_id='world'
                    msg.trajectory_flag=msg.TRAJECTORY_STATUS_EMPTY
                    for obj,value in zip((msg.position,msg.velocity,msg.acceleration),reference):obj.x,obj.y,obj.z=map(float,value)
                    msg.yaw=self.core.yaw
                    self.output.publish(msg)
                    mono=time.monotonic()
                    if self.last_publish is not None:self.max_gap=max(self.max_gap,mono-self.last_publish)
                    self.last_publish=mono
            if now-self.last_status>=.039:
                self.publish_status(now)
                if not self.terminal_only:self.peer_heartbeat(None)

    def publish_status(self,now):
        self.last_status=now;state=self.core.status(now)
        state.update(max_command_gap=self.max_gap,rejection=getattr(self,'rejection',None),output_owned=self.output_owned)
        self.status.publish(String(json.dumps(state,separators=(',',':'),allow_nan=False)))


if __name__=='__main__':ExecutorNode();rospy.spin()
