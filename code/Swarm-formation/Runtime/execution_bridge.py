"""Mission-side transport; no command publisher and no reference integration."""
import json
import numpy as np
import rospy
from std_msgs.msg import String


class ExecutionBridge:
    def __init__(self,mission):
        self.m=mission;self.state={};self.owned=False
        self.command=rospy.Publisher(mission.ns+'/execution/command',String,queue_size=1)
        self.proof=rospy.Publisher(mission.ns+'/execution/proof',String,queue_size=1)
        self.authority=rospy.Publisher(mission.ns+'/execution/authority',String,queue_size=1)
        rospy.Subscriber(mission.ns+'/execution/status',String,self.receive,queue_size=1,tcp_nodelay=True)

    def receive(self,msg):
        try:
            state=json.loads(msg.data)
            if state.get('stamp',0)<self.state.get('stamp',0):return
            self.state=state
            if state.get('error') and self.m.armed and not self.m.failure:
                self.m.fail('Independent execution: '+state['error'])
        except (ValueError,KeyError,TypeError):pass

    def send(self,value):self.command.publish(String(json.dumps(value,separators=(',',':'),allow_nan=False)))

    def send_environment(self,value):
        # Also used for revocation: never gate this on ownership/stop_inside.
        self.proof.publish(String(json.dumps(value,separators=(',',':'),allow_nan=False)))

    def publish_authority(self,headers,stamp):
        m=self.m
        value=dict(run_id=m.handoff.run_id,epoch=m.planner_epoch,stamp=stamp,
                   commits=[dict(digest=digest,start=start) for digest,start in headers])
        self.authority.publish(String(json.dumps(value,separators=(',',':'),allow_nan=False)))

    def prepare(self,packet,start,predecessor=None,commit=False):
        digest=packet['digest'];s=self.state
        if s.get('active')==digest:return True
        if s.get('prepared')!=digest:
            self.send(dict(kind='stage',packet=packet,start=start,predecessor=predecessor,commit=commit))
            return False
        if commit and not s.get('prepared_committed'):self.send(dict(kind='commit',digest=digest))
        return True

    def publish_proof(self,client,now,digests=None):
        m=self.m
        if not client.packet or not client.stop_inside:return
        if getattr(client,'independent_environment',False):
            # The checker commits one whole immutable-by-ownership snapshot.
            # Never attach newer map times or successor IDs while sending it.
            from terminal_environment import owns_execution
            with client.observation_lock:
                value=client.stop_proof
                if value is not None and owns_execution(client) and value['environment_generation']==client.environment_generation:
                    self.proof.publish(String(json.dumps(value,separators=(',',':'),allow_nan=False)))
            return
        stamps=[m.map.depth_stamp,m.map.ground_stamp]+[m.frames.get(c,(-1e9,))[0] for c in ('front','up','down')]
        value=dict(run_id=client.packet['run_id'],epoch=2,
                   digests=digests or (client.proof_digests() if hasattr(client,'proof_digests') else [client.packet['digest']]),checked=client.stop_checked,
                   odom_stamp=client.stop_stamp,reference_stamp=client.proof_reference_time(),
                   sensor_stamp=min(stamps),valid=m.sensor_ready() is None and not m.failure,
                   reference=client.stop_reference)
        self.proof.publish(String(json.dumps(value,separators=(',',':'),allow_nan=False)))

    def reference(self,now):
        state=self.state
        if not -.05<=now-state.get('stamp',-1e9)<=.15:return None
        value=state.get('reference')
        return None if value is None else np.asarray(value,float)

    def synchronize(self,now):
        m=self.m;client=getattr(m,'terminal_client',None)
        terminal=m.planning_mode=='YOPO' and m.phase in ('TERRAIN_FOLLOW','END_HOLD')
        active=bool(self.state.get('active'))
        recovering=self.state.get('source_gap') is not None
        controlling=active or self.owned or recovering
        if m.failure:
            if controlling:self.send(dict(kind='stop',reason=m.failure))
        elif m.stop_pending or ((active or recovering) and (m.five.suspended or not m.five.active)) or (recovering and not terminal):
            if controlling:
                self.owned=True
                self.send(dict(kind='hold',reason='Coordinated mission hold'))
        elif terminal and client and client.committed:
            self.owned=True
            self.prepare(client.packet,client.start,commit=True)
        elif self.owned and (not terminal or client and not client.committed and client.done):
            self.send(dict(kind='release'))
            if not active and not self.state.get('holding'):self.owned=False
        return self.reference(now) if (active or self.owned or recovering) else None
