"""Five-mode adapter. Mission retains the sole PositionCommand publisher."""
import json,time
import numpy as np
import rospy
from std_msgs.msg import String
from formation_core import validate_reference,peer_safe,damped_settle

class FormationFollower:
    def __init__(self,mission):
        self.m=mission;self.reference=None;self.peers={};self.active=False
        self.run_id=None;self.blocked=False;self.note='waiting for group start';self.applied=0;self.interrupted=False
        self.group_status=None
        rospy.Subscriber('/swarm5/reference',String,self.receive,queue_size=1)
        rospy.Subscriber('/swarm5/status',String,self.group,queue_size=1)
        for i in range(5):
            if i!=mission.id:rospy.Subscriber('/drone_%d/mission/status'%i,String,self.peer,callback_args=i,queue_size=1)

    def peer(self,msg,i):
        try:self.peers[i]=json.loads(msg.data)
        except (ValueError,TypeError):pass

    def group(self,msg):
        try:self.group_status=json.loads(msg.data)
        except (ValueError,TypeError):pass

    def group_running(self):
        return self.active or bool(self.group_status and self.group_status.get('stage') not in ('IDLE','FAILED','LANDED'))

    def validate(self,ref,now):return validate_reference(ref,now)

    def receive(self,msg):
        try:
            ref=json.loads(msg.data)
            if ref.get('op')=='stop':
                self.active=False;self.reference=None
                if self.m.armed and not self.m.failure:self.m.task(String(json.dumps(dict(op='hold',_formation_internal=True))))
                return
            self.validate(ref,rospy.Time.now().to_sec())
            if not self.m.armed or self.m.failure or self.interrupted:return
            if self.run_id and ref['run_id']!=self.run_id and self.active:return
            self.run_id=ref['run_id'];self.reference=ref;self.active=True
        except (ValueError,KeyError,TypeError) as exc:
            self.note='Rejected group reference: '+str(exc)

    def prepare(self):
        m=self.m
        if not self.active:return
        try:points=self.validate(self.reference,rospy.Time.now().to_sec())
        except (ValueError,KeyError,TypeError):
            self.active=False;m.fail('Five-agent coordinator reference timeout');return
        if m.failure:return
        m.goal=points[m.id].copy();m.mode='ABS';m.height=float(m.goal[2]);m.speed=float(self.reference['speed'])
        if m.phase!='FORMATION':
            m.hold=m.p.copy();m.set_phase('FORMATION','Five-agent '+self.reference['stage'])
        self.applied+=1

    def plan(self,p,v):
        m=self.m;self.blocked=False
        delta=m.goal-p
        if np.linalg.norm(delta[:2])<.25:
            m.hold=m.goal.copy();m.vertical_plan(p,v)
        else:
            m.horizontal_plan(p,v)
        m.desired[2]=np.clip(m.desired[2],-.6,.6)
        m.desired[:2]*=min(1.,.8/max(np.linalg.norm(m.desired[:2]),1e-9))
        m.desired=damped_settle(delta,v,m.desired)
        # Horizontal planner may legitimately turn or wait for observations.
        self.blocked=(np.linalg.norm(delta)>.7 and np.linalg.norm(m.desired)<.05)

    def safety(self,p,v):
        m=self.m
        if not m.armed:return
        now=rospy.Time.now().to_sec()
        if self.group_status and self.group_status.get('stage') not in ('IDLE','FAILED','LANDED') and now-self.group_status.get('stamp',0)>1.2:
            self.active=False;m.desired[:]=0;m.fail('Five-agent coordinator heartbeat timeout');return
        for i in range(5):
            if i==m.id:continue
            s=self.peers.get(i)
            if s is None or not -.1<=now-float(s.get('stamp',0))<=1.2:
                m.desired[:]=0;self.blocked=True;self.note='Peer %d state stale'%i
                if self.active:m.fail(self.note)
                return
            other=np.asarray(s.get('position',[]),float);vel=np.asarray(s.get('velocity',[]),float)
            if other.shape!=(3,) or vel.shape!=(3,) or not np.all(np.isfinite(np.r_[other,vel])):
                m.desired[:]=0;m.fail('Invalid five-agent peer state');return
            if s.get('failure') or s.get('controller_fault'):
                m.desired[:]=0;self.active=False;m.fail('Five-agent peer %d fault'%i);return
            limit=1.0
            if np.linalg.norm(p-other)<limit:
                m.desired[:]=0;self.active=False;m.fail('Five-agent actual separation below %.1fm'%limit);return
            if getattr(m,'relay_enabled',False) and m.id%5==2:continue
            if not peer_safe(p,m.desired,other,vel) or not peer_safe(p,v,other,vel):
                m.desired[:]=0;self.blocked=True;self.note='Predicted peer conflict; braking';return
        self.note='Five-agent peer states and projected clearance checked'

    def status(self):
        return dict(active=self.active,run_id=self.run_id,stage=(self.reference or {}).get('stage'),
                    blocked=bool(self.blocked),note=self.note,references_applied=self.applied,interrupted=self.interrupted)
