"""Finite observation reuse for the verified, closed static-map simulator only.

This never turns old measurements into fresh ones. Motion permission belongs to
one exact finite packet; odometry, authority, tracking and local braking remain
independent requirements in the executor.
"""
import numpy as np


def launch_observation_fresh(created,proof,now):
    sensor=proof.get('sensor_stamp',-1e9)
    checked=proof.get('environment_checked',-1e9)
    return bool(np.isfinite([created,sensor,checked,now]).all()
                and created<=sensor<=checked and -.05<=now-sensor<=.6
                and -.05<=now-checked<=.35)


def launch_observation_covers_switch(created,proof,now,start):
    # Keep one 50 Hz command tick and transport jitter inside the deadlines
    # at activation. A certificate valid only at commit cannot authorize it.
    return bool(launch_observation_fresh(created,proof,now)
                and now<=start and start-proof['sensor_stamp']<=.565
                and start-proof['environment_checked']<=.33)


class StaticSceneGuard:
    def __init__(self,world_hash,source="pointcloud"):
        self.source=source
        self.world_hash=world_hash;self.frames=None;self.advanced=None;self.revoked=None

    def healthy(self,now):
        if self.advanced is not None and not -.05<=now-self.advanced<=1.2:
            self.revoked=self.revoked or 'Scene heartbeat stopped advancing'
        return self.advanced is not None and self.revoked is None

    def observe(self,message,now):
        self.healthy(now)
        if self.revoked:return
        frame=message.get('frames')
        if (message.get('map_sha256')!=self.world_hash or message.get('depth_source')!=self.source
                or message.get('drone_count')!=10 or message.get('person_present') is not False
                or message.get('collision_events')!=0 or message.get('bounds_events')!=0
                or not isinstance(frame,int) or isinstance(frame,bool) or frame<0
                or self.frames is not None and frame<self.frames):
            self.revoked='Static scene identity or safety changed';return
        if self.frames is None or frame>self.frames:self.advanced=float(now)
        self.frames=frame


class PerceptionPermission:
    def __init__(self,guard):
        self.guard=guard;self.certificates={};self.revoked=set()

    @staticmethod
    def key(slot):return (slot['run'],slot['epoch'],slot['digest'],slot['start'])

    def observe(self,slot,proof,now):
        key=self.key(slot)
        if proof.get('run_id')!=slot['run'] or proof.get('epoch')!=slot['epoch'] or slot['digest'] not in proof.get('digests',[]):return
        # Revocation is never filtered by timestamp or clear-proof schema. A
        # delayed clear message cannot undo an observed obstacle for this slot.
        if proof.get('valid') is False:self.revoked.add(key);return
        if key in self.revoked or not self.guard.healthy(now):return
        try:
            bounds=np.asarray([proof['lower'],proof['upper']],float);box=slot['box']
            dates=[proof['sensor_stamp'],proof['environment_checked'],proof['checked']]
            if (proof.get('valid') is not True or proof.get('kind')!='environment'
                    or proof.get('world_hash')!=self.guard.world_hash
                    or [slot['digest'],slot['start']] not in [list(s) for s in proof['environment_slots']]
                    or bounds.shape!=(2,3) or not np.isfinite(bounds).all()
                    or np.any(bounds[0]>box.min(0)+1e-9) or np.any(bounds[1]<box.max(0)-1e-9)
                    or not all(np.isfinite(s) and -.05<=now-s<=.6 for s in dates)):return
            self.certificates[key]=dict(run_id=slot['run'],epoch=slot['epoch'],digest=slot['digest'],start=slot['start'],
                world_hash=self.guard.world_hash,sensor_stamp=proof['sensor_stamp'],environment_checked=proof['environment_checked'],
                end=slot['start']+slot['plan'].duration)
        except (KeyError,ValueError,TypeError):return

    def ready(self,slot,now):
        key=self.key(slot)
        return bool(key in self.certificates and key not in self.revoked and self.guard.healthy(now))

    def launch_ready(self,slot,now):
        if not self.ready(slot,now):return False
        return (self.guard.source!='unity' or launch_observation_fresh(
            slot['created'],self.certificates[self.key(slot)],now))

    def scheduled_launch_ready(self,slot,now):
        if not self.ready(slot,now):return False
        return (self.guard.source!='unity' or launch_observation_covers_switch(
            slot['created'],self.certificates[self.key(slot)],now,slot['start']))

    def allowed(self,slot,now):
        return bool(slot['committed'] and self.ready(slot,now) and
                    (now<=slot['start']+slot['plan'].duration+5. or slot.get('tail_settled',False)))

    def describe(self,slot,now):
        if not self.ready(slot,now):return None
        return dict(self.certificates[self.key(slot)],committed=slot['committed'],
                    allowed=self.allowed(slot,now),mode='hold' if slot.get('tail_settled',False) else 'trajectory')


def matching_permission(state,packet,start,now,world_hash=None,committed=True):
    """Read an executor ACK, never manufacture a permission in the parent."""
    if not packet or state.get('error') or not -.05<=now-state.get('stamp',-1e9)<=.35:return False
    for cert in state.get('perception_permissions',[]):
        if cert.get('handoff_to') is not None and (cert['handoff_to']!=state.get('active') or not 0<=now-cert['handoff_start']<=.35):continue
        if (cert.get('run_id')==packet['run_id'] and cert.get('epoch')==packet['epoch']
                and cert.get('digest')==packet['digest'] and cert.get('start')==start
                and (world_hash is None or cert.get('world_hash')==world_hash)
                and (not committed or cert.get('committed') and cert.get('allowed'))):return True
    return False


def state_sensors_fresh(state,now):
    if not state.get('execution',{}).get('perception_enabled'):return True
    stamps=[state.get('map',{}).get(k) for k in ('ground_stamp','depth_stamp')]
    stamps += [state.get('integrated_image_stamps',{}).get(c) for c in ('front','up','down')]
    ages=state.get('frame_ages',{})
    return bool(all(s is not None and np.isfinite(s) and -.05<=now-s<=.6 for s in stamps)
        and set(ages)=={'front','up','down'} and all(np.isfinite(a) and -.05<=a+now-state['stamp']<=.6 for a in ages.values()))


def state_perception_allowed(state,now,world_hash,global_commits=()):
    joint=state.get('terminal_joint',{})
    if state.get('phase') not in ('TERRAIN_FOLLOW','END_HOLD'):return False
    packet=dict(run_id=state.get('planning_run_id'),epoch=2,digest=joint.get('digest'))
    if not joint.get('committed'):
        if not joint.get('perception_waiting'):return False
        for slot in global_commits:
            if (slot and slot.get('committed') and 0<=now-slot.get('committed_at',-1e9)<=.35
                    and slot.get('start')==joint.get('request_start') and now<=slot['start']+.35
                    and all(slot.get('packet',{}).get(k)==packet[k] for k in ('run_id','epoch','digest'))):
                return matching_permission(state.get('execution',{}),packet,slot['start'],now,world_hash,committed=False)
        return False
    if matching_permission(state.get('execution',{}),packet,joint.get('start'),now,world_hash):return True
    received=joint.get('commit_received')
    if received is None or not 0<=now-received<=.35:return False
    for slot in global_commits:
        if (slot and slot.get('committed') and slot.get('start')==joint.get('start')
                and all(slot.get('packet',{}).get(k)==packet[k] for k in ('run_id','epoch','digest'))
                and now<=slot['start']+.35):
            return matching_permission(state.get('execution',{}),packet,joint['start'],now,world_hash,committed=False)
    return False
