"""Finite trajectory execution without ROS, sensors, Torch or native libraries.

One state machine owns the reference actually sent to the controller. Input
proof timestamps are never changed on receipt. All times use the ROS clock.
"""
import numpy as np
import time
from terminal_formation import packet_paths, tracking_reservation, TRACKING_LIMIT
from terminal_stopping import actual_stop_paths
from mission_core import smooth_velocity
from terminal_corridor import contains_stops


class TrajectoryExecutor:
    def __init__(self, identity, require_authority=False, local_stopping=False, clock=None, recover_source_gaps=False, perception=None):
        self.identity=int(identity);self.armed=False;self.p=None;self.v=None;self.odom_stamp=-1e9
        self.active=None;self.previous=None;self.prepared=None;self.leases={};self.serial=0
        self.error=None;self.detail=None;self.reference=None;self.source=None;self.source_stamp=-1e9
        self.yaw=0.;self.tick=None;self.switches=0;self.last_switch=None;self.brake=None;self.holding=False
        self.require_authority=require_authority;self.authority={};self.authority_stamp=-1e9
        self.authority_reuse_ticks=0
        self.local_stopping=local_stopping;self.clock=clock;self.stop_checked=None;self.stop_seconds=0.;self.stop_digest=None
        self.recover_source_gaps=recover_source_gaps;self.released_tail=None;self.source_gap=None;self.source_recoveries=0
        self.perception=perception
        self.perception_reuse_ticks=0
        self.settled_stop_recoveries=0

    def arm(self, value):
        value=bool(value)
        if value != self.armed:
            self.stop_checked=None;self.stop_digest=None
            self.active=None;self.previous=None;self.prepared=None;self.leases={};self.error=None;self.detail=None;self.brake=None;self.holding=False
            self.released_tail=None;self.source_gap=None
            if self.perception:self.perception.certificates.clear();self.perception.revoked.clear()
        self.armed=value

    def observe(self, p, v, stamp):
        p,v=np.asarray(p,float),np.asarray(v,float)
        if p.shape!=(3,) or v.shape!=(3,) or not np.isfinite([p,v]).all() or not np.isfinite(stamp):return
        if stamp<self.odom_stamp:return
        self.p=p.copy();self.v=v.copy();self.odom_stamp=float(stamp)

    def forward(self, pva, stamp, yaw):
        pva=np.asarray(pva,float)
        if pva.shape!=(3,3) or not np.isfinite(pva).all() or not np.isfinite([stamp,yaw]).all():return
        if stamp<self.source_stamp:return
        self.source=pva.copy();self.source_stamp=float(stamp);self.yaw=float(yaw)

    def stage(self, packet, start, now, predecessor=None):
        if self.error or self.holding:raise ValueError('Executor stop is latched')
        if packet.get('group')!=self.identity//5 or packet.get('epoch')!=2:raise ValueError('Executor ownership')
        if self.prepared and self.prepared['digest']==packet.get('digest'):
            if self.prepared['start']!=start:raise ValueError('Prepared start changed')
            return
        if not np.isfinite([start,now]).all() or start<now+.05:raise ValueError('Late trajectory preparation')
        if self.active and predecessor is None:raise ValueError('Active owner requires explicit predecessor')
        if packet.get('serial',0)<=self.serial:raise ValueError('Replayed trajectory')
        plan=packet_paths(packet)[self.identity%5]
        box=None
        if self.local_stopping:
            box=np.asarray(packet['envelopes'][self.identity%5],float).copy()
            if box.shape!=(8,3) or not np.isfinite(box).all():raise ValueError('Invalid execution reservation')
        velocity_limit,_=tracking_reservation(plan)
        if max(np.linalg.norm(x) for x in plan.sample(plan.duration)[1:])>.001:raise ValueError('Missing finite stopping tail')
        if self.active:
            if predecessor!=self.active['digest'] or packet['run_id']!=self.active['run']:
                raise ValueError('Wrong trajectory predecessor')
            elapsed=start-self.active['start']
            if elapsed<=0.:raise ValueError('Switch outside active horizon')
            if elapsed>=self.active['plan'].duration and not self.settled(now):
                raise ValueError('Post-tail switch requires measured settlement')
            old=np.array(self.active['plan'].sample(min(elapsed,self.active['plan'].duration)));new=np.array(plan.sample(0.))
            if np.max(np.linalg.norm(old-new,axis=1))>1e-5:raise ValueError('Discontinuous switch PVA')
        elif predecessor is not None:raise ValueError('Missing trajectory predecessor')
        else:
            p,v,a=plan.sample(0.)
            if self.p is None or np.linalg.norm(p-self.p)>.10 or np.linalg.norm(self.v)>=.035 or max(np.linalg.norm(v),np.linalg.norm(a))>.06:
                raise ValueError('Initial trajectory start changed')
        self.prepared=dict(digest=packet['digest'],run=packet['run_id'],epoch=2,plan=plan,start=float(start),
                           predecessor=predecessor,committed=False,velocity_limit=velocity_limit,serial=packet['serial'],
                           created=packet['created'],box=box)
        self.serial=packet['serial']

    def commit(self, digest, now, minimum_lead=.05):
        slot=self.prepared
        if self.error or slot is None or slot['digest']!=digest:raise ValueError('Commit without prepared trajectory')
        if slot['committed']:return
        if not minimum_lead<=slot['start']-now<=1.:raise ValueError('Commit outside start window')
        if self.perception and not self.perception.scheduled_launch_ready(slot,now):raise ValueError('Launch observation does not cover switch')
        slot['committed']=True
        self.remember_authority(slot,now)

    def refresh(self, proof, now):
        if self.error:return
        if not isinstance(proof,dict) or proof.get('epoch')!=2:return
        if self.perception:
            for slot in (self.active,self.prepared):
                if slot is not None:self.perception.observe(slot,proof,now)
        if self.local_stopping and proof.get('valid'):
            try:
                dates=[proof[k] for k in ('checked','environment_checked','sensor_stamp','reference_stamp')]
                bounds=np.asarray([proof['lower'],proof['upper']],float)
                slots=proof['environment_slots']
                if (not np.isfinite(dates).all() or bounds.shape!=(2,3) or not np.isfinite(bounds).all()
                        or np.any(bounds[0]>bounds[1]) or not 1<=len(slots)<=2
                        or any(len(x)!=2 or not isinstance(x[0],str) or not np.isfinite(x[1]) for x in slots)):
                    return
            except (KeyError,ValueError,TypeError):return
        slots=[s for s in (self.active,self.prepared) if s is not None]
        for slot in slots:
            if proof.get('run_id')!=slot['run'] or slot['digest'] not in proof.get('digests',[]):continue
            old=self.leases.get(slot['digest'])
            if old is None or proof.get('checked',-1e9)>=old.get('checked',-1e9):
                self.leases[slot['digest']]=dict(proof)

    def commit_from_coordinator(self,message,now,allow_initial=False):
        # Explicit all-member commit, separate from ordinary authority
        # heartbeats. It can only activate an already prepared successor.
        slot=self.prepared
        if (not self.perception or not self.local_stopping or not self.require_authority or self.error or self.holding
                or slot is None or (slot['predecessor'] is None and not allow_initial) or not isinstance(message,dict)):
            return False
        try:
            if (message.get('kind')!='formation_commit' or message.get('group')!=self.identity//5
                    or message.get('run_id')!=slot['run'] or message.get('epoch')!=slot['epoch']
                    or not -.05<=now-message['stamp']<=.35):return False
            headers=message.get('commits',[])
            if not isinstance(headers,list) or not 1<=len(headers)<=2:return False
            if not any(h.get('digest')==slot['digest'] and h.get('start')==slot['start']
                       and h.get('predecessor')==slot['predecessor'] for h in headers):return False
            # The dedicated compact Unity authority arrives independently of
            # image fusion and the parent reference callback. Leave one
            # control tick for activation; keep the ordinary path unchanged.
            minimum_lead=.02 if allow_initial else .05
            if not self.perception.scheduled_launch_ready(slot,now) or not minimum_lead<=slot['start']-now<=1.:return False
            self.authorize(message,now)
            key=(slot['run'],slot['epoch'],slot['digest'],slot['start'])
            if not -.05<=now-self.authority.get(key,-1e9)<=.35:return False
            self.commit(slot['digest'],now,minimum_lead);return True
        except (KeyError,TypeError,ValueError,AttributeError):return False

    def authorize(self, message, now):
        if self.error or not isinstance(message,dict):return
        owners={(s['run'],s['epoch']) for s in (self.active,self.prepared) if s is not None}
        if owners and (message.get('run_id'),message.get('epoch')) not in owners:return
        stamp=message.get('stamp',-1e9)
        if not np.isfinite(stamp) or stamp>now+.05 or stamp<self.authority_stamp:return
        commits=message.get('commits',[])
        if not isinstance(commits,list) or len(commits)>2:return
        headers={}
        for item in commits:
            if not isinstance(item,dict) or not isinstance(item.get('digest'),str):return
            start=item.get('start')
            if not isinstance(start,(int,float)) or not np.isfinite(start):return
            headers[(message.get('run_id'),message.get('epoch'),item['digest'],start)]=stamp
        # Heartbeats describe authority, never transfer control. A release or
        # coordinated hold must arrive through the explicit command channel.
        self.authority=headers;self.authority_stamp=stamp
        for slot in (self.active,self.prepared):
            if slot is not None:self.remember_authority(slot,now)

    def remember_authority(self,slot,now):
        # Admission records an exact committed reservation, not a renewable
        # velocity lease. Its owner cannot be retired by missing heartbeats.
        # This recovery mode requires the closed-scene environment certificate
        # and independent local braking; legacy/live-sensor execution is intact.
        key=(slot['run'],slot['epoch'],slot['digest'],slot['start'])
        stamp=self.authority.get(key,-1e9)
        if (self.require_authority and self.local_stopping and self.perception
                and slot['committed'] and self.perception.ready(slot,now)
                and -.05<=now-stamp<=.35):slot['authority_grant']=(key,stamp)

    def retained_authority(self,slot,now):
        grant=slot.get('authority_grant')
        key=(slot['run'],slot['epoch'],slot['digest'],slot['start'])
        return bool(self.require_authority and self.local_stopping and grant and grant[0]==key
                    and self.perception and self.perception.guard.source!='unity' and self.perception.allowed(slot,now))

    def fail(self, reason, now, **detail):
        if self.error:return
        self.error=str(reason);self.detail=dict(stamp=now,odom_stamp=self.odom_stamp,**detail)
        acceleration=np.zeros(3) if self.reference is None else self.reference[2].copy()
        if self.p is not None:self.brake=np.array([self.p.copy(),self.v.copy(),acceleration])
        self.active=None;self.prepared=None;self.leases={}
        self.source_gap=None;self.released_tail=None

    def hold(self, reason, now):
        if self.error or self.holding:return
        acceleration=np.zeros(3) if self.reference is None else self.reference[2].copy()
        if self.p is not None:self.brake=np.array([self.p.copy(),self.v.copy(),acceleration])
        self.holding=True;self.active=None;self.prepared=None;self.leases={}
        self.source_gap=None;self.released_tail=None

    def proof_problem(self, slot, now):
        proof=self.leases.get(slot['digest'],{})
        if self.perception and self.perception.guard.revoked:return 'Static scene permission revoked'
        if self.perception and self.perception.key(slot) in self.perception.revoked:return 'Environment permission revoked'
        if not proof.get('valid'):return 'Invalid stopping proof'
        limits=[('checked',.15),('reference_stamp',.35),('sensor_stamp',.6),('odom_stamp',.5)]
        if self.local_stopping:
            if proof.get('kind')!='environment':return 'Missing environment proof'
            if [slot['digest'],slot['start']] not in [list(x) for x in proof.get('environment_slots',[])]:return 'Environment start mismatch'
            lower=np.asarray(proof.get('lower',[]),float);upper=np.asarray(proof.get('upper',[]),float)
            box=slot['box']
            if (lower.shape!=(3,) or upper.shape!=(3,) or not np.isfinite([lower,upper]).all()
                    or np.any(lower>box.min(0)+1e-9) or np.any(upper<box.max(0)-1e-9)):
                return 'Environment does not cover execution reservation'
            limits=[('sensor_stamp',.6),('environment_checked',.6),('reference_stamp',.35)]
        for key,limit in limits:
            stamp=proof.get(key,-1e9)
            if key=='reference_stamp' and self.require_authority:
                stamp=self.authority.get((slot['run'],slot['epoch'],slot['digest'],slot['start']),-1e9)
            age=now-stamp
            if not -.05<=age<=limit:
                if age>limit and key=='reference_stamp' and self.retained_authority(slot,now):continue
                if (age>limit and key in ('sensor_stamp','environment_checked') and self.local_stopping
                        and self.perception and self.perception.allowed(slot,now)):continue
                return 'Expired/future '+key
        return None

    def check_local_stop(self,slot,now,next_reference):
        begun=time.monotonic();previous=self.stop_checked
        if previous is not None and not -.05<=now-previous<=.15:
            if now>previous and self.recheck_settled_stop(slot,now):return True
            self.fail('Expired local stopping check',now,recovery=getattr(self,'settled_recovery_check',None));return False
        # Keep the original25Hz full-brake cadence. The50Hz caller has already
        # checked authority/environment/odometry and measured P/V tracking.
        # A promoted reservation always requires a new actual-stop check.
        if previous is not None and self.stop_digest==slot['digest'] and 0.<=now-previous<.04-1e-9:
            return self.finish_local_stop(slot,now,begun,previous,refresh=False)
        # The larger observation is not permission to leave the current
        # reservation. On promotion this is immediately the new box only.
        box=slot['box'];margin=.10/np.sqrt(3.)
        lower=box.min(0)-margin;upper=box.max(0)+margin
        references=[next_reference]
        if self.reference is not None:references.append(self.reference)
        for reference in references:
            paths=actual_stop_paths(self.p,self.v,reference[2],reference[1])
            corridor=getattr(slot['plan'],'stop_corridor',None)
            inside=(contains_stops(corridor,paths) if corridor is not None else
                    all(np.all((q>=lower)&(q<=upper)) for q in paths))
            if not inside:
                self.fail('Measured actual stopping path exceeds reserved envelope',now);return False
        self.stop_seconds=time.monotonic()-begun
        return self.finish_local_stop(slot,now,begun,previous,refresh=True)

    def finish_local_stop(self,slot,now,begun,previous,refresh):
        # Even a cached proof must survive a scheduler pause between sample
        # entry and publication. Reuse never re-dates the dynamic proof.
        finished=float(self.clock()) if self.clock else now+time.monotonic()-begun
        if (not -.05<=finished-now<=.15 or previous is not None and not -.05<=finished-previous<=.15):
            if finished>=now and self.recheck_settled_stop(slot,finished):return True
            self.fail('Expired local stopping computation',finished,recovery=getattr(self,'settled_recovery_check',None));return False
        problem=self.proof_problem(slot,finished)
        if problem:self.fail(problem,finished);return False
        if not -.05<=finished-self.odom_stamp<=.5:
            self.fail('Expired/future odometry',finished);return False
        if refresh:self.stop_checked=now;self.stop_digest=slot['digest']
        return True

    def recheck_settled_stop(self,slot,now):
        # Only a previously settled, still-owned static hold can restart a
        # dynamic check after scheduling delay. No moving deadline is extended,
        # and no stale proof is re-dated: compute the complete brakes again.
        problem=self.proof_problem(slot,now)
        # A known hold can acquire small residual drift during the pause.
        # Stay inside the existing tracking tolerance and recompute its full
        # brakes; initial settlement and successor admission still use .035.
        drift_limit=min(.10,slot['velocity_limit'])
        checks=dict(active=slot is self.active,same_reservation=self.stop_digest==slot['digest'],
                    authority=self.retained_authority(slot,now),settled=self.settled(now,drift_limit),proof=problem is None)
        self.settled_recovery_check=dict(checks=checks,proof_problem=problem,odom_age=now-self.odom_stamp,
                                        speed=float(np.linalg.norm(self.v)))
        if not all(checks.values()):return False
        begun=time.monotonic();box=slot['box'];margin=.10/np.sqrt(3.)
        lower=box.min(0)-margin;upper=box.max(0)+margin
        final=np.array(slot['plan'].sample(slot['plan'].duration))
        for reference in (final,self.reference):
            paths=actual_stop_paths(self.p,self.v,reference[2],reference[1])
            corridor=getattr(slot['plan'],'stop_corridor',None)
            inside=(contains_stops(corridor,paths) if corridor is not None else
                    all(np.all((q>=lower)&(q<=upper)) for q in paths))
            if not inside:return False
        finished=float(self.clock()) if self.clock else now+time.monotonic()-begun
        if (not 0<=finished-now<=.15 or not self.settled(finished,drift_limit)
                or self.proof_problem(slot,finished) is not None):return False
        self.stop_seconds=time.monotonic()-begun;self.stop_checked=now
        self.settled_stop_recoveries+=1
        return True

    def settled(self, now,measurement_speed=.035):
        slot=self.active
        if slot is None or not slot.get('tail_settled') or self.p is None or self.reference is None:return False
        end=slot['start']+slot['plan'].duration
        final=np.array(slot['plan'].sample(slot['plan'].duration))
        return bool(now>=end and self.odom_stamp>=end and -.05<=now-self.odom_stamp<=.5
                    and np.linalg.norm(self.p-final[0])<.10 and np.linalg.norm(self.v)<measurement_speed
                    and np.linalg.norm(self.reference[0]-final[0])<.10
                    and np.linalg.norm(self.reference[1])<.035 and np.linalg.norm(self.reference[2])<.06)

    def release(self, now):
        # A normal landing handoff is allowed only from the actual settled
        # reference. A stale parent reference can never seize moving control.
        if self.error:raise ValueError('Cannot release a fault')
        if self.source_gap is not None:raise ValueError('Source interruption requires fresh continuous input')
        if self.holding and (self.reference is None or max(np.linalg.norm(self.v),np.linalg.norm(self.reference[1]))>=.035 or np.linalg.norm(self.reference[2])>=.06):
            raise ValueError('Release before coordinated brake settles')
        if self.active:
            if now<self.active['start']+self.active['plan'].duration or self.p is None or max(np.linalg.norm(self.v),np.linalg.norm(self.reference[1]))>=.035:
                raise ValueError('Release before terminal stop')
            final=np.array(self.active['plan'].sample(self.active['plan'].duration))
            if (-.05<=now-self.odom_stamp<=.5 and np.linalg.norm(self.p-final[0])<.10
                    and np.linalg.norm(self.reference[0]-final[0])<.10 and np.linalg.norm(self.reference[2])<.06):
                self.released_tail=dict(digest=self.active['digest'],position=final[0].copy())
        self.active=None;self.prepared=None;self.leases={}
        self.holding=False;self.brake=None
        self.stop_checked=None;self.stop_digest=None

    def protect_source_gap(self,now):
        # Only a previously certified, completed and measured-settled finite
        # tail grants this permission. Never invent continuation for moving
        # passthrough, takeoff or landing references without a finite path.
        r=self.reference;tail=self.released_tail
        if (not self.recover_source_gaps or tail is None or r is None
                or np.linalg.norm(self.p-tail['position'])>=.10
                or np.linalg.norm(r[0]-tail['position'])>=.10
                or np.linalg.norm(self.v)>.10 or np.linalg.norm(r[1])>=.035
                or np.linalg.norm(r[2])>=.06):return False
        self.source_gap=dict(since=now,digest=tail['digest'],source_stamp=self.source_stamp)
        self.reference=np.array([r[0].copy(),np.zeros(3),np.zeros(3)])
        return True

    def gap_reference(self,now):
        # A stationary reference has the same P/V at every odometry timestamp.
        # Keep the original tracking bounds; source recovery cannot mask drift.
        if np.linalg.norm(self.p-self.reference[0])>TRACKING_LIMIT or np.linalg.norm(self.v)>.10:
            self.fail('Source protection tracking exceeded reservation',now)
            return self.sample(now)
        now=float(self.clock()) if self.clock is not None else now
        if not -.05<=now-self.odom_stamp<=.5:
            self.fail('Expired/future odometry',now);return self.sample(now)
        fresh=self.source is not None and self.source_stamp>=self.source_gap['since'] and -.05<=now-self.source_stamp<=.3
        if fresh and np.all(np.linalg.norm(self.source-self.reference,axis=1)<np.array([.10,.035,.06])):
            self.source_gap=None;self.source_recoveries+=1
            self.reference=self.source.copy()
        return self.reference.copy()

    def sample(self, now):
        dt=.02 if self.tick is None else max(0.,now-self.tick)
        if self.tick is not None and now<self.tick:self.fail('Clock moved backwards',now)
        self.tick=now
        if self.p is None:return None
        if not self.armed:
            self.reference=np.array([self.p,np.zeros(3),np.zeros(3)])
            return self.reference.copy()
        if self.error is None and not -.05<=now-self.odom_stamp<=.5:self.fail('Expired/future odometry',now)
        candidate=self.prepared
        if self.error is None and candidate and candidate['committed'] and now>=candidate['start']:
            problem=self.proof_problem(candidate,now)
            if self.perception and not self.perception.launch_ready(candidate,now):
                problem=problem or 'Launch observation expired before switch'
            if self.active:problem=problem or self.proof_problem(self.active,now)
            if self.active and self.active['digest'] not in self.leases.get(candidate['digest'],{}).get('digests',[]):
                problem='Switch proof omits predecessor reservation'
            if problem:self.fail(problem,now)
            else:
                if self.active:
                    self.switches+=1;self.last_switch=dict(stamp=candidate['start'],old=self.active['digest'],
                        new=candidate['digest'],speed=float(np.linalg.norm(candidate['plan'].sample(0.)[1])))
                self.previous=self.active;self.active=candidate;self.prepared=None
                self.source_gap=None;self.released_tail=None
                self.leases={candidate['digest']:self.leases[candidate['digest']]}
        if self.error is None and self.active:
            slot=self.active;plan=slot['plan'];problem=self.proof_problem(slot,now)
            if problem:self.fail(problem,now)
            else:
                measured_slot=self.previous if self.previous is not None and self.odom_stamp<slot['start'] else slot
                measured_plan=measured_slot['plan']
                measured=min(measured_plan.duration,max(0.,self.odom_stamp-measured_slot['start']))
                expected=measured_plan.sample(measured)
                error=float(np.linalg.norm(self.v-expected[1]))
                if error>measured_slot['velocity_limit']:
                    self.fail('Measured velocity invalidates stopping reservation',now,velocity_error=error,
                              velocity_limit=measured_slot['velocity_limit'],actual_velocity=self.v.tolist(),
                              nominal_velocity=expected[1].tolist())
                elif np.linalg.norm(self.p-expected[0])>TRACKING_LIMIT:self.fail('Measured position exceeds tracking reservation',now)
                else:
                    reference=np.array(plan.sample(min(plan.duration,max(0.,now-slot['start']))))
                    if not self.local_stopping or self.check_local_stop(slot,now,reference):
                        self.reference=reference
                        authority_key=(slot['run'],slot['epoch'],slot['digest'],slot['start'])
                        if self.retained_authority(slot,now) and now-self.authority.get(authority_key,-1e9)>.35:self.authority_reuse_ticks+=1
                        if self.perception and any(now-self.leases[slot['digest']].get(k,-1e9)>.6 for k in ('sensor_stamp','environment_checked')):
                            self.perception_reuse_ticks+=1
                        if (now>=slot['start']+plan.duration and self.odom_stamp>=slot['start']+plan.duration
                                and np.linalg.norm(self.v)<.035 and np.linalg.norm(self.p-reference[0])<.10
                                and np.linalg.norm((self.p-reference[0])[:2])<.04
                                and np.linalg.norm(reference[1])<.035 and np.linalg.norm(reference[2])<.06):
                            slot['tail_settled']=True
                        return self.reference.copy()
        if self.error is not None or self.holding:
            if self.brake is None:return None
            # The same bounded smooth brake as the certified mission model.
            while dt>1e-9:
                step=min(.02,dt);p,v,a=self.brake
                w,b=smooth_velocity(v,a,np.zeros(3),step,vertical_speed=7.,vertical_acc=2.5,vertical_jerk=8.)
                self.brake=np.array([p+(v+w)*step*.5,w,b]);dt-=step
            self.reference=self.brake.copy();return self.reference.copy()
        if self.source_gap is not None:return self.gap_reference(now)
        if self.source is None or not -.05<=now-self.source_stamp<=.3:
            if self.protect_source_gap(now):return self.gap_reference(now)
            self.fail('Mission source heartbeat expired',now,source_stamp=self.source_stamp,
                      recover_source_gaps=self.recover_source_gaps,
                      released_tail=None if self.released_tail is None else self.released_tail['digest'],
                      measured_speed=float(np.linalg.norm(self.v)),
                      reference_speed=None if self.reference is None else float(np.linalg.norm(self.reference[1])),
                      reference_acceleration=None if self.reference is None else float(np.linalg.norm(self.reference[2])),
                      tail_position_error=None if self.released_tail is None else float(np.linalg.norm(self.p-self.released_tail['position'])))
            return self.sample(now)
        if self.released_tail is not None and (np.linalg.norm(self.source[0]-self.released_tail['position'])>=.10
                or np.linalg.norm(self.source[1])>=.035 or np.linalg.norm(self.source[2])>=.06):
            self.released_tail=None
        self.reference=self.source.copy();return self.reference.copy()

    def peer_state(self):
        if self.p is None:return None
        value=dict(drone_id=self.identity,stamp=self.odom_stamp,position=self.p.tolist(),velocity=self.v.tolist(),
                   failure=None if self.error is None else self.error[:256],holding=self.holding)
        if self.reference is not None:
            value['control_reference']=dict(position=self.reference[0].tolist(),velocity=self.reference[1].tolist(),
                                            acceleration=self.reference[2].tolist())
        return value

    def status(self, now):
        slot=self.active
        permissions=[] if self.perception is None else [self.perception.describe(s,now) for s in (self.active,self.prepared) if s is not None]
        if (self.perception and slot and self.previous and slot['predecessor']==self.previous['digest']
                and slot['run']==self.previous['run'] and slot['start']<=now<=slot['start']+.35):
            old=self.perception.describe(self.previous,now)
            if old is not None:permissions.append(dict(old,handoff_to=slot['digest'],handoff_start=slot['start']))
        return dict(stamp=now,armed=self.armed,error=self.error,detail=self.detail,holding=self.holding,
                    observation=self.peer_state(),
                    perception_enabled=self.perception is not None,
                    perception_source=None if self.perception is None else self.perception.guard.source,
                    perception_permissions=[p for p in permissions if p is not None],
                    perception_reuse_ticks=self.perception_reuse_ticks,
                    authority_reuse_ticks=self.authority_reuse_ticks,
                    authority_stamp=None if self.authority_stamp < 0 else self.authority_stamp,
                    active=None if slot is None else slot['digest'],
                    active_start=None if slot is None else slot['start'],
                    tail_settled=self.settled(now),
                    prepared=None if self.prepared is None else self.prepared['digest'],
                    prepared_committed=bool(self.prepared and self.prepared['committed']),
                    switches=self.switches,last_switch=self.last_switch,
                    reference=None if self.reference is None else self.reference.tolist(),
                    odom_stamp=self.odom_stamp,source_stamp=self.source_stamp,
                    stop_checked=self.stop_checked,stop_seconds=self.stop_seconds,settled_stop_recoveries=self.settled_stop_recoveries,
                    source_gap=self.source_gap,source_recoveries=self.source_recoveries)
