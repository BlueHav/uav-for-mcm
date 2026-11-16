"""Per-aircraft YOPO proposal, observed-map ACK, and joint reference reader."""
import math
import time
import threading
from contextlib import nullcontext
import numpy as np
from terminal_formation import packet_paths,stop_clouds,environment_clear,TRACKING_LIMIT,tracking_reservation
from terminal_stopping import actual_stop_paths
from terminal_shape import formation_kept

class TerminalClient:
    def __init__(self,mission,clock=None):
        self.m=mission;self.packet=None;self.plan=None;self.box=None;self.accepted=False
        # Production supplies the ROS clock; omitted clocks retain the explicit
        # receive(command, now) contract used by deterministic/replay callers.
        self.clock=clock;self.reject_detail=None;self.checks=None
        self.start=None;self.committed=False;self.received=-1e9;self.checked=-1e9
        self.error=None;self.done=False;self.scanned=False;self.scan_angle=0.;self.last_yaw=None
        self.intent=None;self.intent_stamp=-1e9;self.focus=np.zeros(3);self.native_packets=0;self.last_serial=0
        self.goal_token=None
        self.velocity_limit=.10
        self.stop_stamp=None;self.stop_inside=False;self.stop_checked=-1e9;self.monitored=-1e9;self.stop_reference=None
        self.abort_detail=None
        self.stop_lock=threading.RLock()
        self.independent_environment=False;self.environment=None;self.stop_proof=None;self.environment_generation=0
        self.executor_stopping=False
        self.observation_lock=threading.RLock();self.map_observation=None
        self.perception_revoked=set()
        self.awaiting_commit_since=None
        self.request_start=None
        self.launch_scan=None
    def stationary(self):
        m=self.m
        return max(np.linalg.norm(m.v),np.linalg.norm(m.refv))<.035 and np.linalg.norm(m.refa)<.06
    def start_matches(self):
        if self.plan is None:return False
        p,v,a=self.plan.sample(0.);m=self.m
        return self.stationary() and max(np.linalg.norm(m.p-p),np.linalg.norm(m.refp-p))<.10 and np.linalg.norm(v)<.035 and np.linalg.norm(a)<.06
    def ownership(self,packet):
        m=self.m
        return m.planning_mode=='YOPO' and m.planner_epoch==2 and packet.get('epoch')==2 and packet.get('run_id')==m.handoff.run_id and packet.get('group')==m.id//5
    def perception_recovery_enabled(self):
        return bool(getattr(getattr(self.m,'executor',None),'state',{}).get('perception_enabled'))
    def fresh_observation(self,now):
        if self.m.sensor_ready() is not None:return False
        if not self.executor_stopping:return True
        stamps=[getattr(self.m.map,'ground_stamp',None),getattr(self.m.map,'depth_stamp',None)]
        stamps += [getattr(self.m.handoff,'integrated',{}).get(c) for c in ('front','up','down')]
        return all(s is not None and np.isfinite(s) and -.05<=now-s<=.6 for s in stamps)
    def perception_allowed(self,now):
        from static_perception import matching_permission
        if not self.packet or not -.05<=now-self.m.odom_stamp<=.5:return False
        if not self.committed:return self.perception_waiting(now)
        if (self.packet['digest'],self.start) in self.perception_revoked:return False
        state=getattr(getattr(self.m,'executor',None),'state',{});world=getattr(self.m,'context_hash',None)
        if matching_permission(state,self.packet,self.start,now,world):return True
        # The global commit is already accepted. Give its asynchronous local
        # receipt the original350ms activation allowance, never a new lease.
        return bool(self.awaiting_commit_since is not None and 0<=now-self.awaiting_commit_since<=.35
            and now<=self.start+.35 and matching_permission(state,self.packet,self.start,now,world,committed=False))
    def perception_waiting(self,now):
        # Receipt protection at an already certified stationary start does not
        # authorize trajectory motion; only the executor's commit can do that.
        from static_perception import matching_permission
        # The executor cache is the historical admission evidence. A repeated
        # stale request must lose its fresh ACK without losing safe waiting.
        return bool(not self.committed and self.packet and self.request_start is not None
            and now<=self.request_start+.35 and -.05<=now-self.m.odom_stamp<=.5 and self.start_matches()
            and (self.packet['digest'],self.request_start) not in self.perception_revoked
            and matching_permission(getattr(getattr(self.m,'executor',None),'state',{}),self.packet,self.request_start,now,
                                    getattr(self.m,'context_hash',None),committed=False))
    def launch_scans_ready(self,packet,start,depth_stamp):
        key=(packet['digest'],start)
        if depth_stamp<packet['created']:
            return False
        if self.launch_scan is None or self.launch_scan[0]!=key:
            self.launch_scan=(key,depth_stamp)
            return False
        return depth_stamp-self.launch_scan[1]>=.12

    def precertify(self,packet,start,box,now,predecessor=None):
        executor=getattr(self.m,'executor',None)
        if executor is None or (packet['digest'],start) in self.perception_revoked or not self.fresh_observation(now):return False
        depth_stamps=[self.m.handoff.integrated[c] for c in ('front','up','down')]
        stamps=[self.m.map.ground_stamp,self.m.map.depth_stamp]+depth_stamps
        if getattr(self.m,'terminal_executor',False):
            from static_perception import launch_observation_fresh
            if (not launch_observation_fresh(packet['created'],dict(sensor_stamp=min(stamps),environment_checked=now),now)
                    or not self.launch_scans_ready(packet,start,min(depth_stamps))):
                return False
        if not executor.prepare(packet,start,predecessor=predecessor):return False
        slots=[[packet['digest'],start]]
        if predecessor:slots.insert(0,[self.packet['digest'],self.start])
        value=dict(kind='environment',run_id=packet['run_id'],epoch=packet['epoch'],valid=True,
            digests=[s[0] for s in slots],environment_slots=slots,lower=box.min(0).tolist(),upper=box.max(0).tolist(),
            checked=now,environment_checked=now,sensor_stamp=min(stamps),reference_stamp=self.proof_reference_time(),
            world_hash=self.m.context_hash,environment_generation=self.environment_generation)
        executor.send_environment(value)
        from static_perception import matching_permission
        return matching_permission(executor.state,packet,start,now,self.m.context_hash,committed=False)
    def observed_clear(self,packet,start,box,now):
        from terminal_corridor import owned_clear,owned_conflict
        clear=owned_clear(self,self.m.map,packet) if 'corridors' in packet else environment_clear(self.m.map,box)
        if not clear and self.launch_scan is not None and self.launch_scan[0]==(packet['digest'],start):
            self.launch_scan=None
        if not clear and self.perception_recovery_enabled() and owned_conflict(self,self.m.map,packet):
            self.perception_revoked.add((packet['digest'],start))
            self.m.executor.send_environment(dict(valid=False,run_id=packet['run_id'],epoch=packet['epoch'],
                                                digests=[packet['digest']],checked=now))
        return clear
    def receive(self,command,now):
        if not isinstance(command,dict):return
        if self.clock is not None:now=float(self.clock())
        kind=command.get('kind');packet=command.get('packet')
        if getattr(self,'rolling',False) and self.committed:
            if kind=='wait' or kind in ('request','commit') and packet and packet.get('digest')!=self.packet['digest']:
                # Keep the owner. An ordinary request cannot substitute for a
                # successor handshake, even at a measured stationary tail.
                self.error='Waiting for explicit successor or mission handoff';return
        if kind=='wait':
            if self.committed and not self.done:self.abort('Executing packet lost its commit reference')
            else:
                # A completed finite polynomial has no live commitment once
                # the coordinator releases it. Hold at rest while replanning.
                with self.stop_lock:
                    self.received=now;self.committed=False;self.accepted=False;self.checks=None
                    self.stop_inside=False;self.stop_checked=-1e9;self.stop_stamp=None
            return
        if kind=='abort':self.abort('Joint coordinator cancelled packet');return
        if kind not in ('request','commit') or not isinstance(packet,dict) or not self.ownership(packet):
            self.accepted=False;self.error='Invalid terminal ownership';return
        if self.packet is None or packet.get('digest')!=self.packet.get('digest'):
            if not isinstance(packet.get('serial'),int) or packet['serial']<=self.last_serial or not -.05<=now-packet.get('created',-1e9)<=4.:
                self.accepted=False;self.error='Expired/replayed joint proposal';return
            if self.committed and not self.done:self.abort('New packet before previous stop');return
            self.packet=packet;self.accepted=False;self.error=None;self.start=None;self.committed=False;self.done=False
            self.reject_detail=None;self.checks=None
            self.stop_stamp=None;self.stop_inside=False;self.stop_checked=-1e9;self.monitored=-1e9
            try:
                plans=packet_paths(packet)
                if not formation_kept(plans,flexible=packet.get('flexible',False)):raise ValueError('Joint packet does not preserve the common star')
                self.plan=plans[self.m.id%5]
                from terminal_corridor import validated_box
                self.box=validated_box(self.plan,packet['envelopes'][self.m.id%5])
                self.velocity_limit,_=tracking_reservation(self.plan)
                supplied=np.asarray(packet['envelopes'][self.m.id%5],float)
                if supplied.shape!=self.box.shape or not np.allclose(supplied,self.box,atol=1e-6):raise ValueError('Stopping box mismatch')
                self.last_serial=packet['serial']
            except (KeyError,ValueError,TypeError) as exc:
                self.plan=None;self.error=str(exc);return
        if self.plan is None:return
        self.received=now
        # Immutable packet/envelope preparation is not observation validation.
        # Date the actual state/map check here, never at its completion.
        validation_started=float(self.clock()) if self.clock is not None else now
        if kind=='request':
            self.request_start=command.get('start')
            self.accepted=False;self.reject_detail=None;self.checks=None
            clear=self.observed_clear(self.packet,command.get('start'),self.box,validation_started)
            self.checks=dict(clear=bool(clear))
            if not clear and getattr(self.m,'terminal_executor',False):
                from terminal_corridor import packet_conflict
                detail={}
                packet_conflict(self.m.map,self.packet,self.m.id%5,detail,self.m.p)
                self.checks['map_refusal']=detail or dict(kind='unknown_or_outside')
            ready=bool(self.scanned and self.start_matches() and self.fresh_observation(validation_started)
                       and clear)
            if ready and self.perception_recovery_enabled():
                ready=self.precertify(self.packet,command.get('start',-1e9),self.box,validation_started)
            # Odometry and sensor callbacks can advance while the map check
            # runs. Do not ACK a start that moved or sensors that became stale.
            ready=bool(ready and self.start_matches() and self.fresh_observation(float(self.clock()) if self.clock else now))
            finished=float(self.clock()) if self.clock is not None else now
            self.checked=validation_started
            self.accepted=bool(ready and -.05<=finished-self.checked<=.35)
            self.error=None if self.accepted else 'Start/map/scan not ready'
        elif not self.committed:
            start=command.get('start',-1e9)
            if self.perception_recovery_enabled():
                from static_perception import matching_permission
                execution=self.m.executor.state
                digest=self.packet['digest']
                executor_owned=(execution.get('prepared')==digest and execution.get('prepared_committed')
                                or execution.get('active')==digest and execution.get('active_start')==start)
                if (getattr(self.m,'terminal_executor',False) and executor_owned
                        and validation_started<=start+.35 and matching_permission(
                            execution,self.packet,start,validation_started,self.m.context_hash)):
                    self.start=float(start);self.committed=True;self.accepted=True;self.native_packets+=1
                    self.awaiting_commit_since=validation_started
                    return
                clear=self.observed_clear(self.packet,start,self.box,validation_started)
                if (clear and (self.packet['digest'],start) not in self.perception_revoked
                        and self.start_matches() and validation_started+.05<=start<=validation_started+1.
                        and matching_permission(self.m.executor.state,self.packet,start,validation_started,self.m.context_hash,committed=False)):
                    self.start=float(start);self.committed=True;self.accepted=True;self.native_packets+=1
                    self.awaiting_commit_since=validation_started
                    self.m.executor.prepare(self.packet,start,commit=True);return
            matches=self.start_matches()
            if not self.accepted or not -.05<=validation_started-self.checked<=.35 or not validation_started+.05<=start<=validation_started+1. or not matches:
                self.reject_commit('Commit expired or changed start',validation_started,start,matches);return
            if not self.fresh_observation(validation_started) or not self.observed_clear(self.packet,start,self.box,validation_started):
                finished=float(self.clock()) if self.clock is not None else now
                self.reject_commit('Commit map stale/blocked',finished,start,self.start_matches());return
            matches=self.start_matches();sensor_error=self.m.sensor_ready()
            finished=float(self.clock()) if self.clock is not None else now
            # Never re-date the ACK or execute a commit that expired while its
            # current map was being checked. Preserve both original windows.
            if not -.05<=finished-self.checked<=.35 or not finished+.05<=start<=finished+1. or not matches:
                self.reject_commit('Commit expired or changed start',finished,start,matches);return
            if sensor_error is not None:
                self.reject_commit('Commit map stale/blocked',finished,start,matches);return
            self.reject_detail=None
            self.start=float(start);self.committed=True;self.native_packets+=1
    def reject_commit(self,reason,now,start,matches):
        # Preserve the first rejection for this attempt: repeated commits would
        # otherwise obscure the cause with the accepted=False consequence.
        if self.reject_detail is None:
            self.reject_detail=dict(reason=reason,now=now,check_stamp=self.checked,
                                    check_age=now-self.checked,start=start,start_delta=start-now,
                                    accepted=self.accepted,start_matches=bool(matches),
                                    ack_fresh=bool(-.05<=now-self.checked<=.35),
                                    start_window=bool(now+.05<=start<=now+1.))
        self.accepted=False;self.error=reason
    def abort(self,reason):
        with self.stop_lock:
            self.abort_detail=dict(reason=reason,plan_stage=getattr(self.m,'plan_stage',None),
                                   plan_stage_age=time.monotonic()-getattr(self.m,'plan_stage_started',time.monotonic()),
                                   stop_checked=self.stop_checked,stop_inside=self.stop_inside,stop_stamp=self.stop_stamp)
            self.abort_detail.update(plan_profile=dict(getattr(self.m,'plan_profile',{})),
                                     previous_plan_profile=dict(getattr(self.m,'previous_plan_profile',{})))
            observation=self.map_observation
            self.abort_detail['environment']=None if observation is None else dict(
                generation=observation.generation,checked=observation.checked,sensor_stamp=observation.sensor_stamp,
                proof=None if self.stop_proof is None else {k:self.stop_proof.get(k) for k in
                    ('checked','sensor_stamp','environment_generation','environment_checked')})
            self.committed=False;self.accepted=False;self.error=reason;self.m.desired[:]=0.;self.m.braking=True
            self.stop_stamp=None;self.stop_inside=False;self.stop_checked=-1e9
        self.m.fail('Terminal joint packet: '+reason)
    def reference_time(self):return self.received
    def proof_reference_time(self):return self.reference_time()
    def reference(self,now):
        if not self.committed or self.start is None:return None
        m=self.m
        if m.failure or m.controller_fault or m.stop_pending or not m.armed:return None
        if self.executor_stopping:
            # The independent writer checks its own fresh odometry, tracking,
            # stopping corridor and exact committed authority. This parent is
            # an observer: delayed planner/status IPC must not cancel that
            # owner or perform a second check with differently timed samples.
            state=m.executor.state
            if (state.get('error') or not -.05<=now-state.get('stamp',-1e9)<=.35
                    or state.get('active')!=self.packet['digest'] or state.get('active_start')!=self.start):return None
            value=np.asarray(state.get('reference',[]),float)
            if value.shape!=(3,3) or not np.isfinite(value).all():return None
            self.done=bool(self.done or state.get('tail_settled'))
            return value.copy()
        if not -.05<=now-self.reference_time()<=.35 or (m.sensor_ready() is not None and not self.perception_allowed(now)):
            self.abort('Reference or observed sensors expired');return None
        if now<self.start:return None
        if not self.executor_stopping and (not self.stop_inside or not -.05<=now-self.stop_checked<=.15):
            self.expire_stop_certificate(now);return None
        elapsed=min(now-self.start,self.plan.duration)
        with getattr(m,'lock',nullcontext()):
            position=m.p.copy();velocity=m.v.copy();stamp=m.odom_stamp
        measured_reference,velocity_limit=self.measurement_reference(stamp)
        velocity_error=float(np.linalg.norm(velocity-measured_reference[1]))
        if velocity_error>velocity_limit:
            self.abort('Measured velocity invalidates actual stopping certificate')
            self.abort_detail.update(velocity_error=velocity_error,velocity_limit=velocity_limit,
                                     actual_velocity=velocity.tolist(),nominal_velocity=measured_reference[1].tolist(),odom_stamp=stamp)
            return None
        if np.linalg.norm(position-measured_reference[0])>TRACKING_LIMIT:
            self.abort('Joint polynomial tracking exceeded 0.15m');return None
        if now-self.start>=self.plan.duration:
            # The star certificate is horizontal; retain the prior vertical
            # tracking allowance while tightening XY for the next start.
            error=m.p-self.plan.sample(self.plan.duration)[0]
            # Completion is a historical ACK consumed asynchronously by the
            # coordinator. Keep it monotonic for this digest; current tracking
            # checks above and every next proposal's start_matches still apply.
            self.done=bool(self.done or (self.stationary() and np.linalg.norm(error[:2])<.04 and np.linalg.norm(error)<.10))
        return self.plan.sample(elapsed)
    def measurement_reference(self,stamp):
        return self.plan.sample(min(max(0.,stamp-self.start),self.plan.duration)),self.velocity_limit
    def expire_stop_certificate(self,now):
        self.abort('Measured actual stopping certificate expired/outside reserved envelope')
        self.abort_detail['certificate_age']=now-self.abort_detail['stop_checked']
    def capture_observation(self,now):
        from terminal_environment import observe
        if self.committed:observe(self,now)
    def capture_environment(self,now,refresh=True):
        # Caller owns maplock at a completed update boundary. No map pointers
        # escape; a blocked fusion operation cannot mutate this observation.
        from terminal_environment import capture
        if self.committed:
            if refresh:self.capture_observation(now)
            self.environment=capture(self,now)
    def monitor(self,now):
        # ROS timers catch up after a delayed callback. Never skip that fresh
        # state merely because the preceding check happened less than 40ms ago.
        if not self.committed:return
        if self.executor_stopping:
            self.monitor_environment(now);return
        begun=time.monotonic()
        if self.clock is not None:now=float(self.clock())
        active=self.start is not None and now>=self.start
        previous_checked=self.stop_checked
        # The controller may not have been scheduled during a long map update.
        # A checkpoint must not revive a certificate that already expired.
        if active and (not self.stop_inside or not -.05<=now-previous_checked<=.15):
            self.expire_stop_certificate(now);return
        environment=None
        if self.independent_environment:
            from terminal_environment import covers,owns_execution
            if not owns_execution(self):return
            self.capture_environment(now,refresh=False);environment=self.environment
            if not covers(environment,self):return
            if not environment.clear:self.abort('New obstacle in certified stopping box');return
            if not all(-.05<=now-s<=.6 for s in (environment.sensor_stamp,environment.checked)):
                self.abort('Completed environment observation expired');return
        self.monitored=now;m=self.m;packet=self.packet;box=self.box
        with getattr(m,'lock',nullcontext()):
            p=m.p.copy();v=m.v.copy();stamp=m.odom_stamp
            reference=getattr(m,'last_control_reference',None)
            refv=m.refv.copy() if reference is None else np.array(reference['velocity'])
            refa=m.refa.copy() if reference is None else np.array(reference['acceleration'])
            checked_reference=dict(position=(m.refp.copy() if reference is None else np.array(reference['position'])).tolist(),
                                   velocity=refv.tolist(),acceleration=refa.tolist(),
                                   stamp=now if reference is None else reference['stamp'])
        # Heavy full-brake integration stays on the planning timer. The sole
        # 50 Hz writer only checks age and paired odometry/reference bounds.
        paths=actual_stop_paths(p,v,refa,refv)
        margin=.10/np.sqrt(3.) # <= 0.10 m Euclidean expansion per aircraft.
        lower=box.min(0)-margin;upper=box.max(0)+margin
        inside=all(np.all((q>=lower)&(q<=upper)) for q in paths)
        if not inside:self.abort('Measured actual stopping path exceeds reserved envelope');return
        if environment is None and not environment_clear(m.map,self.box):self.abort('New obstacle in certified stopping box');return
        with self.stop_lock:
            if not self.committed or self.packet is not packet or self.box is not box or m.failure:return
            if environment is not None:
                # The dynamic input/box has not changed. Recheck a newer
                # completed map instead of starving every clear-map update.
                self.capture_environment(now,refresh=False);environment=self.environment
                if not owns_execution(self) or not covers(environment,self):return
                if not environment.clear:self.abort('New obstacle in certified stopping box');return
            with self.observation_lock:
                if environment is not None and environment.generation!=self.environment_generation:return
                self.commit_stop_proof(now,begun,active,previous_checked,stamp,checked_reference,packet,environment)

    def monitor_environment(self,now):
        from terminal_environment import covers,owns_execution
        if self.clock is not None:now=float(self.clock())
        if not owns_execution(self):return
        self.capture_environment(now,refresh=False);environment=self.environment
        if not covers(environment,self):return
        discard=getattr(self,'discard_blocked_preparation',None)
        if environment.conflict and discard is not None and discard(now):
            self.capture_environment(now,refresh=False);environment=self.environment
            if not covers(environment,self):return
        if not environment.clear:
            # A rolling-map crop/unknown cell cannot erase prior observation
            # of an unchanged static scene. It still cannot certify a new path.
            if not environment.conflict and self.perception_recovery_enabled():return
            executor=getattr(self.m,'executor',None)
            if executor is not None:
                executor.send_environment(dict(valid=False,run_id=self.packet['run_id'],epoch=2,
                    digests=list(self.proof_digests() if hasattr(self,'proof_digests') else [self.packet['digest']]),checked=now))
            conflict={}
            if self.map_observation is not None and self.map_observation.grid is not None:
                from terminal_corridor import packet_conflict,owned_packets
                for packet in owned_packets(self):
                    if packet_conflict(self.map_observation.grid,packet,self.m.id%5,conflict):
                        conflict['digest']=packet['digest']
                        live_map=self.m.map
                        cell=tuple(np.floor(np.asarray(conflict['position'])/live_map.resolution).astype(int))
                        conflict['near_camera_return']=getattr(live_map,'near_depth_returns',{}).get(cell)
                        break
            self.abort('New obstacle in certified stopping box')
            self.abort_detail['obstacle']=conflict
            return
        if not all(-.05<=now-s<=.6 for s in (environment.sensor_stamp,environment.checked)):
            # Do not grant/refresh anything from stale input. The independent
            # executor alone decides whether its admitted cache still permits
            # the finite trajectory; it reports a real revocation as a fault.
            if all(s<=now+.05 for s in (environment.sensor_stamp,environment.checked)) and self.perception_recovery_enabled():return
            self.abort('Completed environment observation expired');return
        with self.observation_lock:
            if environment.generation!=self.environment_generation or not owns_execution(self):return
            # Accepted successors have already passed their own scan and
            # preparation checks. A new complete union observation refreshes
            # both exact slots even while the successor is awaiting authority.
            # The executor still cannot switch without the group commit.
            digests=[digest for digest,_ in environment.slots]
            self.stop_inside=True
            self.stop_proof=dict(kind='environment',run_id=self.packet['run_id'],epoch=self.packet['epoch'],
                digests=digests,checked=environment.checked,
                sensor_stamp=environment.sensor_stamp,reference_stamp=self.proof_reference_time(),valid=True,
                environment_checked=environment.checked,environment_slots=environment.slots,
                lower=environment.lower,upper=environment.upper,environment_generation=environment.generation,
                world_hash=getattr(self.m,'context_hash',None))

    def commit_stop_proof(self,now,begun,active,previous_checked,stamp,checked_reference,packet,environment):
        if environment is not None:
            from terminal_environment import owns_execution
            if not owns_execution(self):return
        finished=float(self.clock()) if self.clock is not None else now+time.monotonic()-begun
        # Date the new proof from its actual input snapshot, not completion.
        # Active execution needs the previous proof valid throughout this
        # computation too. Before start, the first proof may be established.
        if self.start is not None and finished>=self.start and (
                not -.05<=finished-now<=.15 or
                active and not -.05<=finished-previous_checked<=.15):
            self.expire_stop_certificate(finished);return
        if environment is not None and not all(-.05<=finished-s<=.6 for s in (environment.sensor_stamp,environment.checked)):
            self.abort('Completed environment observation expired');return
        self.stop_inside=True;self.stop_stamp=stamp;self.stop_checked=now;self.stop_reference=checked_reference
        if environment is not None:
            self.stop_proof=dict(run_id=packet['run_id'],epoch=packet['epoch'],
                digests=list(self.proof_digests() if hasattr(self,'proof_digests') else [packet['digest']]),
                checked=now,odom_stamp=stamp,reference_stamp=self.proof_reference_time(),
                sensor_stamp=environment.sensor_stamp,valid=True,reference=checked_reference,
                environment_checked=environment.checked,environment_slots=environment.slots,
                environment_generation=environment.generation)
    def propose(self,p,v,now):
        m=self.m;m.desired[:]=0.
        # A local completion ACK does not release the all-five commitment.
        # Starting YOPO here would hold the map lock and starve the independent
        # stopping timer while slower members are still settling.
        # An accepted request already has a reserved proposal. Keep refreshing
        # its current-map ACK in receive(), instead of spending the same map
        # cycle on an unused replacement and starving sensor integration.
        if (self.accepted and not self.committed) or (self.committed and not getattr(self,'rolling',False)):return
        token=(getattr(getattr(m,'five',None),'applied_terminal_reference',None) or {}).get('terminal_intent_token')
        if not isinstance(token,str):return
        if token!=self.goal_token:
            self.goal_token=token;self.intent=None;self.intent_stamp=-1e9
            m.detour=None;m.last_detour=0.
        if not self.scanned:
            if self.last_yaw is not None:
                self.scan_angle+=max(0.,math.atan2(math.sin(m.yaw-self.last_yaw),math.cos(m.yaw-self.last_yaw)))
            self.last_yaw=m.yaw
            if self.scan_angle<2*math.pi+.1:
                m.heading=m.yaw+.7;m.reason='YOPO terminal: observing full local neighborhood while hovering';return
            self.scanned=True
        worker=getattr(self,'proposal_worker',None)
        if worker is not None:
            worker.step(self,float(self.clock()) if self.clock is not None else now,token)
            return
        if now-self.intent_stamp<.3:return
        # This temporary peer-free map is used only to propose environmental
        # directions. No resulting desired velocity reaches the controller.
        peers=m.map.dynamic_peers;m.map.dynamic_peers=np.empty((0,3))
        saved=m.goal.copy();saved_desired=m.desired.copy()
        try:
            # Early return while yawing or observing must not re-date a path
            # left over from the preceding goal or phase.
            m.chosen=np.empty((0,3));m.last_candidates=[]
            if m.id%5==2 or getattr(m,'terminal_member_policy',False):
                m.horizontal_plan(p,v)
            else:
                # The coordinator consumes only the leader's directions.
                # Followers still provide their OWN observed ground and ACK
                # every trajectory against their own obstacle/stop map.
                delta=m.goal[:2]-p[:2];length=np.linalg.norm(delta)
                if length>.01:
                    m.heading=math.atan2(delta[1],delta[0])
                    self.focus=np.r_[delta/length,0.]
            local_reason=getattr(m,'reason',None)
            chosen=m.chosen.copy();candidates=list(m.last_candidates)
            choices=[chosen]+candidates
            directions=[];curves=[]
            for path in choices:
                if len(path)<2:continue
                d=np.asarray(path[-1])[:2]-p[:2];length=np.linalg.norm(d)
                if length<.1:continue
                d=d/length
                if not any(np.dot(d,old)>.999 for old in directions):
                    directions.append(d)
                    curve=np.asarray(path,float)
                    indices=np.linspace(0,len(curve)-1,min(31,len(curve))).round().astype(int)
                    curves.append(curve[indices].tolist())
                if len(directions)>=16:break
            origin=np.floor(p[:2]/2.)*2.-30.
            x,y=np.meshgrid(origin[0]+np.arange(31)*2.,origin[1]+np.arange(31)*2.,indexing='ij')
            heights=m.map.ground_at_many(np.c_[x.ravel(),y.ravel()]).reshape(31,31)
            from terminal_observation import obstacle_hints
            self.intent=dict(stamp=now,token=token,directions=[d.tolist() for d in directions],curves=curves,
                             obstacles=obstacle_hints(m.map,p),ground_origin=origin.tolist(),
                             search_reason=local_reason,
                             ground_resolution=2.,
                             ground=np.where(np.isfinite(heights),heights,1e9).tolist(),position=p.tolist(),inferences=m.inferences)
            self.intent_stamp=now
            if directions:self.focus=np.r_[directions[0],0.]
        finally:
            m.map.dynamic_peers=peers;m.goal=saved;m.desired=saved_desired
        m.desired[:]=0.;m.reason='YOPO environmental candidates; waiting for jointly certified formation packet'
    def status(self,now):
        with self.stop_lock:
            state=getattr(getattr(self.m,'executor',None),'state',{}) if self.executor_stopping else {}
            checked=state.get('stop_checked');stamp=state.get('odom_stamp')
            inside=bool(checked is not None and state.get('active') and not state.get('error') and -.05<=now-checked<=.15)
            return dict(digest=None if self.packet is None else self.packet['digest'],accepted=self.accepted,
                        flexible=bool(self.packet and self.packet.get('flexible')),
                        start=self.start,
                        commit_received=self.awaiting_commit_since,
                        request_start=self.request_start,perception_waiting=self.perception_waiting(now),
                        check_stamp=self.checked,committed=self.committed,done=self.done,error=self.error,
                        scanned=self.scanned,scan_angle=self.scan_angle,intent=self.intent,native_packets=self.native_packets,
                        actual_stop_inside=inside if self.executor_stopping else self.stop_inside,
                        stop_check_age=(None if checked is None else now-checked) if self.executor_stopping else now-self.stop_checked,
                        stop_odom_stamp=stamp if self.executor_stopping else self.stop_stamp,abort_detail=self.abort_detail,
                        reject_detail=self.reject_detail,checks=self.checks,reference_stamp=self.reference_time())
