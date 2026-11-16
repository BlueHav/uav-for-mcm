"""Current-map acceptance of a future joint packet while the active one flies."""
from concurrent.futures import ThreadPoolExecutor
import numpy as np
from terminal_client import TerminalClient
from terminal_formation import packet_paths, stop_clouds, environment_clear, tracking_reservation
from terminal_shape import formation_kept
from terminal_corridor import packet_conflict, unknown_view_heading, validated_box
from functools import wraps


def serialized(method):
    @wraps(method)
    def locked(self,*args,**kwargs):
        with self.stop_lock:return method(self,*args,**kwargs)
    return locked


def prepare_packet(packet,member):
    plans=packet_paths(packet)
    if not formation_kept(plans,flexible=packet.get('flexible',False)):raise ValueError('Future packet loses common moving star')
    plan=plans[member];box=validated_box(plan,packet['envelopes'][member])
    return plan,box,tracking_reservation(plan)[0]


class ContinuousTerminalClient(TerminalClient):
    rolling=True
    def __init__(self,mission,clock=None):
        super().__init__(mission,clock)
        self.pool=ThreadPoolExecutor(max_workers=1);self.preparing=None;self.candidate=None
        self.active_box=None;self.previous_digest=None;self.previous_tracking=None;self.switches=0
        self.reference_headers=None
        self.local_output_digest=None
    def close(self):
        worker=getattr(self,'proposal_worker',None)
        if worker is not None:worker.close()
        self.pool.shutdown(wait=False)

    def note_reference(self,command,stamp,now):
        # Called only after outer formation validation on the receive callback.
        # This is authority heartbeat, never a map ACK or trajectory install.
        headers={}
        if isinstance(command,dict) and np.isfinite(stamp) and stamp<=now+.05:
            slots=(command.get('active'),command.get('next')) if command.get('kind')=='stream' else (command,)
            for slot in slots:
                if not isinstance(slot,dict) or slot.get('kind')!='commit':continue
                packet=slot.get('packet',{})
                if self.ownership(packet) and isinstance(packet.get('digest'),str) and isinstance(slot.get('start'),(float,int)):
                    headers[(packet['digest'],slot['start'])]=stamp
        executor=getattr(self.m,'executor',None)
        if (executor is not None and not getattr(self.m,'terminal_executor',False)
                and hasattr(executor,'publish_authority')):executor.publish_authority(headers,stamp)
        # Direct executor authority never waits for parent preparation. But
        # Parent headers change atomically. No heartbeat, request, or wait can
        # retire an owner; only a successor or explicit mission handoff can.
        with self.stop_lock:
            self.reference_headers=headers

    def reference_time(self):
        if self.reference_headers is None:return super().reference_time()
        return self.reference_headers.get((None if self.packet is None else self.packet['digest'],self.start),-1e9)

    def proof_reference_time(self):
        stamp=self.reference_time();c=self.candidate
        if self.reference_headers is not None and c and c.get('committed'):
            # Once a successor is committed its withdrawal is a coordination
            # fault: never use the predecessor heartbeat to authorize switch.
            stamp=min(stamp,self.reference_headers.get((c['packet']['digest'],c['start']),-1e9))
        return stamp

    @serialized
    def promote(self,now):
        c=self.candidate;executor=getattr(self.m,'executor',None)
        if not c or now<c['start']:return
        if executor is None:
            if not c.get('committed'):return
            proof=self.stop_proof or {}
            if (not self.stop_inside or not -.05<=now-self.stop_checked<=.15
                    or not all(d in proof.get('digests',()) for d in self.proof_digests())
                    or not -.05<=now-self.proof_reference_time()<=.35):
                self.abort('Local successor missing fresh joint stopping proof');return
        if not c.get('committed'):
            from static_perception import matching_permission
            state=executor.state
            if ('plan' not in c or state.get('active')!=c['packet']['digest'] or state.get('active_start')!=c['start']
                    or not matching_permission(state,c['packet'],c['start'],now,getattr(self.m,'context_hash',None))):return
            c['committed']=True
        if executor is not None and executor.state.get('active')!=c['packet']['digest']:return
        self.previous_digest=self.packet['digest']
        self.previous_tracking=(self.plan,self.start,self.velocity_limit)
        self.packet=c['packet'];self.plan=c['plan'];self.start=c['start']
        self.active_box=c['box'];self.box=c['box'];self.velocity_limit=c['velocity_limit']
        self.accepted=True;self.committed=True;self.done=False;self.error=None
        self.last_serial=self.packet['serial'];self.native_packets+=1;self.switches+=1
        # The existing proof was explicitly for both envelopes and remains
        # dated from its original snapshot across promotion.
        self.candidate=None

    @serialized
    def receive(self,command,now):
        if not isinstance(command,dict):return
        if self.clock is not None:now=float(self.clock())
        self.promote(now)
        if command.get('kind')!='stream':
            before=None if self.packet is None else self.packet['digest']
            super().receive(command,now)
            if self.packet and self.packet['digest']!=before:
                self.active_box=self.box
                self.previous_tracking=None;self.previous_digest=None
            return
        active=command.get('active',{'kind':'wait'})
        packet=active.get('packet') or {}
        # While the coordinator awaits the other four switch ACKs it still
        # carries the predecessor plus committed successor. Do not replay it.
        if packet.get('digest')==self.previous_digest and self.committed:self.received=now
        else:self.receive(active,now)
        offer=command.get('next')
        if not offer:
            if self.candidate and not self.candidate.get('committed'):
                self.candidate=None;self.box=self.active_box
            return
        self.receive_candidate(offer,now)

    @serialized
    def receive_candidate(self,offer,now):
        packet=offer.get('packet',{});digest=packet.get('digest')
        if self.packet and digest==self.packet['digest']:return
        if not self.committed or self.m.failure:return
        if self.candidate is None or self.candidate['packet'].get('digest')!=digest:
            if self.candidate and self.candidate.get('committed'):return
            self.candidate=dict(packet=packet,start=offer.get('start',-1e9),accepted=False,committed=False,
                                checked=-1e9,error=None,predecessor=offer.get('predecessor'))
            self.box=self.active_box
        c=self.candidate
        try:
            if not self.ownership(packet) or packet.get('serial',0)<=self.last_serial:raise ValueError('Future ownership/replay')
            if c['predecessor']!=self.packet['digest'] or offer.get('predecessor')!=c['predecessor']:raise ValueError('Future predecessor mismatch')
            if offer.get('start')!=c['start']:raise ValueError('Future switch time changed')
            if not c['committed'] and (digest,c['start']) in self.perception_revoked:
                c['accepted']=False
                c['checks']=dict(c.get('checks') or {},clear=False)
                c['error']='New obstacle in uncommitted successor'
                return
            if c['committed']:
                executor=getattr(self.m,'executor',None)
                if executor is not None:executor.prepare(packet,c['start'],predecessor=c['predecessor'],commit=True)
                return
            if offer.get('kind')=='commit' and self.perception_recovery_enabled():
                from static_perception import matching_permission
                clear='plan' in c and self.observed_clear(packet,c['start'],np.vstack((self.active_box,c['box'])),now)
                if (clear and (digest,c['start']) not in self.perception_revoked and .05<=c['start']-now<=1. and matching_permission(self.m.executor.state,
                        packet,c['start'],now,self.m.context_hash,committed=False)):
                    c['accepted']=True;c['committed']=True;self.box=np.vstack((self.active_box,c['box']))
                    self.m.executor.prepare(packet,c['start'],predecessor=c['predecessor'],commit=True);return
            if now>c['start']-.05:raise ValueError('Future preparation arrived too late')
            if 'plan' not in c:
                if self.preparing is None or self.preparing[0]!=digest:
                    self.preparing=(digest,self.pool.submit(prepare_packet,packet,self.m.id%5))
                    return
                if not self.preparing[1].done():return
                plan,box,velocity_limit=self.preparing[1].result()
                t=c['start']-self.start
                if t<=0.:raise ValueError('Future switch outside current horizon')
                if t>=self.plan.duration:
                    executor=getattr(self.m,'executor',None)
                    if not (self.done and (executor is None or executor.state.get('tail_settled'))):
                        raise ValueError('Future post-tail switch before measured settlement')
                if np.max(np.linalg.norm(np.array(self.plan.sample(min(t,self.plan.duration)))-np.array(plan.sample(0.)),axis=1))>1e-5:
                    raise ValueError('Future PVA discontinuity')
                c.update(plan=plan,box=box,velocity_limit=velocity_limit);self.preparing=None
                if getattr(self.m,'terminal_executor',False):
                    end=min(2.,getattr(plan,'nominal_duration',plan.duration))
                    direction=plan.sample(end)[0][:2]-plan.sample(0.)[0][:2]
                    if np.linalg.norm(direction)>.15:
                        c['observe_heading']=float(np.arctan2(direction[1],direction[0]))
            check=float(self.clock()) if self.clock is not None else now
            union=np.vstack((self.active_box,c['box']))
            clear=self.observed_clear(packet,c['start'],union,check)
            c.pop('coverage_heading',None)
            fresh=self.fresh_observation(check);ready=fresh and clear
            c['checks']=dict(clear=bool(clear),fresh=bool(fresh),executor_prepared=False,cache_ready=False)
            if not clear and getattr(self.m,'terminal_executor',False):
                grid=self.m.map
                member=self.m.id%5
                detail={}
                conflict=packet_conflict(grid,packet,member,detail,self.m.p)
                c['checks']['map_refusal']=detail or dict(kind='unknown_or_outside')
                if self.done and not conflict:
                    heading=unknown_view_heading(grid,packet,member,self.m.p)
                    if heading is not None:c['coverage_heading']=heading
            executor=getattr(self.m,'executor',None)
            if ready and executor is not None:
                ready=executor.prepare(packet,c['start'],predecessor=c['predecessor'])
                c['checks']['executor_prepared']=bool(ready)
                if ready and self.perception_recovery_enabled():ready=self.precertify(packet,c['start'],union,check,c['predecessor'])
                c['checks']['cache_ready']=bool(ready)
            elif executor is None:c['checks']['executor_prepared']=bool(ready)
            finished=float(self.clock()) if self.clock is not None else now
            c['checked']=check;c['accepted']=bool(ready and -.05<=finished-check<=.35 and self.fresh_observation(finished))
            c['checks'].update(seconds=finished-check,finished_fresh=bool(self.fresh_observation(finished)))
            c['error']=None if c['accepted'] else 'Future map/executor not ready'
            if c['accepted']:self.box=union
            if offer.get('kind')=='commit':
                if not c['accepted'] or not .05<=c['start']-finished<=1.:raise ValueError('Future commit expired/not ready')
                c['committed']=True
                if executor is not None:executor.prepare(packet,c['start'],predecessor=c['predecessor'],commit=True)
        except (ValueError,KeyError,TypeError) as exc:
            c['accepted']=False;c['error']=str(exc)
            if not c['committed']:self.box=self.active_box

    def discard_blocked_preparation(self,now):
        candidate=self.candidate
        observation=self.map_observation
        executor=getattr(self.m,'executor',None)
        if not (candidate and candidate['accepted'] and not candidate['committed']
                and observation is not None and observation.grid is not None and executor is not None):
            return False
        packet=candidate['packet']
        digest=packet['digest']
        start=candidate['start']
        if (self.reference_headers or {}).get((digest,start)) is not None:
            return False
        if executor.state.get('prepared')==digest and executor.state.get('prepared_committed'):
            return False
        from terminal_corridor import packet_clear,packet_conflict
        member=self.m.id%5
        detail={}
        if not packet_clear(observation.grid,self.packet,member) or not packet_conflict(observation.grid,packet,member,detail,self.m.p):
            return False
        self.perception_revoked.add((digest,start))
        candidate['accepted']=False
        candidate['checks']=dict(clear=False,map_refusal=detail)
        candidate['error']='New obstacle in uncommitted successor'
        self.box=self.active_box
        executor.send_environment(dict(valid=False,run_id=packet['run_id'],epoch=2,digests=[digest],checked=now))
        return True

    def proof_digests(self):
        result=[] if self.packet is None else [self.packet['digest']]
        if self.candidate and self.candidate.get('accepted') and self.candidate.get('committed'):result.append(self.candidate['packet']['digest'])
        return result

    @serialized
    def monitor(self,now):
        if self.clock is not None:now=float(self.clock())
        self.promote(now)
        super().monitor(now)

    def reference(self,now):
        # Initial packet construction can own stop_lock for hundreds of ms.
        # With no commitment there is no terminal reference to read; do not
        # stall the independent executor's ordinary source heartbeat.
        if not self.committed:return None
        with self.stop_lock:return self._committed_reference(now)

    def _committed_reference(self,now):
        # A caller may have waited behind a map checkpoint/client transition.
        # Judge proof age and sample the path at the real post-lock time.
        if self.clock is not None:now=float(self.clock())
        if (not self.executor_stopping and self.independent_environment
                and self.start is not None and .04<=now-self.stop_checked<=.15):
            # The command writer can refresh from the last immutable sensor
            # boundary when fusion or the checker timer delays the next proof.
            super().monitor(now)
            if self.clock is not None:now=float(self.clock())
        self.promote(now)
        value=super().reference(now)
        if value is not None and not self.executor_stopping:
            self.local_output_digest=self.packet['digest']
        return value

    @serialized
    def local_execution_status(self,now):
        active=None
        if self.committed and self.start is not None and now>=self.start:
            if self.local_output_digest==self.packet['digest']:active=self.local_output_digest
        candidate=self.candidate
        prepared=candidate['packet']['digest'] if candidate and candidate['accepted'] else None
        return dict(stamp=now,odom_stamp=self.m.odom_stamp,active=active,prepared=prepared,
                    tail_settled=bool(active and self.done),error=self.error if not self.committed else None)

    def measurement_reference(self,stamp):
        if self.previous_tracking and stamp<self.start:
            plan,start,limit=self.previous_tracking
            return plan.sample(min(plan.duration,max(0.,stamp-start))),limit
        return super().measurement_reference(stamp)

    def status(self,now):
        value=super().status(now);c=self.candidate
        worker=getattr(self,'proposal_worker',None)
        if worker is not None:value['proposal_worker']=worker.status()
        value.update(switches=self.switches,candidate=None if c is None else dict(digest=c['packet'].get('digest'),
                     accepted=c['accepted'],committed=c['committed'],check_stamp=c['checked'],start=c['start'],error=c['error'],checks=c.get('checks')))
        return value

    @serialized
    def coordination_status(self,now):
        # Small control-plane snapshot. Never serialize images, map tiles,
        # optimizer hints or coefficients on the admission path.
        import copy
        if self.clock is not None:now=float(self.clock())
        self.promote(now)
        value=self.status(now)
        keys=('digest','accepted','start','commit_received','request_start','perception_waiting',
              'check_stamp','checks','committed','done','error','scanned','switches','candidate')
        result=dict(stamp=now,run_id=self.m.handoff.run_id,epoch=2,
                    joint=copy.deepcopy({k:value[k] for k in keys}))
        if not self.executor_stopping and self.m.planning_mode=='YOPO':
            result['execution']=self.local_execution_status(now)
        return result
