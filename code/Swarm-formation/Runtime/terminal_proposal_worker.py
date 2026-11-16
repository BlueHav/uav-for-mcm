"""One speculative leader search on an isolated map; never a command writer."""
import collections,threading,time
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace
import numpy as np
from terminal_hint import usable_hint


def planning_view(m,cancelled):
    view=object.__new__(type(m))
    scalars=('id','count','planning_mode','planner_epoch','phase','mode','height','speed','yaw','heading',
             'ground','width','height_px','device','last_safe','last_detour','detour_candidate_cursor','buffer_escape_active',
             'terminal_member_policy')
    for name in scalars:
        if hasattr(m,name):setattr(view,name,getattr(m,name))
    for name in ('p','v','refp','refv','refa','rot','goal','desired','detour'):
        value=getattr(m,name,None);setattr(view,name,None if value is None else np.array(value,copy=True))
    # Inference-only objects: no publishers, callbacks, handoff or executors.
    for name in ('policy','state_transform','lattice_primitive'):
        if hasattr(m,name):setattr(view,name,getattr(m,name))
    view.lock=threading.RLock();view.maplock=threading.RLock()
    view.terminal_client=None;view.executor=None;view.peer=None;view.failure=None
    view.inferences=0;view.chosen=np.empty((0,3));view.last_candidates=[];view.rejects=collections.Counter()
    view.frames={name:(stamp,depth.copy(),(pose[0].copy(),pose[1].copy()))
                 for name,(stamp,depth,pose) in list(m.frames.items())}
    grid=object.__new__(type(m.map))
    for name in ('state','ground','origin','shape','position'):
        value=np.array(getattr(m.map,name),copy=True);value.flags.writeable=False;setattr(grid,name,value)
    if hasattr(m.map,'peer_only'):
        grid.peer_only=m.map.peer_only.copy();grid.peer_only.flags.writeable=False
    for name in ('resolution','dynamic_clearance','ground_stamp','depth_stamp'):
        setattr(grid,name,getattr(m.map,name))
    grid.dynamic_peers=np.empty((0,3));grid._detour_grid_cache=None;grid.last_detour_stats={}
    view.map=grid
    begun=time.monotonic()
    view.terminal_checkpoint=lambda:not cancelled.is_set() and time.monotonic()-begun<=.65
    return view


def search(view,token,stamp):
    from terminal_client import TerminalClient
    view.five=SimpleNamespace(applied_terminal_reference={'terminal_intent_token':token})
    client=TerminalClient(view);client.scanned=True;client.goal_token=token
    client.propose(view.p,view.v,stamp)
    fields={name:getattr(view,name) for name in ('heading','detour','last_detour','chosen','last_candidates')}
    for name in ('detour_candidate_cursor','buffer_escape_active'):
        if hasattr(view,name):fields[name]=getattr(view,name)
    return dict(intent=client.intent,focus=client.focus,fields=fields,inferences=view.inferences,rejects=dict(view.rejects))


class LeaderProposalWorker:
    def __init__(self):
        self.pool=ThreadPoolExecutor(max_workers=1);self.pending=None;self.generation=0;self.key=None
        self.lock=threading.RLock();self.closed=False;self.submitted=0;self.completed=0;self.rejected=0;self.error=None
        self.rejection_reasons=collections.Counter();self.last_result=None;self.last_front_stamp=None
    def invalidate(self):
        with self.lock:
            self.generation+=1
            if self.pending:self.pending['cancelled'].set()
    def close(self):
        with self.lock:self.closed=True;self.invalidate()
        self.pool.shutdown(wait=False)
    def owner(self,m,token):
        return (m.handoff.run_id,m.planner_epoch,m.planning_mode,m.phase,token,tuple(m.goal))
    def allowed(self,m):
        return (not self.closed and m.planning_mode=='YOPO' and m.planner_epoch==2 and m.armed
                and m.phase in ('TERRAIN_FOLLOW','END_HOLD') and not m.failure and not m.stop_pending
                and not m.controller_fault and m.five.active and not m.five.suspended)
    def step(self,client,now,token):
        with self.lock:return self._step(client,now,token)
    def _step(self,client,now,token):
        m=client.m;key=self.owner(m,token)
        if self.key!=key or not self.allowed(m):
            self.invalidate();self.key=key;self.last_front_stamp=None
        job=self.pending
        if job and job['future'].done():
            self.pending=None;self.completed+=1
            try:
                result=job['future'].result();m.inferences+=result['inferences']
                intent=result['intent']
                if intent is not None:
                    intent['sensor_stamp']=job['sensor_stamp']
                    if job.get('static'):
                        intent['seed_identity']=dict(run_id=job['key'][0],epoch=job['key'][1],world_hash=job['world'])
                checks=dict(owner=self.allowed(m) and job['generation']==self.generation and job['key']==key,
                            uncancelled=not job['cancelled'].is_set(),job_fresh=-.05<=now-job['stamp']<=.7,
                            sensor_fresh=-.05<=now-job['sensor_stamp']<=.6,has_intent=intent is not None,
                            world=job.get('world')==getattr(m,'context_hash',None),
                            hint_relevant=usable_hint(intent,m.p,now,token,m.handoff.run_id,getattr(m,'context_hash',None),
                                static=bool(getattr(getattr(m,'executor',None),'state',{}).get('perception_enabled'))))
                # Freshness at search launch is unchanged. A delayed result in
                # the guarded static world can remain a spatially local seed;
                # only later current-map admission may authorize its trajectory.
                valid=all(v for k,v in checks.items() if k not in ('sensor_fresh','job_fresh'))
                self.last_result=dict(checks=checks,job_age=now-job['stamp'],sensor_age=now-job['sensor_stamp'])
                if valid:
                    # Worker owns every result object; these become main-loop
                    # state only after identity/spatial validation succeeds.
                    m.__dict__.update(result['fields']);m.rejects.update(result['rejects'])
                    client.focus=result['focus'];client.intent=result['intent'];client.intent_stamp=job['stamp']
                    self.error=None
                else:
                    self.rejected+=1;self.rejection_reasons.update(k for k,v in checks.items() if not v and k not in ('sensor_fresh','job_fresh'))
            except Exception as exc:self.error=repr(exc);self.rejected+=1
        terminal=getattr(m,'terminal_executor',False)
        if self.pending or not self.allowed(m) or now-client.intent_stamp<(1./15 if terminal else .3):return
        # Recheck the already clear successor before spending CPU on another
        # speculative search. A refusal or withdrawal reopens search immediately.
        candidate=getattr(client,'candidate',None) or {}
        if (terminal and not candidate.get('committed')
                and (candidate.get('checks') or {}).get('clear') is True
                and 0.<candidate['start']-now<=.55):
            return
        stamps=[m.map.ground_stamp,m.map.depth_stamp]+[m.handoff.integrated.get(k) for k in ('front','up','down')]
        if any(s is None or not np.isfinite(s) for s in stamps):return
        sensor=min(stamps)
        if not -.05<=now-sensor<=.6:return
        front=m.frames.get('front')
        if terminal and (front is None or front[0]!=stamps[2] or front[0]==self.last_front_stamp):return
        cancelled=threading.Event();view=planning_view(m,cancelled)
        if terminal:self.last_front_stamp=front[0]
        self.pending=dict(key=key,generation=self.generation,cancelled=cancelled,stamp=now,sensor_stamp=sensor,
                          world=getattr(m,'context_hash',None),
                          static=bool(getattr(getattr(m,'executor',None),'state',{}).get('perception_enabled')),
                          future=self.pool.submit(search,view,token,now))
        self.submitted+=1
    def status(self):
        return dict(pending=self.pending is not None,submitted=self.submitted,completed=self.completed,rejected=self.rejected,error=self.error,
                    rejection_reasons=dict(self.rejection_reasons),last_result=self.last_result)
