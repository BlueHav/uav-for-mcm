"""Map real physical observations to the accepted two-agent safety policy."""
import numpy as np
import time
from shared_policy import Policy,stop_bound

class PairSafety:
    ids=(2,7)
    def __init__(self):self.policy=Policy()
    def decide(self,states,goals,now,stages,mission_states=None,climb_geometry=None):
        logical={i:states[physical] for i,physical in enumerate(self.ids) if physical in states}
        targets={i:goals[physical] for i,physical in enumerate(self.ids) if physical in goals}
        decision=self.policy.decide(logical,targets,now)
        decision['allow']={physical:bool(decision['allow'].get(i,False)) for i,physical in enumerate(self.ids)}
        decision['speed_cap_3d']={physical:10.5 for physical in self.ids}
        decision['cap_generation']={physical:0 for physical in self.ids}
        if climb_geometry is not None and all(stages.get(i) in ('CLIMB','CLIMB_READY') for i in self.ids) and not decision.get('hard_fault'):
            from pair_climb_stopping import climb_stop_certificate
            try:
                began=time.monotonic()
                certificate=climb_stop_certificate(states,mission_states,goals,climb_geometry,now)
                certificate['computation_seconds']=time.monotonic()-began
                if certificate['computation_seconds']>.05:raise ValueError('Parallel climb computation exceeded delay budget')
                decision['parallel_climb_certificate']=certificate
                if certificate['safe']:
                    decision.update(allow={i:True for i in self.ids},reason='parallel_climb_stopping_envelopes_clear')
                else:decision.update(allow={i:False for i in self.ids},reason='parallel_climb_stopping_envelopes_overlap')
            except (ValueError,KeyError,TypeError) as exc:
                decision.update(allow={i:False for i in self.ids},reason='parallel_climb_invalid_evidence: '+str(exc))
            decision['speed_cap_3d']={i:.8 for i in self.ids}
        decision['cruise_admission_ready']=False
        if not decision.get('hard_fault') and len(logical)==2:
            # The second arrival needs room for both measured stopping bounds,
            # independent of which physical ID reached cruise first.
            admission_guard=200.+sum(decision['stop_bounds'])+1.5+5.
            decision['cruise_admission_guard']=admission_guard
            decision['cruise_admission_ready']=bool(decision['distance']>admission_guard)
        if not decision.get('hard_fault') and len(logical)==2 and all(stages.get(i)=='CRUISE' for i in self.ids):
            p=[np.asarray(logical[i]['p'],float) for i in (0,1)];v=[np.asarray(logical[i]['v'],float) for i in (0,1)]
            relative=p[1]-p[0];distance=float(np.linalg.norm(relative));closing=-float(np.dot(relative,v[1]-v[0]))/max(distance,1e-9)
            reserve=200.+sum(stop_bound(float(np.linalg.norm(x)),decision.get('settled_ages',{}).get(i,0.)>=.5) for i,x in enumerate(v))+1.5
            decision['cruise_lower_guard']=reserve
            if distance<=200.:
                decision.update(hard_fault='Actual cruise distance not greater than 200 m',reason='cruise_distance_violation',allow={i:False for i in self.ids})
            elif closing>0 and distance<=reserve:
                decision.update(reason='cruise_lower_distance_brake',allow={i:False for i in self.ids})
        return decision

class CoordinatedClimbBudget:
    """Real elapsed time split by the previously published healthy safety permit."""
    WAIT_REASONS=frozenset(('fixed_priority_1_yield','both_brake_until_loser_stopped_outside_winner_lane','upper_distance_leader_yield','cruise_lower_distance_brake'))
    def __init__(self,member,started,active_limit=180.):
        self.active_limit=float(active_limit);self.wall_limit=self.active_limit+120.
        if not np.isfinite(self.active_limit) or not 0<self.active_limit<=600:raise ValueError('Invalid climb budget limit')
        self.member=member;self.started=float(started);self.last=float(started);self.previous=None
        self.active_seconds=0.;self.wait_seconds=0.;self.wall_seconds=0.
    def observe(self,now,decision,healthy):
        now=float(now);wall=now-self.started
        if not np.isfinite([now,self.started]).all() or now<self.last:raise ValueError('Climb budget time reversed/invalid')
        if wall>self.wall_limit:raise ValueError('Climb wall clock budget exceeded')
        if not healthy or 'hard_fault' not in decision or decision['hard_fault']:raise ValueError('Unhealthy permit cannot grant climb wait credit')
        stamp=decision.get('stamp');seq=decision.get('seq');allows=decision.get('allow',{})
        allowed=allows.get(self.member,allows.get(str(self.member)))
        if (not isinstance(stamp,(int,float)) or not np.isfinite(stamp) or not -.05<=now-stamp<=.2
                or not isinstance(seq,int) or isinstance(seq,bool) or not isinstance(allowed,bool)):
            raise ValueError('Missing/invalid climb permit evidence')
        if self.previous is not None and (now-self.last>.2 or seq<=self.previous['seq']):raise ValueError('Missing/nonmonotonic climb permit history')
        elapsed=now-self.last
        if self.previous is not None and self.previous['coordination_wait']:self.wait_seconds+=elapsed
        else:self.active_seconds+=elapsed
        self.last=now;self.wall_seconds=wall
        self.previous=dict(seq=seq,stamp=stamp,allowed=allowed,reason=decision.get('reason'),coordination_wait=not allowed and decision.get('reason') in self.WAIT_REASONS)
        if self.wait_seconds>120.:raise ValueError('Climb coordination wait budget exceeded')
        if self.active_seconds>self.active_limit:raise ValueError('Phase timeout: CLIMB (effective budget exceeded)')
    def status(self):
        return dict(started=self.started,last_observed=self.last,wall_seconds=self.wall_seconds,active_seconds=self.active_seconds,
                    coordination_wait_seconds=self.wait_seconds,active_limit=self.active_limit,coordination_wait_limit=120.,wall_limit=self.wall_limit,last_permit=self.previous)
