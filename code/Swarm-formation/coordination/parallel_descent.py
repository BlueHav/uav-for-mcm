"""Group-aware regional descent scheduling. Run only on the SSH server."""
import itertools
import math
import numpy as np
from coordination_config import validate_config
from shared_policy import CFG, Policy, stop_bound


class GroupScheduler:
    def __init__(self, config):
        self.config = validate_config(config)
        self.ids = self.config['vehicle_ids']
        self.routes = {i:self.config['routes'][g['route_id']] for g in self.config['groups'] for i in g['members']}
        self.lengths = {i:float(np.linalg.norm(np.subtract(r['handoff'], r['entry']))) for i,r in self.routes.items()}
        self.policies = {pair:Policy() for pair in itertools.combinations(self.ids,2)}
        self.region_since = {}
        self.region_passed = {i:False for i in self.ids}
        self.caps = {}
        self.generation = {i:0 for i in self.ids}
        self.changed = {}
        self.previous_stamps = {}

    @staticmethod
    def _descent(phase):
        return phase == 'DESCENT' or str(phase).startswith('DESCENT_')

    def decide(self, states, phases, now, goals=None):
        out = dict(allow={i:False for i in self.ids},speed_cap_3d={i:0. for i in self.ids},
                   cap_generation=dict(self.generation),group_progress={},region_passed=dict(self.region_passed),pair_decisions={},requested_speed_3d={},hard_fault=None)
        try:
            if not math.isfinite(now) or set(states) != set(self.ids) or set(phases) != set(self.ids):
                raise ValueError('missing_or_unknown_active_member')
            positions, velocities, stamps, progress = {}, {}, [], {}
            frames = set()
            for i in self.ids:
                s=states[i];p=np.asarray(s['p'],float);v=np.asarray(s['v'],float);stamp=float(s['stamp'])
                if p.shape!=(3,) or v.shape!=(3,) or not np.isfinite(np.r_[p,v,stamp]).all():
                    raise ValueError('invalid_global_state')
                if not -.05<=now-stamp<=CFG['odom_max_age'] or stamp<self.previous_stamps.get(i,-math.inf):
                    raise ValueError('stale_or_reversed_global_state')
                if np.linalg.norm(v)>15.001:
                    raise ValueError('actual_speed_limit')
                frames.add(s.get('frame'));stamps.append(stamp);positions[i]=p;velocities[i]=v
                route=self.routes[i];entry=np.asarray(route['entry']);delta=np.subtract(route['handoff'],entry)
                progress[i]=float(np.dot(p-entry,delta)/np.dot(delta,delta))
            if len(frames)!=1 or not frames.issubset({'world','map'}) or max(stamps)-min(stamps)>CFG['stamp_skew']:
                raise ValueError('frame_or_timestamp_skew')
            pairs=list(self.policies)
            for i,j in pairs:
                distance=float(np.linalg.norm(positions[i]-positions[j]))
                if not CFG['min_distance']<=distance<=CFG['max_distance']:
                    raise ValueError('actual_distance_outside_2_2000')
            self.previous_stamps={i:float(states[i]['stamp']) for i in self.ids}
            for g in self.config['groups']:
                values=[progress[i] for i in g['members']]
                out['group_progress'][g['id']]={'min':min(values),'max':max(values)}
            active=[i for i in self.ids if self._descent(phases.get(i))]
            for i in self.ids:
                s=states[i]
                agl=float(s.get('agl',float('nan')));agl_stamp=float(s.get('agl_stamp',float('nan')))
                # Original sensor age threshold is 0.6 s; independent from odometry age.
                valid=(i in active and math.isfinite(agl) and math.isfinite(agl_stamp) and -.05<=now-agl_stamp<=.6
                       and abs(agl-5.)<=.25 and np.linalg.norm(positions[i][:2]-self.config['region_center'])<=self.config['region_radius_m'])
                if valid:
                    self.region_since.setdefault(i,now)
                    if now-self.region_since[i]>=.5:self.region_passed[i]=True
                else:self.region_since.pop(i,None)
            allow={i:True for i in self.ids}
            current_goals={}
            for i in self.ids:
                if goals is not None and i in goals:target=goals[i]
                elif i in active:target=self.routes[i]['handoff']
                elif str(phases.get(i)).startswith(('TERMINAL','END')):target=self.routes[i]['end']
                else:target=positions[i]
                current_goals[i]=np.asarray(target,float)
                if current_goals[i].shape!=(3,) or not np.isfinite(current_goals[i]).all():
                    raise ValueError('invalid_current_goal')
            # The original policy remains the hard gate, including every member pair.
            # Before descent, the existing coordinator owns its proven takeoff/cruise schedule.
            if active:
                for (i,j),policy in self.policies.items():
                    mapped={0:states[i],1:states[j]}
                    pair_goals={0:current_goals[i],1:current_goals[j]}
                    decision=policy.decide(mapped,pair_goals,now)
                    out['pair_decisions']['%d:%d'%(i,j)]=decision
                    if decision['hard_fault']:raise ValueError(decision['hard_fault'])
                    allow[i] &= bool(decision['allow'][0]);allow[j] &= bool(decision['allow'][1])
            caps={i:CFG['command_speed'] for i in self.ids}
            if active:
                desired={i:5. for i in active}
                for group in self.config['groups']:
                    members=[i for i in group['members'] if i in active]
                    if len(members)>1:
                        slowest=min(progress[i] for i in members)
                        for i in members:
                            desired[i]*=max(.2,1.-6.*max(0.,progress[i]-slowest-.015))
                out['requested_speed_3d']=dict(desired)

                def feasible(candidate, check_pairs=None):
                    for i,j in (pairs if check_pairs is None else check_pairs):
                        if i not in active and j not in active:continue
                        rel=positions[i]-positions[j]
                        directions={}
                        speeds={}
                        for k in (i,j):
                            speed=float(np.linalg.norm(velocities[k]))
                            speeds[k]=max(speed,candidate.get(k,speed))
                            delta=np.subtract(self.routes[k]['handoff'],positions[k]);norm=float(np.linalg.norm(delta))
                            directions[k]=(delta/max(norm,1e-12))*candidate[k] if k in candidate else velocities[k]
                        # Check both actual velocity and the proposed route velocity; never
                        # pretend an aircraft has already slowed down to its new cap.
                        clearance=float(np.linalg.norm(rel))
                        horizon=max(8.,max(speeds.values())/CFG['brake_acc']+2.)
                        for dv in (velocities[i]-velocities[j],directions[i]-directions[j]):
                            t=float(np.clip(-np.dot(rel,dv)/max(1e-12,np.dot(dv,dv)),0,horizon))
                            clearance=min(clearance,float(np.linalg.norm(rel+dv*t)))
                        guard=CFG['min_distance']+CFG['tracking_margin']+CFG['release_margin']+1.
                        guard+=stop_bound(speeds[i])+stop_bound(speeds[j])
                        if clearance<guard:return False
                    return True

                wanted=dict(desired);constrained=set()
                for _ in range(max(1,3*len(pairs))):
                    bad=[pair for pair in pairs if not feasible(wanted,[pair])]
                    if not bad:break
                    changed=False
                    for pair in bad:
                        affected=[i for i in pair if i in active]
                        constrained.add(pair)
                        zero=dict(wanted);zero.update({i:0. for i in affected})
                        candidate=dict(zero)
                        if feasible(zero,[pair]):
                            lo,hi=0.,1.
                            for _ in range(32):
                                mid=(lo+hi)*.5;trial=dict(wanted)
                                trial.update({i:wanted[i]*mid for i in affected})
                                if feasible(trial,[pair]):lo=mid
                                else:hi=mid
                            candidate=dict(wanted);candidate.update({i:wanted[i]*lo*.95 for i in affected})
                        if not feasible(candidate,[pair]):candidate=zero
                        for i in affected:
                            changed |= abs(wanted[i]-candidate[i])>1e-8
                            wanted[i]=candidate[i]
                            if wanted[i]<=1e-8:allow[i]=False
                    if not changed:break
                # CPA need not improve monotonically when a third member slows.
                # Recheck all pairs and propagate braking only to affected members.
                for _ in range(len(active)+1):
                    bad=[pair for pair in pairs if not feasible(wanted,[pair])]
                    changed=False
                    for pair in bad:
                        constrained.add(pair)
                        for i in pair:
                            if i in active:
                                changed |= wanted[i]>0 or allow[i]
                                wanted[i]=0.;allow[i]=False
                    if not changed:break
                old={i:self.caps.get(i,wanted[i]) for i in active}
                old_safe=feasible(old)
                for i in active:
                    previous=old[i]
                    update=(i not in self.caps or not old_safe or not allow[i] or
                            (now-self.changed.get(i,-math.inf)>=5. and abs(wanted[i]-previous)>=.15))
                    caps[i]=wanted[i] if update else previous
                if not feasible({i:caps[i] for i in active}):
                    for i in active:caps[i]=wanted[i]
                out['distance_constrained_pairs']=[list(pair) for pair in sorted(constrained)]
                if constrained:out['soft_reason']='pair_distance_stopping_constraints'
                out['speed_limit_reasons']={i:[] for i in active}
                for i in active:
                    if desired[i]<5.-1e-8:out['speed_limit_reasons'][i].append('within_group_formation')
                    if caps[i]<desired[i]-1e-8:
                        reason='pair_distance_stopping_constraints' if any(i in pair for pair in constrained) else 'distance_cap_recovery_hysteresis'
                        out['speed_limit_reasons'][i].append(reason)
            for i in self.ids:
                if i not in self.caps or abs(caps[i]-self.caps[i])>1e-8:
                    self.generation[i]+=1;self.changed[i]=now
                self.caps[i]=caps[i]
            out.update(allow=allow,speed_cap_3d=caps,cap_generation=dict(self.generation),region_passed=dict(self.region_passed))
        except (KeyError,ValueError,TypeError,OverflowError) as exc:
            out['hard_fault']=str(exc)
        return out
