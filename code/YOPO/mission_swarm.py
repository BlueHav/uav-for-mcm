"""Native-trajectory, conservative receding-horizon swarm guard.

Motion forecasts finish stopped after 7.5 seconds and contain an explicit sixty
second constant PolyTraj tail. Peers sample those actual polynomials; no status
velocity extrapolation is permitted. The caller must execute reference(now),
including its position/velocity/acceleration, and publish the returned traj.
This module owns no ROS publisher and no controller.
"""
import time
import numpy as np
from swarm_adapter import optimize, to_polytraj, sample_polytraj


class SwarmGuard:
    def __init__(self, drone_id=0, horizon=7.5, hold_tail=60., pending_timeout=2., optimizer_timeout=.3, plan_period=.2):
        self.drone_id=int(drone_id);self.horizon=float(horizon);self.hold_tail=float(hold_tail)
        self.pending_timeout=float(pending_timeout);self.optimizer_timeout=float(optimizer_timeout);self.plan_period=float(plan_period)
        self.agl=None;self.cached_agl=None;self.cached=None;self.cached_start=None;self.last_plan=-float('inf');self.pending_since=None;self.serial=0
        self.cached_message=None;self.fault_reason=None

    def reset_fault(self):
        """Only call for an explicit reset/new grounded mission, never on recovery."""
        self.fault_reason=None;self.pending_since=None
        self.cached=None;self.cached_start=None;self.cached_message=None
        self.last_plan=-float('inf')

    @staticmethod
    def _stamp(value):
        return float(value.to_sec()) if hasattr(value,'to_sec') else float(value)

    def reference(self, now):
        """Evaluate the last certified native motion, including stopped tail."""
        if self.cached is None:return None
        if now < self.cached_start-.1 or now > self.cached_start+sum(self.cached['durations']):return None
        r=self.cached;t=max(0.,float(now)-self.cached_start);ds=r['durations'];coef=r['coefficients'];ends=np.cumsum(ds)
        piece=min(int(np.searchsorted(ends,t,side='right')),len(ds)-1);local=min(t-(0 if piece==0 else ends[piece-1]),ds[piece])
        values=[]
        for order in range(3):
            value=np.zeros(3)
            for power in range(order,6):
                multiplier=1
                for derivative in range(order):multiplier*=power-derivative
                value+=coef[piece,:,5-power]*multiplier*local**(power-order)
            values.append(value)
        return dict(position=values[0],velocity=values[1],acceleration=values[2])

    def _message(self, result, now):
        import rospy
        self.serial+=1
        return to_polytraj(result,self.drone_id,self.serial,rospy.Time.from_sec(now))

    def _stationary(self,p,now):
        coefficients=np.zeros((1,3,6));coefficients[0,:,5]=p
        result=dict(accepted=True,durations=np.array([self.horizon+self.hold_tail]),coefficients=coefficients)
        return self._message(result,now)

    def _reject(self,reason,p,v,now,hard=False):
        if self.pending_since is None:self.pending_since=now
        expired=now-self.pending_since>=self.pending_timeout
        ref=self.reference(now)
        # Continue an already certified braking forecast while negotiating.
        # Never fabricate a new instantaneous zero-velocity polynomial in flight.
        return dict(accepted=False,velocity=np.zeros(3) if ref is None else ref['velocity'],reference=ref,reason=reason,traj=None,bandfailure=bool(hard or expired),pending_seconds=now-self.pending_since)

    def _normal_update(self,p,v,a,desired,map,peer_status,peer_traj,active,now=None):
        now=time.time() if now is None else float(now)
        p,v,a,desired=[np.asarray(x,float) for x in (p,v,a,desired)]
        if any(x.shape!=(3,) or not np.all(np.isfinite(x)) for x in (p,v,a,desired)):
            return self._reject('nonfinite_own_state',p,v,now,True)
        if not active:
            self.cached=None;self.cached_start=None
            self.pending_since=None
            # Bootstrap only an actually stationary vehicle. TAKEOFF motion must
            # not be represented as a fictional stationary peer forecast.
            stationary=np.linalg.norm(v)<.1 and np.linalg.norm(desired)<.1
            msg=self._stationary(p,now) if stationary else None
            return dict(accepted=True,velocity=desired.copy(),reference=None,reason='stationary_native_bootstrap' if stationary else 'outside_band_moving_no_false_bootstrap',traj=msg,bandfailure=False)
        old_reference=self.reference(now)
        if self.cached_agl!=self.agl or old_reference is not None and np.linalg.norm(old_reference['position']-p)>.75:
            self.cached=None;self.cached_start=None;self.last_plan=-float('inf')
        if peer_status is None or now-float(peer_status.get('stamp',-float('inf')))>.6:
            return self._reject('peer_status_stale',p,v,now)
        peer_p=np.asarray(peer_status.get('position',[np.nan]*3),float)
        if peer_p.shape!=(3,) or not np.all(np.isfinite(peer_p)):return self._reject('peer_state_nonfinite',p,v,now,True)
        actual=float(np.linalg.norm(p-peer_p))
        if not 30<=actual<=80:return self._reject('actual_pair_distance_outside_30_80',p,v,now,True)
        if peer_traj is None:return self._reject('native_peer_trajectory_missing',p,v,now)
        try:
            age=now-self._stamp(peer_traj.start_time)
            if age<-.1 or age>.6:raise ValueError('native_peer_trajectory_stale')
            tail=np.array([getattr(peer_traj,name)[-6:] for name in ('coef_x','coef_y','coef_z')],float)
            if peer_traj.duration[-1]<self.hold_tail or np.any(np.abs(tail[:,:5])>1e-8):raise ValueError('peer_missing_explicit_stopped_tail')
            dt=.1;times=np.linspace(0,self.horizon,int(round(self.horizon/dt))+1)
            # Native explicit stopped tail supplies horizon even at nonzero age.
            neighbor=sample_polytraj(peer_traj,times+max(age,0.))
        except (ValueError,TypeError,AttributeError):return self._reject('native_peer_horizon_invalid_or_stale',p,v,now)
        if now-self.last_plan<self.plan_period and self.cached is not None:
            ref=self.reference(now)
            future=np.array([self.reference(now+t)['position'] for t in times])
            distances=np.linalg.norm(future-neighbor,axis=1)
            if np.any((distances<30)|(distances>80)) or not np.all(map.points_clear(future,radius=.35)):
                return self._reject('cached_motion_new_peer_or_map_conflict',p,v,now)
            if self.agl is not None:
                terrain=map.ground_at_many(future[:,:2]);error=np.abs(future[:,2]-terrain-float(self.agl))
                if not np.all(np.isfinite(error)) or np.max(error)>.25:return self._reject('cached_agl_error_or_unknown',p,v,now)
            return dict(accepted=True,velocity=ref['velocity'],reference=ref,reason='cached_certified_polynomial',traj=None,bandfailure=False)
        self.last_plan=now
        # One second near cruise, then conservative stopping. Retry smaller
        # desired progress without increasing measured initial velocity.
        knots=np.array([0.,.5,1.,4.,self.horizon])
        seed=desired.copy()
        if self.agl is not None:seed[2]=0.
        direction=seed/max(np.linalg.norm(seed),1e-9)
        budget=time.perf_counter()+self.optimizer_timeout
        result=None;failure='optimizer_dynamic_limit'
        for scale in (1.,.8,.6):
            remaining=budget-time.perf_counter()
            if remaining<=.005:break
            speed=np.linalg.norm(seed)*scale;distance=[]
            for t in knots:
                cruise=min(t,1.);brake=max(0.,min(t-1.,speed/2.2))
                distance.append(speed*cruise+speed*brake-1.1*brake**2)
            candidate=p+np.asarray(distance)[:,None]*direction
            if self.agl is not None:
                terrain=map.ground_at_many(candidate[:,:2])
                if not np.all(np.isfinite(terrain)):failure='terrain_knots_unknown';continue
                candidate[:,2]=terrain+float(self.agl);candidate[0]=p
            trial=optimize(candidate,np.diff(knots),neighbor[None],dt,start_velocity=v,start_acceleration=a,end_velocity=(0,0,0),end_acceleration=(0,0,0),timeout=remaining,max_acc=2.5,max_vz=1.8)
            if not trial.get('accepted'):failure='optimizer_'+str(trial.get('reason'));continue
            if np.max(np.linalg.norm(trial['accelerations'],axis=1))>2.5 or np.max(np.abs(trial['velocities'][:,2]))>1.8:
                failure='controller_dynamic_reserve';continue
            if np.max(np.linalg.norm(trial['samples']-p,axis=1))>40.:
                failure='optimized_motion_exceeds_local_40m';continue
            if self.agl is not None:
                terrain=map.ground_at_many(trial['samples'][:,:2])
                error=np.abs(trial['samples'][:,2]-terrain-float(self.agl))
                if not np.all(np.isfinite(error)) or np.max(error)>.25:
                    failure='optimized_agl_error_or_unknown';continue
                trial['agl_max_error']=float(np.max(error))
            trial['desired_scale']=scale;result=trial;break
        if result is None:return self._reject(failure,p,v,now)
        # Include every swept sample. Current body occupancy is seeded by caller.
        if not np.all(map.points_clear(result['samples'],radius=.35)):
            return self._reject('optimized_motion_local_unknown_or_obstacle',p,v,now)
        for left,right in zip(result['samples'][::5],result['samples'][5::5]):
            if not map.corridor_clear(left,right,radius=.35):return self._reject('optimized_swept_corridor_blocked',p,v,now)
        endpoint=result['samples'][-1];hold=np.zeros((1,3,6));hold[0,:,5]=endpoint
        result['durations']=np.r_[result['durations'],self.hold_tail];result['coefficients']=np.concatenate([result['coefficients'],hold])
        # After 7.5 seconds both communicated motions are in explicit stopped
        # tails. This endpoint pair remains within the checked hard band.
        self.cached=result;self.cached_start=now;self.cached_agl=self.agl;self.pending_since=None
        ref=self.reference(now);message=self._message(result,now)
        self.cached_message=message
        return dict(accepted=True,velocity=ref['velocity'],reference=ref,reason='real_minco_native_peer_certified',traj=message,result=result,bandfailure=False,pair_distance=actual)

    def _validate_stop(self,result,start,p,v,map,peer_status,peer_traj,now):
        """Recheck the entire remaining stop, including the peer's moving tail.

        A stale peer may be used ONLY as a native, finite committed forecast.
        This is conditional on the peer continuing that commitment (or publishing
        its replacement before execution), not proof against a failed controller.
        The returned flag makes that assumption visible to the caller/evidence.
        Missing/expired data and unexplained fresh tracking errors fail closed.
        """
        if peer_traj is None:return False,'native_peer_trajectory_missing',False
        try:
            if peer_status is None:return False,'peer_measurement_missing',False
            peer_p=np.asarray(peer_status.get('position',[np.nan]*3),float)
            peer_stamp=float(peer_status.get('stamp',np.nan))
            if peer_p.shape!=(3,) or not np.all(np.isfinite(peer_p)) or not np.isfinite(peer_stamp) or peer_stamp>now+.1:
                return False,'peer_measurement_invalid',False
            ds=np.asarray(result['durations'],float)
            if len(ds)<2 or ds[-1]<self.hold_tail:return False,'own_stop_tail_missing',False
            age=now-self._stamp(peer_traj.start_time)
            peer_ds=np.asarray(peer_traj.duration,float)
            tail=np.array([getattr(peer_traj,name)[-6:] for name in ('coef_x','coef_y','coef_z')],float)
            if (not np.all(np.isfinite(peer_ds)) or np.any(peer_ds<=0) or age<-.1 or
                peer_ds[-1]<self.hold_tail or tail.shape!=(3,6) or not np.all(np.isfinite(tail)) or
                np.any(np.abs(tail[:,:5])>1e-8)):
                return False,'peer_stop_tail_invalid',False
            # Never clamp/extrapolate a peer beyond its explicit finite horizon.
            remaining=max(0.,start+sum(ds[:-1])-now,self._stamp(peer_traj.start_time)+sum(peer_ds[:-1])-now)+.1
            if now+remaining>start+sum(ds) or age+remaining>sum(peer_ds):
                return False,'committed_horizon_expired',False
            times=np.linspace(0.,remaining,max(2,int(np.ceil(remaining/.02))+1))
            own=to_polytraj(result,self.drone_id,self.serial,self._stamp_to_ros(start))
            positions=sample_polytraj(own,now-start+times)
            neighbor=sample_polytraj(peer_traj,np.maximum(0.,age)+times)
            # Position and velocity continuity must agree with measured state.
            reference=self._evaluate(result,now-start)
            if np.linalg.norm(reference['position']-p)>.75 or np.linalg.norm(reference['velocity']-v)>.75:
                return False,'own_stop_tracking_error',False
            stale=now-peer_stamp>.6 or age>.6
            if not stale:
                if np.linalg.norm(peer_p-neighbor[0])>.75:
                    return False,'peer_stop_tracking_error',False
                if not 30<=np.linalg.norm(p-peer_p)<=80:return False,'actual_pair_distance_outside_30_80',False
            distance=np.linalg.norm(positions-neighbor,axis=1)
            # Preserve a tracking reserve instead of certifying exactly at a bound.
            if np.any((distance<31.5)|(distance>78.5)):return False,'stop_pair_reserve',stale
            if not np.all(map.points_clear(positions,radius=.35)):return False,'stop_map_conflict',stale
            for i in range(0,len(positions)-1,5):
                if not map.corridor_clear(positions[i],positions[min(i+5,len(positions)-1)],radius=.35):
                    return False,'stop_swept_map_conflict',stale
            if self.agl is not None:
                terrain=map.ground_at_many(positions[:,:2]);error=np.abs(positions[:,2]-terrain-float(self.agl))
                if not np.all(np.isfinite(error)) or np.max(error)>.25:return False,'stop_agl_unknown_or_error',stale
            return True,'stop_revalidated',stale
        except (ValueError,TypeError,AttributeError,IndexError,OverflowError):
            return False,'stop_forecast_invalid',False

    @staticmethod
    def _stamp_to_ros(stamp):
        import rospy
        return rospy.Time.from_sec(float(stamp))

    @staticmethod
    def _evaluate(result,t):
        ds=result['durations'];ends=np.cumsum(ds)
        piece=min(int(np.searchsorted(ends,t,side='right')),len(ds)-1)
        local=t-(0 if piece==0 else ends[piece-1]);coef=result['coefficients'][piece]
        values=[]
        for order in range(3):
            out=np.zeros(3)
            for power in range(order,6):
                factor=1
                for d in range(order):factor*=power-d
                out+=coef[:,5-power]*factor*local**(power-order)
            values.append(out)
        return dict(position=values[0],velocity=values[1],acceleration=values[2])

    def stop(self,p,v,a,map,peer_status,peer_traj,reason,now=None):
        """Sticky coordinated stop; returns one executable native contract.

        accepted means normal progress only. executable/certified govern control.
        Publish traj BEFORE selecting its reference, including committed_stop.
        On unavailable, reference is None and cache is invalidated: caller must
        explicitly report safety_failure and use an uncertified best-effort stop.
        Do not translate fault into zero desired velocity while executing a
        certified stop; fault changes mission phase, not this trajectory.
        """
        now=time.time() if now is None else float(now)
        if self.fault_reason is None:self.fault_reason=str(reason)
        p,v,a=[np.asarray(value,float) for value in (p,v,a)]
        base=dict(accepted=False,fault=True,phasefault='FAULT_HOLD',fault_reason=self.fault_reason,
                  reason=self.fault_reason,bandfailure=True,peer_commitment_assumed=False)
        finite=all(value.shape==(3,) and np.all(np.isfinite(value)) for value in (p,v,a))
        detail='nonfinite_own_state'
        if finite and self.cached is not None:
            ok,detail,assumed=self._validate_stop(self.cached,self.cached_start,p,v,map,peer_status,peer_traj,now)
            if ok:
                if self.cached_message is None:self.cached_message=self._message(self.cached,self.cached_start)
                ref=self.reference(now)
                return dict(base,action='committed_stop',certified=True,executable=True,safety_failure=False,
                    reference=ref,velocity=ref['velocity'],traj=self.cached_message,peer_commitment_assumed=assumed,
                    valid_until=self.cached_start+sum(self.cached['durations']),stop_check=detail)
        # Discard an invalid commitment before trying a measured-state brake.
        self.cached=None;self.cached_start=None;self.cached_message=None
        if finite and peer_traj is not None:
            duration=max(2.,np.linalg.norm(v)*1.8/2.2,abs(v[2])*1.8/1.2)
            knots=np.array([0.,duration*.5,duration]);candidate=p+np.array([0.,.4,.5])[:,None]*duration*v
            try:
                age=max(0.,now-self._stamp(peer_traj.start_time))
                times=np.linspace(0.,duration,int(np.ceil(duration/.1))+1);dt=times[1]-times[0]
                neighbor=sample_polytraj(peer_traj,age+times)
                trial=optimize(candidate,np.diff(knots),neighbor[None],dt,start_velocity=v,start_acceleration=a,
                    end_velocity=(0,0,0),end_acceleration=(0,0,0),timeout=self.optimizer_timeout,max_acc=2.5,max_vz=1.8)
                if trial.get('accepted'):
                    hold=np.zeros((1,3,6));hold[0,:,5]=trial['samples'][-1]
                    trial['durations']=np.r_[trial['durations'],self.hold_tail]
                    trial['coefficients']=np.concatenate([trial['coefficients'],hold])
                    ok,detail,assumed=self._validate_stop(trial,now,p,v,map,peer_status,peer_traj,now)
                    if ok:
                        self.cached=trial;self.cached_start=now;self.cached_agl=self.agl
                        self.cached_message=self._message(trial,now);ref=self.reference(now)
                        return dict(base,action='emergency_stop',certified=True,executable=True,safety_failure=False,
                            reference=ref,velocity=ref['velocity'],traj=self.cached_message,result=trial,
                            peer_commitment_assumed=assumed,valid_until=now+sum(trial['durations']),stop_check=detail)
                else:detail='emergency_'+str(trial.get('reason'))
            except (ValueError,TypeError,AttributeError,IndexError,OverflowError):detail='emergency_peer_forecast_invalid'
        elif finite:detail='native_peer_trajectory_missing'
        return dict(base,action='unavailable',certified=False,executable=False,safety_failure=True,
                    reference=None,velocity=np.zeros(3),traj=None,valid_until=now,stop_check=detail)

    def update(self,p,v,a,desired,map,peer_status,peer_traj,active,now=None):
        now=time.time() if now is None else float(now)
        if self.fault_reason is not None:
            return self.stop(p,v,a,map,peer_status,peer_traj,self.fault_reason,now)
        result=self._normal_update(p,v,a,desired,map,peer_status,peer_traj,active,now)
        if active and not result['accepted']:
            return self.stop(p,v,a,map,peer_status,peer_traj,result['reason'],now)
        executable=bool(active and result.get('reference') is not None)
        result.update(action='normal' if active else 'outside_band',executable=executable,certified=executable,
                      fault=False,phasefault=None,fault_reason=None,safety_failure=False,
                      peer_commitment_assumed=False,
                      valid_until=None if self.cached is None else self.cached_start+sum(self.cached['durations']))
        return result


