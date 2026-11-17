"""Two leader workers only; optimization never blocks the command timer."""
import copy
from concurrent.futures import ThreadPoolExecutor
import numpy as np
from swarm_native_bridge import NativePlanner, NativeTrajectory

def goal_boundary(p,v,target,speed=6.,turning=False):
    from cloud_config import NATIVE_HORIZON,NATIVE_ACCELERATION
    horizon=NATIVE_HORIZON if speed>6 else 18.;acceleration=NATIVE_ACCELERATION if speed>6 else 1.2
    direction=target-p;length=float(np.linalg.norm(direction[:2]));unit=direction[:2]/max(length,1e-9)
    forward=max(0.,float(np.dot(v[:2],unit)))
    lateral=float(np.linalg.norm(v[:2]-unit*forward))
    if turning and speed>6. and lateral>.5:
        # A high-speed bearing change also needs room to remove lateral
        # velocity. The original 80 m failed the unchanged native acceleration
        # certificate at the recorded cruise-to-descent turn.
        horizon=max(horizon,NATIVE_HORIZON+2.*speed*lateral/NATIVE_ACCELERATION)
    if forward>speed:
        # A short fixed horizon forced the 12->4 m/s MINCO boundary to loop
        # backwards into delayed followers. Reserve braking and jerk room.
        horizon=max(horizon,(forward**2-speed**2)/(2*acceleration)+forward*.75+3.)
    goal=p+direction*min(1.,horizon/max(length,1e-9));goal[2]=270.
    distance=min(horizon,length)
    # A stopped vehicle cannot reach 6 m/s in a short remaining segment.
    # Reserve margin for zero acceleration at the MINCO boundary. At cruise
    # speed this leaves the ordinary 6 m/s boundary unchanged.
    reserve=.83 if speed>16. else .7 if speed>6 else .8
    reachable=reserve*np.sqrt(forward**2+2*acceleration*distance)
    return goal,np.r_[unit*min(speed,reachable),0.]

class CloudLeaderPlanner:
    def __init__(self,binary):
        self.binary=binary;self.native=NativePlanner(binary);self.pool=ThreadPoolExecutor(max_workers=1)
        self.pending=None;self.active=None;self.next_plan=None;self.previous=None;self.epoch=0;self.last_request=-1e9
        self.calls=0;self.rejected=0;self.reason='waiting for native terrain trajectory';self.last_ms=None
        self.start_attempt=0
    def close(self):
        self.pool.shutdown(wait=True);self.native.close()
    def cancel(self):
        self.active=None;self.next_plan=None;self.previous=None;self.epoch+=1;self.start_attempt=0
    def reference(self,stamp):
        choices=[a for a in (getattr(self,'previous',None),self.active,self.next_plan) if a is not None and a[1]<=stamp and a[2]==self.epoch]
        if not choices:return None
        active=max(choices,key=lambda a:a[1])
        plan,start,epoch=active
        if epoch!=self.epoch or not 0<=stamp-start<=plan.duration:return None
        return plan.sample(stamp-start)
    def update(self,m,now,climbing=False,descent=None):
        if self.next_plan is not None and self.next_plan[1]<=now:self.previous=self.active;self.active=self.next_plan;self.next_plan=None
        if self.pending is not None and self.pending[0].done():
            job,start,epoch=self.pending;self.pending=None
            try:
                plan=job.result()
                course=getattr(self,'pending_forward',None)
                if plan is not None and epoch==self.epoch and course is not None and now<=start:
                    motion=plan.samples(np.linspace(0,plan.duration,max(2,int(plan.duration/.025)+1)))
                    if np.min(motion[:,1,:2]@course)<-.05:
                        self.cancel();m.fail('Cloud descent trajectory reverses into delayed followers')
                        self.reason='Rejected reversing relay descent';return
                if plan is not None and epoch==self.epoch and now<=start:
                    if self.active is None:self.active=(plan,start,epoch)
                    else:self.next_plan=(plan,start,epoch)
                    self.last_ms=plan.milliseconds
                    self.start_attempt=0
                    self.reason='Native Swarm-Formation A*/MINCO terrain trajectory'
                else:
                    self.rejected+=1;self.reason='Native terrain trajectory late' if plan is not None else 'Native rejected: '+'; '.join(self.native.diagnostics[-2:])[-250:]
                    if plan is None and epoch==self.epoch and self.active is None and self.next_plan is None:
                        self.start_attempt=min(3,self.start_attempt+1)
                        if climbing:self.previous=None
            except Exception as exc:
                self.rejected+=1;self.reason='Native planning error: '+str(exc)
                self.native.close();self.native=NativePlanner(self.binary)
        if self.pending is not None or now-self.last_request<.4:return
        if descent is not None and self.active is not None and self.next_plan is None:
            plan,started,_=self.active
            # A receding horizon would keep moving the deceleration endpoint
            # away. Execute the certified slowdown before requesting another.
            if (m.speed<16. and started+plan.duration-now>.85
                    and np.linalg.norm(plan.sample(plan.duration)[1][:2])<=m.speed+.1):
                return
        start=now+.25;head=self.reference(start)
        if head is None and (self.active is not None or self.next_plan is not None):
            self.cancel();self.reason='Native horizon lacks a continuous future anchor; settling';return
        if head is None:
            # Keep the exact command boundary through the asynchronous first solve.
            coeff=np.zeros((1,3,6))
            coeff[0,:,3]=m.refa*.5;coeff[0,:,4]=m.refv;coeff[0,:,5]=m.refp
            if climbing:coeff[0,:,2]=-m.refa/3.
            bridge=NativeTrajectory([.5],coeff,0.)
            samples=bridge.samples(np.linspace(0.,.5,21))
            if climbing:
                direction=m.goal[:2]-m.refp[:2]
                direction/=max(np.linalg.norm(direction),1e-9)
                if (np.min(samples[:,1,:2]@direction)<.35 or np.min(samples[:,1,2])<-.05):
                    self.reason='Waiting for a forward climbing command boundary';return
            if not m.map.points_clear(samples[:,0],radius=.35).all():
                self.reason='Native entry bridge lacks terrain clearance';return
            self.previous=(bridge,now,self.epoch)
            head=bridge.sample(start-now)
        p,v,a=head;direction=m.goal-p;length=np.linalg.norm(direction[:2])
        if length<.5:return
        from cloud_config import CLIMB_SPEED_LIMIT, CRUISE_SPEED
        speed=min(float(getattr(m,'speed',6.)),CLIMB_SPEED_LIMIT if climbing else CRUISE_SPEED)
        goal,end=goal_boundary(p,v,m.goal,speed,turning=descent is not None)
        goal[2]=m.goal[2]
        if climbing:
            distance=np.linalg.norm(goal[:2]-p[:2])
            goal[2]=min(m.goal[2],p[2]+.24*distance)
            if goal[2]<m.goal[2]-.1:end[2]=min(3.,.24*np.linalg.norm(end[:2]))
            end*=min(1.,speed/max(np.linalg.norm(end),1e-9))
        if self.active is None and self.next_plan is None and (climbing or (np.linalg.norm(v)<.05 and np.linalg.norm(a)<.1)):
            # MINCO's soft feasibility cost can reject a stopped departure near
            # peers indefinitely with an identical endpoint request. Try bounded
            # gentler acceleration endpoints; every new polynomial still passes
            # the unchanged continuous speed/acceleration and map certificates.
            end*= ((.8,.7,.6,.5) if climbing else (1.,.9,.8,.6))[self.start_attempt]
        if descent is not None:
            origin=np.asarray(descent['origin'],float);anchor=np.asarray(descent['anchor'],float)
            distance=np.linalg.norm(anchor-origin)
            along=np.dot(goal[:2]-origin,(anchor-origin)/distance)
            profile=descent['from_z']-(descent['from_z']-descent['to_z'])*np.clip(along/descent['level_after'],0.,1.)
            goal[2]=profile
            if profile>descent['to_z']+.1:
                end[2]=-(descent['from_z']-descent['to_z'])/descent['level_after']*np.linalg.norm(end[:2])
            end*=min(1.,speed/max(np.linalg.norm(end),1e-9))
        source=m.map.planning_grid() if hasattr(m.map,'planning_grid') else m.map
        grid=copy.copy(source)
        for key in ('ground','origin','position','dynamic_peers'):setattr(grid,key,getattr(source,key).copy())
        certificate=m.map.certification_snapshot() if hasattr(m.map,'certification_snapshot') else grid
        peers=[(i,s['position'],s['velocity']) for i,s in m.five.peers.items() if -.1<=now-s.get('stamp',0)<=.6]
        locked_z=m.goal[2] if not climbing and descent is None and abs(p[2]-m.goal[2])<.1 and abs(v[2])<.1 else -1.
        limit=max(speed,min(CRUISE_SPEED,np.linalg.norm(v)+.05)) if descent is not None else speed
        future=self.pool.submit(self.native.plan,grid,p,v,a,goal,end,cruise_z=locked_z,drone_id=m.id,peers=peers,max_speed=limit,certification_grid=certificate)
        self.pending_forward=(direction[:2]/length if descent is not None and
                              np.dot(v[:2],direction[:2]/length)>.5 else None)
        self.pending=(future,start,self.epoch);self.last_request=now;self.calls+=1
