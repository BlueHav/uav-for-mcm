"""Adapt an isolated runtime copy without changing the accepted source baseline."""

def status_runtime_source(source):
    anchor="        if self.five:\n            value['formation']=self.five.status()"
    if source.count(anchor)!=1:raise ValueError('Unknown runtime status layout; refusing adaptation')
    source=source.replace(anchor,"        value['odom_age']=stamp-odom_stamp\n        value['plan_age']=time.monotonic()-self.last_plan\n"+anchor)

    climb="            if abs(here_target-self.p[2])>.4:\n"
    if source.count(climb)!=1:raise ValueError('Unknown goto phase layout')
    return source.replace(climb,"            if ((raw.get('_forward_climb') is True or raw.get('_forward_descent') is True) and op=='goto' and self.mode=='ABS') or (raw.get('_terminal_profile') is True and op=='terrain_follow' and self.mode=='AGL'):\n                self.set_phase(self.pending_next,'Explicit observed continuous altitude profile')\n            elif abs(here_target-self.p[2])>.4:\n")


def single_executor_source(source):
    anchor="  if self.profile and self.phase=='END_HOLD':return"
    if source.count(anchor)!=1:raise ValueError('Unknown terminal hold layout')
    source=source.replace(anchor,"""  if self.profile and self.phase=='END_HOLD':
   if abs(p[2]-self.ground-2.)>.2:
    self.mode='AGL';self.height=2.;self.set_phase('TERRAIN_FOLLOW','Correct terminal AGL before completion')
   else:return
  if self.profile and self.phase in ('TERRAIN_FOLLOW','END_APPROACH') and np.linalg.norm(p[:2]-self.profile_end)<.5 and abs(p[2]-self.ground-2.)>.2:return""")

    task="   super().task(msg)"
    horizontal=" def horizontal_plan(self,p,v):\n"
    if source.count(task)!=1 or source.count(horizontal)!=1:raise ValueError('Unknown single executor task/planner layout')
    source=source.replace(task,task+"\n   self.forward_climb=bool(raw.get('_forward_climb') is True and self.phase=='GOTO' and not self.failure)")
    source=source.replace(horizontal,horizontal+"""  if getattr(self,'forward_climb',False):
   from active_forward_climb import observed_climb
   from mission_core import VERTICAL_SPEED
   delta=self.goal-p;self.heading=float(np.arctan2(delta[1],delta[0]))
   self.desired,self.chosen,self.reason=observed_climb(p,self.goal,self.yaw,self.speed,self.map,VERTICAL_SPEED)
   self.last_candidates=[self.chosen]
   if np.linalg.norm(self.desired)>0:self.last_safe=time.monotonic()
   return
""")
    return source
