"""Phase-owned input and reference contracts; no ROS or model dependency."""
import math
CLOUD='CLOUD'
YOPO_WARMUP='YOPO_WARMUP'
YOPO='YOPO'

def input_problem(state,raw_ground_stamp,now,mode):
    if mode not in (CLOUD,YOPO_WARMUP,YOPO):return 'invalid planning mode'
    if not -.1<=state.get('odom_age',999)<=.5:return 'odometry stale'
    if not 0<=state.get('plan_age',999)<=.8:return 'planner stale'
    if not -.1<=now-raw_ground_stamp<=.6 or not -.1<=state.get('stamp',0)-(state.get('map',{}).get('ground_stamp') or 0)<=.6:return 'local ground stale'
    if mode==YOPO:
        ages=state.get('frame_ages',{})
        if set(ages)!=set(('front','up','down')) or not all(math.isfinite(v) and -.1<=v<=.6 for v in ages.values()):return 'depth missing/stale'
        if not state.get('yopo_ready',False):return 'YOPO model not ready'
    return None

class ModeGate:
    def __init__(self,run_id):
        if not isinstance(run_id,str) or not run_id:raise ValueError('Run id required')
        self.run_id=run_id;self.mode=CLOUD;self.epoch=0
    def reference(self):return dict(run_id=self.run_id,planner_mode=self.mode,planner_epoch=self.epoch)
    def accepts(self,ref):return all(ref.get(k)==v for k,v in self.reference().items())
    def begin_handoff(self,assembled):
        if self.mode!=CLOUD or len(assembled)!=2 or not all(assembled):raise ValueError('Both groups must assemble before handoff')
        self.mode=YOPO_WARMUP;self.epoch+=1
    def release(self,ready):
        if self.mode!=YOPO_WARMUP or len(ready)!=10 or not all(ready):raise ValueError('All ten YOPO inputs must be ready')
        self.mode=YOPO;self.epoch+=1
