"""Prepare image planning while the terrain controller continues hovering."""
import threading
import numpy as np
from local_map import RollingLocalMap

# Match the ground and image freshness required by YOPO flight admission.
OBSERVATION_MAX_AGE=.6

class YopoHandoff:
    def __init__(self,mission,loader=None,start_worker=None):
        self.m=mission;self.run_id=None;self.enabled_stamp=None;self.loaded=False;self.error=None
        self.map=None;self.integrated={};self.loader=loader or self.load
        self.start_worker=start_worker or (lambda f:threading.Thread(target=f,daemon=True).start())
    def load(self):
        from yopo_warmup_gate import model_warmup_slot
        m=self.m
        with model_warmup_slot():
            m.load_yopo_policy()
            for _ in range(3):m.infer(np.full((270,480),80.,np.float32),m.p,m.v,m.rot,m.p+np.array([5.,0,0]))
    def work(self):
        try:self.loader();self.loaded=True
        except Exception as exc:self.error=repr(exc);self.m.yopo_ready=False
    def receive(self,data,now):
        m=self.m;run=data.get('run_id');mode=data.get('planner_mode');epoch=data.get('planner_epoch')
        if not isinstance(run,str) or not run:return False
        if self.run_id is None:
            if mode!='CLOUD' or epoch!=0:return False
            self.run_id=run
        if run!=self.run_id:return False
        if epoch==m.planner_epoch and mode==m.planning_mode:return True
        if mode=='YOPO_WARMUP' and epoch==1 and m.planning_mode=='CLOUD':
            if m.p is None or abs(m.p[2]-m.ground-5.)>.25 or np.linalg.norm(m.v)>.2:return False
            self.enabled_stamp=now;self.map=RollingLocalMap(size=(60.,60.,60.),depth_stride=12)
            m.planning_mode=mode;m.planner_epoch=epoch;m.subscribe_yopo_images();self.start_worker(self.work);return True
        if mode=='YOPO' and epoch==2 and m.planning_mode=='YOPO_WARMUP' and self.ready(now):
            m.map=self.map;m.planning_mode=mode;m.planner_epoch=epoch;return True
        return False
    def ready(self,now):
        if not self.loaded or self.error or not self.m.yopo_ready or self.map is None or self.map.ground_stamp is None:return False
        if self.map.origin is None or not 0<=now-self.map.ground_stamp<=OBSERVATION_MAX_AGE:return False
        return all(k in self.m.frames and self.enabled_stamp<self.integrated.get(k,0)<=now and now-self.integrated[k]<=OBSERVATION_MAX_AGE
                   and self.enabled_stamp<self.m.frames[k][0]<=now and now-self.m.frames[k][0]<=OBSERVATION_MAX_AGE for k in ('front','up','down'))
