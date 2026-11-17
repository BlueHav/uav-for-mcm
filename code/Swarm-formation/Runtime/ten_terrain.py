"""Fixed terminal route and coordinated terrain/landing stage orchestration."""
import json,time,uuid
import numpy as np
import rospy
from std_msgs.msg import String
from ten_core import mission_shape,landing_goal
from star_approach import select_candidate,candidate_ready,star_candidate

class TerrainStages:
 def begin_approach(self):
  self.approach_targets=self.end_points.copy();self.targets=None;self.set_stage('APPROACH')
  self.approach_progress_positions=self.positions().copy();self.approach_progress_times=np.full(10,time.monotonic())
  select_candidate(self,0)

 def approach_tick(self,now):
  if self.stage!='APPROACH':return
  actual=self.positions();mono=time.monotonic();errors=np.linalg.norm(actual-self.approach_targets,axis=1)
  arrived=np.array([errors[i]<.25 and np.linalg.norm(s['velocity'])<.2 and abs(s['agl']-2)<.2 for i,s in sorted(self.states.items())])
  ready=candidate_ready(self,now)
  if self.stage!='APPROACH':return
  if ready and np.all(arrived):self.begin_terrain();return
  moved=np.linalg.norm(actual-self.approach_progress_positions,axis=1)>.3
  self.approach_progress_times[moved|arrived]=mono;self.approach_progress_positions[moved]=actual[moved]
  if ready and np.any(mono-self.approach_progress_times>45) or mono-self.started>180:self.fail('No safe final descent approach');return
  for g,pub in enumerate(self.refs):pub.publish(String(json.dumps(dict(run_id=self.run_id,stage='APPROACH',stamp=now,targets=self.approach_targets[g*5:g*5+5].tolist(),velocity=np.zeros((5,3)).tolist(),speed=.8,safety_pause=False,approach_token=self.approach_token,validate_only=not ready))))
  self.reason='Local 3D approach to AGL 2 m star; orientation %d degrees%s'%(self.approach_angle,'' if ready else '; checking local occupancy')

 def init_terrain(self):
  self.terminal_enabled=bool(rospy.get_param('~continue_to_terminal',True))
  self.diagnostic_start_low=bool(rospy.get_param('~diagnostic_start_low',False))
  self.preset_descent=landing_goal('world',float(rospy.get_param('~descent_x',1495.)),float(rospy.get_param('~descent_y',-5.)))
  self.terminal_goal=landing_goal('world',float(rospy.get_param('~terminal_x',1745.)),float(rospy.get_param('~terminal_y',-50.)))
  self.low_speed=float(rospy.get_param('~low_speed',2.))
  if not np.isfinite(self.low_speed) or not .2<=self.low_speed<=3.:raise ValueError('Low speed must be 0.2..3 m/s')
  if np.linalg.norm(self.terminal_goal-self.preset_descent)<10:raise ValueError('Terrain route requires distinct descent and terminal points')
  self.low_targets=None;self.landing_token=None

 def begin_terrain(self):
  self.low_targets=star_candidate(self.positions(),self.terminal_goal,2.,0.)
  self.targets=None;self.set_stage('TERRAIN')
  self.low_started=time.monotonic();self.low_progress_positions=self.positions().copy()
  self.low_progress_times=np.full(10,self.low_started)
  self.low_initial_distance=float(np.linalg.norm(self.low_targets[:,:2]-self.positions()[:,:2],axis=1).max())
  if self.report is not None:
   self.report.setdefault('route',{})['terminal_center']=self.terminal_goal.tolist()
   self.report['low_speed']=self.low_speed;self.report['low_agl']=2.;self.save()

 def start_preland(self):
  self.targets=None;self.stop_stamp=rospy.Time.now().to_sec()
  self.landing_token=self.run_id+':land:'+uuid.uuid4().hex[:8]
  self.stop_refs();self.send('hold',_phase_token=self.landing_token);self.set_stage('PRELAND')

 def terrain_tick(self,now):
  if self.stage!='TERRAIN':return
  actual=self.positions();mono=time.monotonic()
  errors=np.linalg.norm(actual[:,:2]-self.low_targets[:,:2],axis=1)
  arrived=np.array([errors[i]<.5 and s.get('phase')=='END_HOLD' and np.linalg.norm(s['velocity'])<.15 and abs(s['agl']-2)<.25 for i,s in sorted(self.states.items())])
  if np.all(arrived):self.start_preland();return
  moved=np.linalg.norm(actual-self.low_progress_positions,axis=1)>.5
  self.low_progress_times[moved|arrived]=mono;self.low_progress_positions[moved]=actual[moved]
  if np.any(mono-self.low_progress_times>45.):self.fail('No safe terrain progress for 45 seconds');return
  if mono-self.low_started>max(180.,self.low_initial_distance/self.low_speed*8+120):self.fail('Terrain transit timeout');return
  # Deformable low flight must let a clear leader release a blocked follower's
  # corridor. Each member still checks obstacles, peers and its stopping sweep.
  for g,pub in enumerate(self.refs):
   pub.publish(String(json.dumps(dict(run_id=self.run_id,stage='TERRAIN',stamp=now,targets=self.low_targets[g*5:g*5+5].tolist(),velocity=np.zeros((5,3)).tolist(),speed=self.low_speed,agl=2.,height_mode='AGL',safety_pause=False))))
  self.reason='AGL 2 m terrain transit; independent local obstacle avoidance'

 def preland_tick(self,mono):
  if self.stage!='PRELAND':return
  ready=all(s['stamp']>=self.stop_stamp and s['phase']=='HOLD' and not s.get('formation',{}).get('active',True) and np.linalg.norm(s['velocity'])<.15 and s.get('landing_ready',False) and s.get('landing_check_stamp',0)>=self.stop_stamp and s.get('landing_ack_token')==self.landing_token for s in self.states.values())
  if ready:
   self.send('land',_phase_token=self.landing_token);self.set_stage('LANDING')
  elif mono-self.started>20:self.fail('Landing readiness not acknowledged by all members')

 def retry_landing(self,now):
  if self.stage!='LANDING':return
  for i,s in self.states.items():
   if s.get('phase')=='HOLD' and s.get('landing_ready',False) and s.get('landing_ack_token')==self.landing_token and 0<=now-s.get('landing_check_stamp',0)<=.6:
    self.tasks[i].publish(String(json.dumps(dict(op='land',_formation_internal=True,_phase_token=self.landing_token))))
