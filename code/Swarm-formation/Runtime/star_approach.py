"""Choose a star orientation from fresh per-member local occupancy checks."""
import itertools,time,uuid
import numpy as np

def star_candidate(before,center,z,angle):
 center=np.asarray(center,float);theta=np.deg2rad(angle)
 rotation=np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
 template=np.array([[0,-3.3],[-3.3,0],[0,0],[3.3,0],[0,3.3]])
 outer=[0,1,3,4];result=[]
 for group in range(2):
  slots=(template+[0,(-.5 if group==0 else .5)*9.8])@rotation.T+center
  old=np.asarray(before)[group*5:group*5+5,:2]
  options=[]
  for perm in itertools.permutations(outer):
   ids=list(perm[:2])+[2]+list(perm[2:]);options.append((np.sum((slots[ids]-old)**2),ids))
  result.append(np.c_[slots[min(options)[1]],np.full(5,z)])
 return np.concatenate(result)

def select_candidate(coordinator,index):
 angles=(0,90,180,270,45,135,225,315)
 if index>=len(angles):coordinator.fail('No fully observed clear star orientation at the preset descent point');return
 targets=star_candidate(coordinator.positions(),coordinator.preset_descent,2.,angles[index])
 coordinator.approach_targets=coordinator.local_heights(targets,2.)
 coordinator.approach_candidate_index=index;coordinator.approach_selected=False
 coordinator.approach_token=coordinator.run_id+':approach:'+uuid.uuid4().hex[:8]
 coordinator.approach_candidate_started=time.monotonic()
 coordinator.approach_angle=angles[index]

def candidate_ready(coordinator,now):
 was_selected=coordinator.approach_selected
 states=list(coordinator.states.values())
 acknowledged=all(s.get('approach_ack')==coordinator.approach_token and 0<=now-s.get('approach_check_stamp',0)<.6 for s in states)
 if acknowledged and all(s.get('approach_goal_clear',False) for s in states):
  coordinator.approach_selected=True
  if not was_selected:
   if hasattr(coordinator,'approach_progress_times'):coordinator.approach_progress_times[:]=time.monotonic()
   if coordinator.report is not None:coordinator.report['descent_star_angle']=coordinator.approach_angle;coordinator.save()
  return True
 # A clear goal seen from above can become occupied as the canopy is observed
 # from below. Pause and revalidate instead of handing an unsafe start to A*.
 coordinator.approach_selected=False
 if was_selected:coordinator.approach_candidate_started=time.monotonic()
 if acknowledged and time.monotonic()-coordinator.approach_candidate_started>6.:select_candidate(coordinator,coordinator.approach_candidate_index+1)
 return False
