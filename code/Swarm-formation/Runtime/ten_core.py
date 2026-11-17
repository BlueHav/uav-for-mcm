"""Two independent five-agent formations. No terrain/world database access."""
import itertools
import numpy as np
from formation_core import shape

def spawn():return np.c_[np.full(10,45.),-59+2*np.arange(10),np.zeros(10)]
def formation(name):
 p=shape(name);return p-p.mean(axis=0)

# Middle aircraft in the initial row remain the central aircraft throughout.
CENTER_IDS=(2,7)

def formation_tracking_error(actual,targets,relative_limit,common_limit=8.):
 """Bound formation distortion separately from coherent group tracking lag."""
 offset=np.asarray(targets)-np.asarray(actual);common=offset.mean(0)
 return max(float(np.linalg.norm(offset-common,axis=1).max()),float(np.linalg.norm(common))*relative_limit/common_limit)

def phase_duration(origin,dest,stage,ascent_speed=6.,descent_speed=6.):
 """Keep horizontal/low-altitude limits; use the controller's vertical reserve."""
 if not all(np.isfinite(v) and 0<v<=6. for v in (ascent_speed,descent_speed)):
  raise ValueError('Vertical reference speeds must be in (0,6] m/s')
 vertical=ascent_speed if stage=='CLIMB' else descent_speed if stage=='DESCEND' else .9
 delta=np.asarray(dest)-np.asarray(origin)
 horizontal=4. if stage in ('CLIMB','CRUISE','DESCEND') else .8
 return max(1.,np.linalg.norm(delta[:,:2],axis=1).max()/horizontal,np.abs(delta[:,2]).max()/vertical)

def terminal_progress_scale(stage,origin,dest,u,duration):
 """Slow the shared reference and feed-forward together before vertical endpoints."""
 if stage not in ('CLIMB','CRUISE','DESCEND'):return 1.
 distance=float(np.max(np.linalg.norm(np.asarray(dest)-origin,axis=1)))
 nominal=distance/max(duration,1.)
 # Shared S-curve follower accelerates beneath this conservative 0.8 m/s² envelope.
 ramp=min(max(.3,np.sqrt(1.6*distance*max(0.,u))),max(.3,np.sqrt(1.6*distance*max(0.,1.-u))))
 rate=min(1.,ramp/max(nominal,1e-9))
 if stage=='DESCEND':
  vertical=float(np.max(np.abs(np.asarray(dest)[:,2]-np.asarray(origin)[:,2])))
  terminal_cap=.9+.3*max(0.,vertical*(1.-u)-3.)
  rate=min(rate,terminal_cap/max(vertical/max(duration,1.),1e-9))
 return rate
def route_waypoints(start,landing=None,cruise_distance=30.,ramp_distance=100.):
 """Allocate forward climb, level cruise and forward descent on one route."""
 start=np.asarray(start,float)
 if start.shape!=(2,) or not np.isfinite(start).all():raise ValueError('Invalid route start')
 if not np.isfinite([cruise_distance,ramp_distance]).all() or cruise_distance<=0 or ramp_distance<=0:raise ValueError('Invalid route lengths')
 end=start+np.array([cruise_distance+ramp_distance+20.,0.]) if landing is None else np.asarray(landing,float)
 if end.shape!=(2,) or not np.isfinite(end).all():raise ValueError('Invalid landing center')
 distance=float(np.linalg.norm(end-start))
 if distance<50.:raise ValueError('Choose a landing center at least 50 m from assembly center for forward climb')
 direction=(end-start)/distance;ramp=min(ramp_distance,distance*.6) if landing is not None else ramp_distance
 descent=min(20.,distance*.2)
 points=(start+direction*ramp,end-direction*descent,end.copy())
 for point in points:landing_goal('world',*point)
 return points

def linear_reference(a,b,u):
 return np.asarray(a)+(np.asarray(b)-a)*float(np.clip(u,0,1))

def mission_reference(stage,a,b,u):
 """Gain clearance early in climb; then progressively increase forward travel."""
 u=float(np.clip(u,0,1));p=linear_reference(a,b,u)
 if stage=='CLIMB':p[:,:2]+=(np.asarray(b)[:,:2].mean(0)-np.asarray(a)[:,:2].mean(0))*(u*u-u)
 return p

def mission_tangent(stage,a,b,u):
 delta=np.asarray(b)-np.asarray(a)
 if stage=='CLIMB':
  delta=delta.copy();delta[:,:2]+=(np.asarray(b)[:,:2].mean(0)-np.asarray(a)[:,:2].mean(0))*(2*u-1)
 return delta

def landing_goal(frame,x,y):
 if frame not in ('','world','map'):raise ValueError('Goal frame must be world or map')
 if not np.isfinite([x,y]).all() or not -8400<x<1450 or not -6500<y<3350:
  raise ValueError('Landing center outside formation-safe bounds')
 return np.array([x,y],float)

def mission_shape(before,name,z,spacing,center=None,line_spacing=10.):
 """Assign outer slots only; preserve central IDs and test entire swept path."""
 templates={'square':[[-3.3,-3.3,0],[3.3,-3.3,0],[0,0,0],[-3.3,3.3,0],[3.3,3.3,0]],
            'line':[[-20,0,0],[-10,0,0],[0,0,0],[10,0,0],[20,0,0]],
            'star':[[0,-3.3,0],[-3.3,0,0],[0,0,0],[3.3,0,0],[0,3.3,0]]}
 if not np.isfinite(line_spacing) or not 10.<=line_spacing<=20.:raise ValueError('Line spacing must be 10..20 m')
 if name=='line':templates['line']=[[(i-2)*line_spacing,0,0] for i in range(5)]
 before=np.asarray(before,float);middle=before.mean(0).copy()
 if center is not None:middle[:2]=center
 middle[2]=z;outer=[0,1,3,4];out=[]
 for g in range(2):
  target=np.array(templates[name],float)+middle+[0,(-.5 if g==0 else .5)*spacing,0]
  choices=[]
  for perm in itertools.permutations(outer):
   ids=list(perm[:2])+[2]+list(perm[2:]);q=target[ids]
   if segment_clearance(before[g*5:g*5+5],q)>=1.9-1e-8:
    choices.append((float(np.sum((q-before[g*5:g*5+5])**2)),ids))
  if not choices:raise ValueError('No safe center-preserving '+name+' transition')
  out.append(target[min(choices)[1]])
 result=np.concatenate(out)
 if segment_clearance(before,result)<1.9-1e-8:raise ValueError('Cross-group swept paths overlap')
 return result
def center_distance(p):
 p=np.asarray(p,float);return float(np.linalg.norm(p[:5].mean(0)-p[5:].mean(0)))
def center_band(p,low=30.,high=80.):return bool(low<=center_distance(p)<=high)
def predicted_band(p,v,horizon=3.,low=35.,high=75.):
 p,v=np.asarray(p,float),np.asarray(v,float)
 delta=p[:5].mean(0)-p[5:].mean(0);vel=v[:5].mean(0)-v[5:].mean(0)
 t=float(np.clip(-delta@vel/max(vel@vel,1e-12),0,horizon))
 return bool(all(low<=np.linalg.norm(delta+vel*t0)<=high for t0 in (0,t,horizon)))
def interpolate(a,b,u):
 u=float(np.clip(u,0,1));s=u*u*(3-2*u)
 return np.asarray(a)*(1-s)+np.asarray(b)*s
def segment_clearance(a,b):
 a,b=np.asarray(a,float),np.asarray(b,float);i,j=np.triu_indices(len(a),1)
 delta=a[i]-a[j];v=(b-a)[i]-(b-a)[j]
 t=np.clip(-np.sum(delta*v,axis=1)/np.maximum(np.sum(v*v,axis=1),1e-12),0,1)
 return float(np.linalg.norm(delta+t[:,None]*v,axis=1).min())
def assign(before,after,clearance=2.):
 before,after=np.asarray(before,float),np.asarray(after,float)
 feasible=[]
 for perm in itertools.permutations(range(5)):
  points=after[list(perm)]
  if segment_clearance(before,points)>=clearance-1e-8:
   feasible.append((float(np.sum((points-before)**2)),perm))
 if not feasible:raise ValueError('No collision-separated continuous formation assignment')
 return after[list(min(feasible)[1])].copy()
def advance(u,dt,duration,error,blocked,max_error=1.5):
 return min(1.,u+min(.25,max(0.,dt))/max(duration,1.)) if error<max_error and not blocked else u
def crop(points,origin,radius=20.):
 p=np.asarray(points,float).reshape(-1,3);o=np.asarray(origin,float)
 return p[np.isfinite(p).all(1)&(np.linalg.norm(p-o,axis=1)<=radius)]

def crop_ground(points,origin,radius=20.):
 """Terrain under the aircraft is selected by XY radius, regardless of altitude."""
 p=np.asarray(points,float).reshape(-1,3);o=np.asarray(origin,float)
 return p[np.isfinite(p).all(1)&(np.linalg.norm(p[:,:2]-o[:2],axis=1)<=radius)]
def merge_clouds(clouds,voxel=.5,cap=60000):
 good=[np.asarray(p).reshape(-1,3) for p in clouds if len(p)]
 if not good:return np.empty((0,3),np.float32)
 p=np.concatenate(good);p=p[np.isfinite(p).all(1)]
 if not len(p):return np.empty((0,3),np.float32)
 _,ids=np.unique(np.floor(p/voxel).astype(np.int64),axis=0,return_index=True)
 p=p[np.sort(ids)]
 if len(p)>cap:p=p[np.linspace(0,len(p)-1,cap,dtype=int)]
 return p.astype(np.float32)
def depth_points(depth,position,rotation,focal,cx,cy,stride=8):
 yy,xx=np.mgrid[0:depth.shape[0]:stride,0:depth.shape[1]:stride];z=depth[yy,xx].reshape(-1)
 valid=np.isfinite(z)&(z>.05)&(z<79.9)
 z,xx,yy=z[valid],xx.reshape(-1)[valid],yy.reshape(-1)[valid]
 body=np.c_[z,-(xx-cx)*z/focal,-(yy-cy)*z/focal]
 return body@np.asarray(rotation).T+position

def failure_directory(root,run_id):
 import json
 from pathlib import Path
 parent=(Path(root)/'Reports').resolve()
 index=json.loads((parent/'run-index.json').read_text())
 target=(parent/index['directory']).resolve()
 if not run_id or index.get('run_id')!=run_id or parent not in target.parents:
  raise ValueError('Failure snapshot run mismatch or destination outside session reports')
 if not target.is_dir():raise ValueError('Failure report directory does not exist')
 return target

def register_failure_directory(root,run_id,directory):
 import json
 from pathlib import Path
 parent=(Path(root)/'Reports').resolve();target=Path(directory).resolve()
 if not run_id or parent not in target.parents or not target.is_dir():
  raise ValueError('Failure destination must be inside this session reports')
 parent.mkdir(parents=True,exist_ok=True)
 index=parent/'run-index.json';temporary=index.with_suffix('.tmp')
 temporary.write_text(json.dumps(dict(run_id=run_id,directory=str(target.relative_to(parent)))))
 temporary.replace(index)
