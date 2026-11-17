"""Small sensor-only 3D bypass for the final descent, never a world-map planner."""
import heapq
import time
import numpy as np

def observed_3d_detour(grid,start,goal,margin=6.,max_expansions=6000,radius=.35):
 start,goal=np.asarray(start,float),np.asarray(goal,float);res=grid.resolution
 if not np.isfinite(np.r_[start,goal]).all() or np.linalg.norm(goal-start)>25:return None
 lo=np.floor((np.minimum(start,goal)-margin)/res).astype(int)
 hi=np.ceil((np.maximum(start,goal)+margin)/res).astype(int)
 shape=hi-lo+1
 if np.prod(shape)>100000:return None
 coords=np.indices(tuple(shape)).reshape(3,-1).T;points=(coords+lo+.5)*res
 allowed=np.zeros(len(points),bool)
 for k in range(0,len(points),2048):allowed[k:k+2048]=grid.points_clear(points[k:k+2048],radius=radius)
 allowed=allowed.reshape(tuple(shape))
 def endpoint(p):
  center=np.floor(p/res).astype(int)-lo;candidates=[]
  for offset in np.ndindex(3,3,3):
   cell=center+np.array(offset)-1
   if np.any(cell<0) or np.any(cell>=shape) or not allowed[tuple(cell)]:continue
   q=(cell+lo+.5)*res;candidates.append((np.linalg.norm(q-p),tuple(cell),q))
  for _,cell,q in sorted(candidates):
   if grid.corridor_clear(p,q,radius=radius):return cell
  return None
 a,b=endpoint(start),endpoint(goal)
 if a is None or b is None:return None
 queue=[(float(np.linalg.norm(np.asarray(a)-b)),0.,a)];costs={a:0.};parents={};closed=set()
 while queue and len(closed)<max_expansions:
  _,cost,cell=heapq.heappop(queue)
  if cell in closed:continue
  closed.add(cell)
  if cell==b:
   path=[cell]
   while cell in parents:cell=parents[cell];path.append(cell)
   path=np.array(path[::-1]);return np.vstack([start,(path+lo+.5)*res,goal])
  for axis in range(3):
   for direction in (-1,1):
    nxt=list(cell);nxt[axis]+=direction;nxt=tuple(nxt)
    if not 0<=nxt[axis]<shape[axis] or not allowed[nxt] or nxt in closed:continue
    score=cost+1
    if score>=costs.get(nxt,float('inf')):continue
    costs[nxt]=score;parents[nxt]=cell
    heapq.heappush(queue,(score+np.linalg.norm(np.asarray(nxt)-b),score,nxt))
 return None

def plan_approach(follower,p,v):
 m=follower.m;target=m.goal.copy();follower.blocked=False
 if (getattr(follower,'reference',None) or {}).get('validate_only',False):
  m.desired[:]=0.;m.reason='Checking star goal occupancy before approach'
  delta=m.goal-p
  if np.linalg.norm(delta[:2])>.1:m.heading=float(np.arctan2(delta[1],delta[0]))
  return
 if m.map.corridor_clear(p,target,radius=.5):follower.approach_path=None
 else:
  path=getattr(follower,'approach_path',None)
  if path is not None:
   if np.linalg.norm(path[-1]-p)<.35 and np.linalg.norm(path[-1]-m.goal)>.5:
    visited=getattr(follower,'approach_visited',[])
    if not any(np.linalg.norm(q-path[-1])<.5 for q in visited):visited.append(path[-1].copy())
    follower.approach_visited=visited
    m.heading=float(np.arctan2(m.goal[1]-p[1],m.goal[0]-p[0]))
    yaw_error=np.arctan2(np.sin(m.heading-m.yaw),np.cos(m.heading-m.yaw))
    if abs(yaw_error)>.25:m.desired[:]=0.;m.reason='Observe hidden descent goal from reached side point';return
   nearest=int(np.argmin(np.linalg.norm(path-p,axis=1)));path=path[nearest:]
   visible=[q for q in path if .2<np.linalg.norm(q-p)<1.5 and m.map.corridor_clear(p,q,radius=.5)]
   if visible:target=visible[-1]
   else:path=None
  if path is None:
   if time.monotonic()-getattr(follower,'approach_search',0)>1.:
    follower.approach_search=time.monotonic();search_goal=m.goal
    if not m.map.points_clear([search_goal],radius=.5)[0]:
     # A canopy can hide the target. First reach an observed nearby point at
     # the same height, then observe the final approach from beside it.
     candidates=np.array([m.goal+[dx,dy,0] for radius in (3.,5.) for dx,dy in ((radius,0),(-radius,0),(0,radius),(0,-radius),(radius,radius),(-radius,radius),(radius,-radius),(-radius,-radius))])
     candidates=candidates[m.map.points_clear(candidates,radius=.5)]
     candidates=np.array([q for q in candidates if np.linalg.norm(q-p)>.75 and not any(np.linalg.norm(q-old)<.5 for old in getattr(follower,'approach_visited',[]))]).reshape(-1,3)
     if len(candidates):
      order=np.argsort(np.linalg.norm(candidates-p,axis=1));search_goal=candidates[order[getattr(follower,'approach_candidate',0)%len(candidates)]]
    follower.approach_path=observed_3d_detour(m.map,p,search_goal,radius=.5)
    if follower.approach_path is None:follower.approach_candidate=getattr(follower,'approach_candidate',0)+1
   m.desired[:]=0.;follower.blocked=True;m.reason='Observed 3D descent bypass requested';return
  follower.approach_path=path
 delta=target-p;desired=1.2*delta-.7*v
 desired*=min(1.,.8/max(np.linalg.norm(desired),1e-9))
 desired[:2]*=min(1.,.4/max(np.linalg.norm(desired[:2]),1e-9))
 m.desired=desired;m.chosen=np.array([p,target]);m.reason='Observed local 3D descent approach'
 if np.linalg.norm(desired[:2])>.05:m.heading=float(np.arctan2(desired[1],desired[0]))
