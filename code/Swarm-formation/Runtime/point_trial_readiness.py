"""Conservative acceptance-driver gate; the coordinator remains authoritative."""
def scene_ready(scene,received,now,world_hash):
 return scene.get('drone_count')==10 and scene.get('map_sha256')==world_hash and -.1<=now-received<1.1

def inputs_ready(states,ground,now,mode='YOPO'):
 if set(states)!=set(range(10)) or set(ground)!=set(range(10)):return False
 for i,s in states.items():
  if s.get('armed',True) or not s.get('contact') or s.get('failure') or s.get('controller_fault'):return False
  if not -.1<=now-s.get('stamp',0)<.3 or not -.1<=s.get('odom_age',999)<.3 or not 0<=s.get('plan_age',999)<.4:return False
  if not -.1<=now-ground[i]<.3 or not -.1<=s['stamp']-(s.get('map',{}).get('ground_stamp') or 0)<.3:return False
  if mode=='CLOUD':
   if s.get('planner_mode')!='CLOUD' or any(s.get(k)!=0 for k in ('image_callbacks','network_inferences')):return False
   context=s.get('map',{})
   if not context.get('context_hash') or not -.1<=s['stamp']-(context.get('context_stamp') or 0)<.3:return False
  else:
   ages=s.get('frame_ages',{})
   if set(ages)!=set(('front','up','down')) or not all(-.1<=age<.3 for age in ages.values()):return False
 return True
