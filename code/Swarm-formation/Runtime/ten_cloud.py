"""Display-only, timestamped bounded fusion. Never feeds planners."""
import json,threading
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from cloud_serialization import xyz32
from std_msgs.msg import Header,String
from ten_core import crop,crop_ground,merge_clouds,depth_points

def publish_raw(m,header,frames=None):
 clouds=[];now=rospy.Time.now().to_sec()
 for name,(stamp,d,pose) in list((m.frames if frames is None else frames).items()):
  if not -.1<=now-stamp<=1.2:continue
  p=depth_points(d,pose[0],pose[1],240. if name=='front' else 77.94228634059948,240.,135.)
  clouds.append(crop(p,pose[0]))
 m.raw_pub.publish(xyz32(header,merge_clouds(clouds)))

def xyz(msg):
 fields={f.name:f for f in msg.fields}
 if any(k not in fields or fields[k].datatype!=7 or fields[k].count!=1 for k in ('x','y','z')):raise ValueError('Expected xyz float32')
 dtype=np.dtype(dict(names=['x','y','z'],formats=[('>f4' if msg.is_bigendian else '<f4')]*3,offsets=[fields[k].offset for k in ('x','y','z')],itemsize=msg.point_step))
 rows=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.row_step,msg.point_step))
 return np.stack([rows[k].reshape(-1) for k in ('x','y','z')],axis=1)

class Fusion:
 def __init__(self):
  rospy.init_node('ten_cloud_fusion');self.lock=threading.RLock();self.clouds={};self.states={};self.rejected=0;self.ground_display=None
  topics={'map':'/display/swarm10/map_fused','raw':'/display/swarm10/depth_points_fused','ground':'/display/swarm10/ground_points_fused'}
  self.pubs={k:rospy.Publisher(t,PointCloud2,queue_size=1) for k,t in topics.items()}
  self.status=rospy.Publisher('/display/swarm10/cloud_status',String,queue_size=1)
  rospy.Subscriber('/unity/formation_ground',PointCloud2,self.formation_ground,queue_size=1,buff_size=4000000)
  for i in range(10):
   rospy.Subscriber('/drone_%d/mission/status'%i,String,self.state,callback_args=i,queue_size=1)
   for kind,suffix in [('map','local_map'),('raw','unity_points')]:
    rospy.Subscriber('/display/drone_%d/%s'%(i,suffix),PointCloud2,self.cloud,callback_args=(kind,i),queue_size=1,buff_size=2000000)
  rospy.Timer(rospy.Duration(1.),self.tick)
 def state(self,msg,i):
  try:
   with self.lock:self.states[i]=json.loads(msg.data)
  except ValueError:pass
 def cloud(self,msg,key):
  try:
   if msg.header.frame_id!='world':raise ValueError('Input not in world frame')
   p=xyz(msg)
   with self.lock:self.clouds[key]=(msg.header.stamp.to_sec(),p)
  except (ValueError,TypeError):self.rejected+=1
 def formation_ground(self,msg):
  try:
   if msg.header.frame_id!='world':raise ValueError('Invalid display ground')
   p=xyz(msg)
   if not np.isfinite(p).all():raise ValueError('Non-finite display ground')
   with self.lock:self.ground_display=msg
  except (ValueError,TypeError):self.rejected+=1
 def tick(self,_):
  now=rospy.Time.now().to_sec();out={};sources={}
  with self.lock:
   for kind,pub in self.pubs.items():
    if kind=='ground':
     msg=self.ground_display
     if msg is not None and -.1<=now-msg.header.stamp.to_sec()<=2.5:
      pub.publish(msg);out[kind]=msg.width*msg.height;sources[kind]='formation_terrain_display'
     else:
      self.ground_display=None;out[kind]=0;sources[kind]=[]
      pub.publish(xyz32(Header(stamp=rospy.Time.now(),frame_id='world'),[]))
     continue
    clouds=[];ids=[]
    for i in range(10):
     item=self.clouds.get((kind,i));s=self.states.get(i)
     if item is None or s is None:continue
     stamp,p=item
     if not -.1<=now-stamp<=2.5 or not -.1<=now-s.get('stamp',0)<=1.2:
      self.clouds.pop((kind,i),None);continue
     clouds.append((crop_ground if kind=='ground' else crop)(p,s['position']));ids.append(i)
    merged=merge_clouds(clouds,voxel=.5,cap=60000);out[kind]=len(merged);sources[kind]=ids
    pub.publish(xyz32(Header(stamp=rospy.Time.now(),frame_id='world'),merged))
  self.status.publish(String(json.dumps(dict(stamp=now,points=out,sources=sources,radius_m=20,ground_extent='formation_aabb_plus_50m_xy',ground_padding_m=50,ground_spacing_m=.5,ground_capacity=None,voxel_m=.5,capacity_per_layer=60000,ttl_s=2.5,rejected=self.rejected))))
if __name__=='__main__':Fusion();rospy.spin()
