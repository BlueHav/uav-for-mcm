"""Bounded display copy of normalized front depth and policy input."""
import math
import time
import numpy as np

def normalize_depth_image(depth,width,height):
    import cv2
    image=cv2.resize(depth,(width,height),interpolation=cv2.INTER_NEAREST)
    image=np.nan_to_num(image,nan=0.,posinf=20.,neginf=0.)
    return np.clip(image,.04,20.)/20.

def sensor_depth_image(message):
    types={'32FC1':('f4',4,1.),'16UC1':('u2',2,.001)}
    if message.encoding not in types:return None
    kind,item_size,scale=types[message.encoding]
    if not (0<message.height<=270 and 0<message.width<=480 and
            message.step%item_size==0 and message.step>=message.width*item_size and
            len(message.data)==message.height*message.step):
        return None
    endian='>' if message.is_bigendian else '<'
    rows=np.frombuffer(message.data,dtype=endian+kind).reshape(message.height,message.step//item_size)
    return rows[:,:message.width].astype(np.float32)*scale


class PreviewSampler:
    def __init__(self,drone_id):
        if type(drone_id) is not int or not 0<=drone_id<10:raise ValueError('drone ID')
        self.drone_id=drone_id;self.last_stamp=-float('inf');self.sequence=0

    def take(self,image,source_stamp,now,monotonic):
        if source_stamp is None:return None  # Synthetic model warmup has no sensor receipt.
        if not all(math.isfinite(x) for x in (source_stamp,now,monotonic)):return None
        if not -.05<=now-source_stamp<=.6:return None
        # Capture timestamps deduplicate inference and display callbacks.
        # Preserve legitimate consecutive frames at the requested 15 Hz.
        if source_stamp<=self.last_stamp:return None
        array=np.asarray(image)
        if array.ndim!=2 or not 0<array.shape[0]<=270 or not 0<array.shape[1]<=480:return None
        if not np.isfinite(array).all() or np.any(array<0) or np.any(array>1):return None
        data=np.ascontiguousarray(array,dtype='<f4').tobytes()
        self.sequence+=1;self.last_stamp=source_stamp
        return dict(frame_id='drone_%d/yopo_input'%self.drone_id,source_stamp=source_stamp,
                    sequence=self.sequence,height=array.shape[0],width=array.shape[1],data=data)

class YopoInputPreview:
    def __init__(self,drone_id):
        import rospy
        from sensor_msgs.msg import Image
        self.rospy=rospy;self.Image=Image;self.sampler=PreviewSampler(drone_id)
        self.publisher=rospy.Publisher('/drone_%d/yopo/input_depth'%drone_id,Image,queue_size=1,tcp_nodelay=True)

    def publish_sensor_frame(self,message,width,height):
        if not self.publisher.get_num_connections():return
        try:
            depth=sensor_depth_image(message)
            if depth is not None:self.publish(normalize_depth_image(depth,width,height),message.header.stamp.to_sec())
        except (ValueError,TypeError) as error:
            self.rospy.logwarn_throttle(5.,'YOPO front-depth preview: %s',str(error))

    def publish(self,image,source_stamp):
        # Display failure cannot interrupt the flight controller or successful inference.
        try:
            if not self.publisher.get_num_connections():return
            p=self.sampler.take(image,source_stamp,self.rospy.Time.now().to_sec(),time.monotonic())
            if p is None:return
            msg=self.Image();msg.header.seq=p['sequence'];msg.header.stamp=self.rospy.Time.from_sec(p['source_stamp'])
            msg.header.frame_id=p['frame_id'];msg.height=p['height'];msg.width=p['width']
            msg.encoding='32FC1';msg.is_bigendian=0;msg.step=p['width']*4;msg.data=p['data']
            self.publisher.publish(msg)
        except Exception as error:self.rospy.logwarn_throttle(5.,'YOPO input display: %s',str(error))
