"""Exercise the two RViz command gates in Unity and retain full coordinator evidence."""
import json
import os
import time
from pathlib import Path
import numpy as np
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from point_mission_core import PointMission
from point_trial_readiness import inputs_ready,scene_ready

root=Path(('__UAV_PROJECT__'.replace("__UAV_PROJECT__", __import__("os").environ["UAV_PROJECT_ROOT"])));model=PointMission(root/'point.json')
rospy.init_node('point_full_flight_trial',anonymous=True,disable_signals=True)
states={};coordinator={};scene={};ground={};scene_received=0.
def state(msg,i):states[i]=json.loads(msg.data)
def status(msg):coordinator.clear();coordinator.update(json.loads(msg.data))
def world(msg):
    global scene_received
    scene.clear();scene.update(json.loads(msg.data));scene_received=rospy.Time.now().to_sec()
def ground_state(msg,i):ground[i]=msg.header.stamp.to_sec()
for i in range(10):
    rospy.Subscriber('/drone_%d/mission/status'%i,String,state,callback_args=i,queue_size=1)
    rospy.Subscriber('/drone_%d/local_ground'%i,PointCloud2,ground_state,callback_args=i,queue_size=1,buff_size=1000000)
rospy.Subscriber('/swarm10/status',String,status,queue_size=1)
rospy.Subscriber('/scene/status',String,world,queue_size=1)
pub=rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=1)
hold=rospy.Publisher('/swarm10/command',String,queue_size=1)
out=root/'verification/2026-10-31';out.mkdir(exist_ok=True,parents=True)
trial=out/('trial-'+time.strftime('%H%M%S')+'.jsonl')
log=trial.open('w',buffering=1)
def record(event,**kw):
    value=dict(stamp=rospy.Time.now().to_sec(),event=event,**kw);log.write(json.dumps(value)+'\n');print(json.dumps(value),flush=True)
def click(label):
    msg=PoseStamped();msg.header.frame_id='world';msg.header.stamp=rospy.Time.now()
    msg.pose.position.x=float(model.start[0]);msg.pose.position.y=float(model.start[1]);msg.pose.position.z=2.;msg.pose.orientation.w=1.
    pub.publish(msg);record('rviz_command',command=label,actor='automated acceptance via RViz topic')

started=time.monotonic();first=None;second=None;assembled=None;last_summary=0;ready_since=None
record('trial_start',point_hash=model.hashes['point'],log=str(trial))
try:
    while not rospy.is_shutdown():
        now=rospy.Time.now().to_sec();mono=time.monotonic();stage=coordinator.get('stage')
        if stage in ('FAILED','HOLD'):
            record('flight_failed',status=coordinator,states=states);break
        if stage=='LANDED':
            good=model.landed([states[i] for i in range(10)]) and scene.get('collision_events')==0 and scene.get('bounds_events')==0
            record('flight_finished',passed=good,status=coordinator,states=states,scene=scene);break
        if first is None:
            ready=stage=='IDLE' and pub.get_num_connections()>0 and scene_ready(scene,scene_received,now,model.hashes['world'])
            ready=ready and inputs_ready(states,ground,now,mode='CLOUD' if os.environ.get('YOPO_CLOUD_MISSION')=='1' else 'YOPO')
            if not ready:ready_since=None
            elif ready_since is None:ready_since=mono
            if ready_since is not None and mono-ready_since>=2 and now-scene_received<.25:first=mono;click('takeoff_assemble')
            elif mono-started>180:raise RuntimeError('Preflight telemetry not ready')
        elif stage=='IDLE' and mono-first>5:raise RuntimeError('First RViz command rejected: '+coordinator.get('reason',''))
        if stage=='ASSEMBLED' and second is None:
            if assembled is None:assembled=mono;record('assembly_hover_verified',states=states)
            if mono-assembled>=5:second=mono;click('depart_climb')
        if mono-last_summary>10:
            record('progress',stage=stage,groups=coordinator.get('group_phases'),reason=coordinator.get('reason'),
                   readiness=None if first is not None else dict(ready_seconds=0 if ready_since is None else mono-ready_since,scene_age=now-scene_received,
                     raw_ground_max_age=max([now-t for t in ground.values()],default=999),raw_ground_count=len(ground),
                     map_max_age=max([s['stamp']-(s.get('map',{}).get('ground_stamp') or 0) for s in states.values()],default=999),
                     frame_max_age=max([max(s.get('frame_ages',{}).values(),default=999) for s in states.values()],default=999)),
                   leaders={str(i):{k:states.get(i,{}).get(k) for k in ('position','speed','phase','reason')} for i in (2,7)})
            last_summary=mono
        if mono-started>7200:raise RuntimeError('Acceptance exceeded two-hour deadline')
        time.sleep(.2)
except Exception as exc:
    hold.publish(String('{"op":"hold"}'));record('trial_exception',error=repr(exc));raise
finally:log.close()
