"""Refresh only the task-owned terrain/context publisher; flight nodes are untouched."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

root=Path(('__UAV_PROJECT__'.replace("__UAV_PROJECT__", __import__("os").environ["UAV_PROJECT_ROOT"])));out=root/'display';statefile=out/'processes.json'
state=json.loads(statefile.read_text());runtime=Path(__file__).resolve().parent
pid=state['display_pid'];proc=Path('/proc')/str(pid)
if proc.exists():
    cmd=(proc/'cmdline').read_bytes().split(b'\0')
    allowed=[root/'staging/Runtime/terrain_display_node.py',root/'verified/terrain-context/terrain_display_node.py']
    if not any(str(path).encode() in cmd for path in allowed):
        raise RuntimeError('Saved display PID belongs to a different process')
rviz=Path('/proc')/str(state['rviz_pid'])
env=dict(s.split('=',1) for s in (rviz/'environ').read_bytes().decode().split('\0') if '=' in s)
if env.get('ROS_MASTER_URI','').rstrip('/')!='http://127.0.0.1:11322':raise RuntimeError('Unexpected mission master')
if proc.exists():os.kill(pid,signal.SIGTERM)
with (out/'terrain-display.log').open('a') as log:
    p=subprocess.Popen(['/usr/bin/python3',str(runtime/'terrain_display_node.py')],env=env,stdout=log,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,start_new_session=True)
state['display_pid']=p.pid;statefile.write_text(json.dumps(state,indent=2))
time.sleep(4)
if p.poll() is not None:raise RuntimeError((out/'terrain-display.log').read_text()[-2000:])
os.environ.update(env)
import rospy
from std_msgs.msg import String
rospy.init_node('verify_terrain_context',anonymous=True,disable_signals=True)
report={}
for g in range(2):
    hint=json.loads(rospy.wait_for_message('/swarm10/group_%d/terrain_guidance'%g,String,timeout=15).data)
    report['group_'+str(g)]={k:v for k,v in hint.items() if k!='path'}
report['display']=json.loads(rospy.wait_for_message('/swarm10/display/status',String,timeout=15).data)
(out/'context-live-evidence.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report))
