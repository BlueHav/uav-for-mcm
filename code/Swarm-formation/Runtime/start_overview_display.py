"""Start and verify display topics, then optionally switch only the RViz window."""
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import yaml
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--switch-rviz', action='store_true')
parser.add_argument('--rviz-pid', type=int)
args = parser.parse_args()

root = Path(('__UAV_PROJECT__'.replace("__UAV_PROJECT__", __import__("os").environ["UAV_PROJECT_ROOT"])))
runtime = Path(__file__).resolve().parent
out = root/'display'
out.mkdir(exist_ok=True)
candidates = [int(p) for p in subprocess.check_output(['pgrep', '-x', 'rviz']).split()]
if args.rviz_pid is not None:
    if args.rviz_pid not in candidates: raise RuntimeError('Requested PID is not a running RViz')
    old_pid = args.rviz_pid
elif len(candidates) == 1:
    old_pid = candidates[0]
else:
    raise RuntimeError('Multiple RViz sessions; pass --rviz-pid for the intended mission: '+str(candidates))
proc = Path('/proc')/str(old_pid)
birth = (proc/'stat').read_text().split(') ',1)[1].split()[19]
env = dict(x.split('=',1) for x in (proc/'environ').read_bytes().decode().split('\0') if '=' in x)
if env.get('ROS_MASTER_URI', '').rstrip('/') != 'http://127.0.0.1:11322':
    raise RuntimeError('Selected RViz is not connected to this mission ROS master')
config = yaml.safe_load((runtime/'ten.rviz').read_text())
displays = config['Visualization Manager']['Displays']
template = next(d for d in displays if d.get('Topic') == '/display/swarm10/ground_points_fused')
for name, topic, pixels in [('Terrain near groups 1km / 10m spacing', '/swarm10/display/terrain_near', 2), ('Terrain overview 20km / 100m spacing', '/swarm10/display/terrain_far', 2)]:
    d = copy.deepcopy(template)
    d.update(Name=name, Topic=topic, Enabled=True, Value=True)
    d['Size (Pixels)'] = pixels
    displays.append(d)
views = config['Visualization Manager']['Views']
follow = copy.deepcopy(views['Current']); follow['Name'] = 'Close fleet view'
views['Saved'].insert(0, follow)
views['Current'].update(Distance=26000, Pitch=1.2)
views['Current']['Focal Point'] = dict(X=0, Y=0, Z=0)
(out/'overview20km.rviz').write_text(yaml.safe_dump(config, sort_keys=False))
statefile = out/'processes.json'
state = json.loads(statefile.read_text()) if statefile.exists() else {}
if state.get('display_pid') and Path('/proc/%d'%state['display_pid']).exists():
    raise RuntimeError('Display process already exists; inspect saved state before replacing')
with (out/'terrain-display.log').open('w') as log:
    p = subprocess.Popen(['/usr/bin/python3', str(runtime/'terrain_display_node.py')], env=env, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
state = dict(display_pid=p.pid, previous_rviz_pid=old_pid, previous_rviz_birth=birth)
statefile.write_text(json.dumps(state, indent=2))
time.sleep(4)
if p.poll() is not None: raise RuntimeError((out/'terrain-display.log').read_text())
# Keep the ROS environment from the currently connected RViz, not a guessed master.
os.environ.update(env)
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
rospy.init_node('verify_overview_display', anonymous=True, disable_signals=True)
near = rospy.wait_for_message('/swarm10/display/terrain_near', PointCloud2, timeout=20)
far = rospy.wait_for_message('/swarm10/display/terrain_far', PointCloud2, timeout=20)
status = json.loads(rospy.wait_for_message('/swarm10/display/status', String, timeout=20).data)
assert near.width > 30000 and far.width > 30000 and status['missing_dem_points'] == 0, status
(out/'live-topic-evidence.json').write_text(json.dumps(status, indent=2))
if args.switch_rviz:
    with (out/'rviz-overview.log').open('w') as log:
        rviz = subprocess.Popen(['/opt/ros/noetic/lib/rviz/rviz', '-d', str(out/'overview20km.rviz'), '__name:=rviz_overview20km'], env=env, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
    time.sleep(7)
    if rviz.poll() is not None: raise RuntimeError((out/'rviz-overview.log').read_text())
    if proc.exists() and (proc/'stat').read_text().split(') ',1)[1].split()[19] == birth:
        os.kill(old_pid, signal.SIGTERM)
    state['rviz_pid'] = rviz.pid
    statefile.write_text(json.dumps(state, indent=2))
print(json.dumps(status))
