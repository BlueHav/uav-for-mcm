"""Restart only RViz and its presentation relay; preserve all flight processes."""
import portable_paths
import json,os,signal,subprocess,time
import launch
state=json.loads(launch.STATE.read_text())
for item in state:
 if item['name'] in ('rviz','rviz_text_filter') and item.get('boot_id')==launch.BOOT and launch.ticks(item['pid'])==item['ticks']:
  os.killpg(item['pid'],signal.SIGTERM)
time.sleep(1)
for item in state:
 if item['name'] in ('rviz','rviz_text_filter') and item.get('boot_id')==launch.BOOT and launch.ticks(item['pid'])==item['ticks']:
  os.killpg(item['pid'],signal.SIGKILL)
state=[x for x in state if x['name'] not in ('rviz','rviz_text_filter')]
env=dict(os.environ,DISABLE_ROS1_EOL_WARNINGS='1',DISPLAY=os.environ.get('YOPO_DISPLAY',os.environ.get('DISPLAY',':20')),ROS_MASTER_URI='http://127.0.0.1:11322',ROS_IP='127.0.0.1',LD_LIBRARY_PATH='/usr/lib/x86_64-linux-gnu/blas:/usr/lib/x86_64-linux-gnu/lapack:'+os.environ.get('LD_LIBRARY_PATH',''))
base=('source /opt/ros/noetic/setup.bash; source ' + str(portable_paths.RVIZ) + '/devel/setup.bash; ')
commands={'rviz_text_filter':base+'exec python3 '+str(launch.RUN/'rviz_text_filter.py'),
 'rviz':base+'exec /bin/bash '+__import__('shlex').quote(str(launch.RUN/'rviz_render.sh'))+' rviz -d '+__import__('shlex').quote(str(launch.RUN/'ten.rviz'))}
for name,cmd in commands.items():
 with open(launch.LOG/(name+'.log'),'w') as log:
  p=subprocess.Popen(['/bin/bash','-c',cmd],env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 state.append(dict(name=name,pid=p.pid,ticks=launch.ticks(p.pid),boot_id=launch.BOOT))
launch.STATE.write_text(json.dumps(state,indent=2));time.sleep(5)
assert all(launch.ticks(x['pid'])==x['ticks'] for x in state[-2:]),'Display component failed'
print('RViz and display relay refreshed; flight processes preserved')
