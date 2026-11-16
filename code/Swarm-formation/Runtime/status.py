import json,sys,socket,signal
from pathlib import Path
import launch
result={'scene':'MountainCity2000x1000','ros_master':'http://127.0.0.1:11322','components':{},'missions':{}}
if launch.STATE.exists():
 for p in json.loads(launch.STATE.read_text()):result['components'][p['name']]=p.get('boot_id')==launch.BOOT and launch.ticks(p['pid'])==p['ticks']
entry=launch.RUN/'entry-state.json'
if entry.exists():
 p=json.loads(entry.read_text());result['components']['ten_entry']=p.get('boot_id')==launch.BOOT and launch.ticks(p['pid'])==p['ticks']

def deadline(*_):raise TimeoutError('Status timeout')
signal.signal(signal.SIGALRM,deadline);signal.alarm(15)
try:
 with socket.create_connection(('127.0.0.1',11322),timeout=1):pass
 import rospy
 from std_msgs.msg import String
 rospy.init_node('mountain_status',anonymous=True,disable_signals=True)
 result['scene_status']=json.loads(rospy.wait_for_message('/scene/status',String,timeout=3).data)
 for i in range(int((launch.RUN/'drone-count').read_text())):
  result['missions'][str(i)]=json.loads(rospy.wait_for_message('/drone_%d/mission/status'%i,String,timeout=3).data)
except Exception as e:result['error']=str(e)
finally:signal.alarm(0)
result['ready']=bool(result['components']) and all(result['components'].values()) and 'error' not in result and all(not s.get('failure') and s.get('frame_ages') and max(s['frame_ages'].values())<.6 for s in result['missions'].values())
print(json.dumps(result,indent=2))
if '--ready' in sys.argv and not result['ready']:raise SystemExit(1)
