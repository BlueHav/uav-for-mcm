#!/usr/bin/env python3
"""Independent process group launcher. Never terminates a process without matching birth ticks."""
import portable_paths
from depth_source import depth_source, sensor_ports, validate_player
from pathlib import Path
import os,sys,json,time,subprocess,signal,socket,argparse
ROOT=Path((str(portable_paths.SCENE)));RUN=ROOT/'Runtime';LOG=ROOT/'Reports';STATE=RUN/'processes.json'
BOOT=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
def ticks(pid):
 try:return Path('/proc/%d/stat'%pid).read_text().split(') ',1)[1].split()[19]
 except FileNotFoundError:return None
def stop():
 if not STATE.exists():return
 state=json.loads(STATE.read_text())
 for item in reversed(state):
  if item.get('boot_id')==BOOT and ticks(item['pid'])==item['ticks']:
   os.killpg(item['pid'],signal.SIGTERM)
 time.sleep(1)
 for item in reversed(state):
  if item.get('boot_id')==BOOT and ticks(item['pid'])==item['ticks']:
   try:os.killpg(item['pid'],signal.SIGKILL)
   except ProcessLookupError:pass
 STATE.unlink()
def waitport(port,seconds=30):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  with socket.socket() as s:
   if s.connect_ex(('127.0.0.1',port))==0:return
  time.sleep(.2)
 raise RuntimeError('Port did not become ready: '+str(port))
def start(count,audit):
 source_kind=depth_source();ports=sensor_ports(count,source_kind);validate_player(ROOT,source_kind)
 graphics=os.environ.get('YOPO_GRAPHICS_API','vulkan')
 if graphics not in ('vulkan','glcore'):raise ValueError('YOPO_GRAPHICS_API must be vulkan or glcore')
 RUN.mkdir(exist_ok=True);LOG.mkdir(exist_ok=True)
 native=RUN/'ray_carve.so';source=RUN/'ray_carve.cpp'
 if not native.exists() or source.stat().st_mtime>native.stat().st_mtime:
  temporary=native.with_name(native.name+'.tmp.'+str(os.getpid()))
  subprocess.run(['g++','-O3','-std=c++11','-shared','-fPIC',str(source),'-o',str(temporary)],check=True)
  temporary.replace(native)
 if source_kind=='pointcloud':
  raster_source=RUN/'cloud_depth_raster.cpp';raster_library=RUN/'cloud_depth_raster_native.so'
  if not raster_library.exists() or raster_source.stat().st_mtime>raster_library.stat().st_mtime:
   temporary=raster_library.with_name(raster_library.name+'.tmp.'+str(os.getpid()))
   subprocess.run(['g++','-O3','-std=c++11','-shared','-fPIC',str(raster_source),'-o',str(temporary)],check=True)
   temporary.replace(raster_library)
 if os.environ.get('YOPO_AUTO_DESKTOP','1')=='1':subprocess.run(['bash',str(portable_paths.DESKTOP_SCRIPT)],check=True)
 if STATE.exists():
  alive=[p for p in json.loads(STATE.read_text()) if p.get('boot_id')==BOOT and ticks(p['pid'])==p['ticks']]
  if alive:raise RuntimeError('Scene already running; use stop before changing count or audit mode')
 (RUN/'drone-count').write_text(str(count))
 for port in ports:
  with socket.socket() as s:
   if s.connect_ex(('127.0.0.1',port))==0:raise RuntimeError('Independent port already occupied: '+str(port))
 env=os.environ.copy();env.update(YOPO_DRONE_COUNT=str(count),MC_CAPTURE='1' if audit else '0',MC_REPORTS=str(LOG))
 from runtime_cpu_layout import detect_layout,child_command
 isolation=os.environ.get('YOPO_CPU_ISOLATION','auto')
 if isolation not in ('auto','0','1'):raise ValueError('YOPO_CPU_ISOLATION must be auto, 0 or 1')
 layout=detect_layout() if count==10 and env.get('YOPO_CLOUD_MISSION')=='1' and isolation!='0' else None
 (LOG/'cpu-layout.json').write_text(json.dumps(dict(mode=isolation,layout=layout),indent=2))
 state=[]
 def launch(name,cmd):
  log=open(LOG/(name+'.log'),'w');p=subprocess.Popen(child_command(name,cmd,layout),cwd=ROOT,env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
  state.append(dict(name=name,pid=p.pid,ticks=ticks(p.pid),boot_id=BOOT));STATE.write_text(json.dumps(state,indent=2))
 base='source "$YOPO_ROOT/env.sh"; '
 overlay=base
 try:
  launch('master',base+'exec roscore -p 11322');waitport(11322)
  launch('endpoint',base+'exec /usr/bin/python3 '+str(portable_paths.TCP/'src/ROS-TCP-Endpoint/src/ros_tcp_endpoint/default_server_endpoint.py')+' _tcp_ip:=127.0.0.1 _tcp_port:=10012');waitport(10012)
  launch('central_sensors',base+'exec /usr/bin/python3 -u '+str(RUN/'central_sensor_service.py'));waitport(10013)
  point=os.environ.get('YOPO_POINT_MISSION')=='1'
  controller_launch=str(RUN/('point_controller.launch' if point else 'ten_controller.launch')) if count==10 else str(RUN/'five_controller.launch') if count==5 else 'so3_quadrotor_simulator mission_ros1.launch'
  launch('controller',overlay+'exec roslaunch '+controller_launch+' count:='+str(count))
  (LOG/'player-unity.log').write_text('')
  launch('player','exec '+str(ROOT/'Build/MountainCity.x86_64')+' -force-'+graphics+' -job-worker-count 4 -screen-width 1440 -screen-height 900 -screen-fullscreen 0 -logFile '+str(LOG/'player-unity.log'))
  deadline=time.monotonic()+150
  while time.monotonic()<deadline:
   log=LOG/'player-unity.log'
   if log.exists() and 'TRACKER_FIELD_READY' in log.read_text(errors='replace'):break
   if ticks(state[-1]['pid'])!=state[-1]['ticks']:raise RuntimeError('Player exited; see player-unity.log')
   time.sleep(.5)
  else:raise RuntimeError('Player readiness timeout')
  if count==10 and os.environ.get('YOPO_CLOUD_MISSION')=='1':
   for i in range(count):launch('executor'+str(i),overlay+'exec /usr/bin/python3 -u '+str(RUN/'trajectory_executor_node.py')+' __ns:=/drone_'+str(i)+' _drone_id:='+str(i))
  for i in range(count):launch('planner'+str(i),overlay+'cd '+str(portable_paths.PLANNER)+'; exec '+str(portable_paths.PYTHON)+' -u '+str(RUN/'mission_node.py')+' __ns:=/drone_'+str(i)+' _drone_id:='+str(i)+' _count:='+str(count))
  if count==10 and os.environ.get('YOPO_CLOUD_MISSION')=='1':launch('cloud_context',overlay+'exec /usr/bin/python3 -u '+str(RUN/'cloud_context_node.py'))
  if count==5:launch('formation',overlay+'exec /usr/bin/python3 -u '+str(RUN/'formation_node.py'))
  if count==10:
   launch('ten_formation',overlay+'exec /usr/bin/python3 -u '+str(RUN/('point_mission_node.py' if point else 'ten_node.py')))
   launch('ten_cloud',overlay+'exec /usr/bin/python3 -u '+str(RUN/'ten_cloud.py'))
   launch('rviz_text_filter',overlay+'exec /usr/bin/python3 -u '+str(RUN/'rviz_text_filter.py'))
   if point:
    launch('terrain_context',overlay+'exec /usr/bin/python3 -u '+str(RUN/'terrain_display_node.py'))
    launch('sensor_health',overlay+'exec /usr/bin/python3 -u '+str(RUN/'record_sensor_health.py'))
  rviz_config=str(RUN/'ten.rviz') if count==10 else str(RUN/'five.rviz') if count==5 else (str(portable_paths.OPS) + '/config/mission.rviz')
  launch('rviz',overlay+'exec /bin/bash '+__import__('shlex').quote(str(RUN/'rviz_render.sh'))+' rviz -d '+__import__('shlex').quote(rviz_config))
  time.sleep(4)
  dead=[p['name'] for p in state if ticks(p['pid'])!=p['ticks']]
  if dead:raise RuntimeError('Components exited: '+','.join(dead))
  print('Mountain City ready; ROS 11322, TCP 10012, drones='+str(count),flush=True)
 except Exception:
  stop();raise
if __name__=='__main__':
 parser=argparse.ArgumentParser();parser.add_argument('action',choices=['start','stop']);parser.add_argument('--count',type=int,choices=[1,2,5,10],default=1);parser.add_argument('--audit',action='store_true');a=parser.parse_args()
 stop() if a.action=='stop' else start(a.count,a.audit)

