#!/usr/bin/env python3
"""SSH-server-only real phase-six flight harness with owned-process cleanup."""
import argparse,hashlib,json,os,signal,socket,subprocess,sys,time
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import psutil

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'code/Swarm-formation/Runtime'))
from point_mission_core import PointMission
from mission_core import BODY_OFFSET

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--run',required=True);ap.add_argument('--timeout',type=float,default=1200.)
    ap.add_argument('--lag-seconds',type=float,default=0.,help='Test-only slow speed cap for group 0 leaf 3, starting 10 s after YOPO references')
    ap.add_argument('--lag-speed',type=float,default=2.,help='Slow injection speed, 2..6 m/s')
    args=ap.parse_args();run=Path(args.run).resolve();run.mkdir(parents=True,exist_ok=False)
    model=PointMission(ROOT/'config/point.json')
    runtime=ROOT/'code/Swarm-formation/Runtime'
    inputs=list(runtime.glob('*.py'))+[ROOT/'config/point.json',ROOT/'unity/Build/MountainCity_Data/Managed/Assembly-CSharp.dll']
    (run/'source-manifest.json').write_text(json.dumps({str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},indent=2))
    controllers=ET.parse(runtime/'point_controller.launch')
    for group in controllers.getroot().findall('group'):
        ident=int(group.get('ns').split('_')[-1]);g=ident//5
        xy=model.descent(g)[1][ident%5,:2];ground=float(model.ground(xy[None,:])[0])
        for node in group.findall('node'):
            for param in node.findall('param'):
                key=param.get('name')
                values={'initial_ground_z':ground,'simulator/init_state_x':xy[0],'simulator/init_state_y':xy[1],'simulator/init_state_z':ground+BODY_OFFSET}
                if key in values:param.set('value',str(values[key]))
    controllers.write(run/'controllers.launch',encoding='unicode')
    launch=ET.parse(runtime/'point_swarm_mission.launch')
    for include in launch.getroot().findall('include'):
        if 'point_controller' in include.get('file',''):include.set('file',str(run/'controllers.launch'))
    for node in launch.getroot().findall('node'):
        if node.get('name')=='point_coordinator':node.set('launch-prefix','/usr/bin/python3 -u '+str(ROOT/'tests/phase6_coordinator.py'))
    launch.write(run/'phase6.launch',encoding='unicode')
    (run/'Runtime').symlink_to(runtime,target_is_directory=True)
    env=os.environ.copy();env.update(SWARM_RUN_DIR=str(run),YOPO_POINT_MISSION='1',YOPO_CLOUD_MISSION='1',
        YOPO_RELAY='0',YOPO_DRONE_COUNT='10',YOPO_DEPTH_SOURCE='unity',YOPO_GROUP_CONTEXT='1',
        YOPO_INPROCESS_EXECUTOR='1',YOPO_ACTIVE_IDS=','.join(map(str,range(10))))
    if not 0<=args.lag_seconds<=60:raise ValueError('Test lag duration outside 0..60 seconds')
    if not 2.<=args.lag_speed<=6.:raise ValueError('Test lag speed outside 2..6 m/s')
    env['SWARM_TEST_LAG_SECONDS']=str(args.lag_seconds)
    env['SWARM_TEST_LAG_SPEED']=str(args.lag_speed)
    processes=[];result={'scope':'focused phase-six flight with ground takeoff, not full mission','passed':False}
    def interrupted(*_):raise RuntimeError('Flight harness interrupted')
    signal.signal(signal.SIGTERM,interrupted)
    try:
        def start(command,name):
            with (run/name).open('w') as log:
                p=subprocess.Popen(['bash','-c',command],cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            processes.append(p);return p
        unity=start('bash bash/unity.sh','unity.out')
        deadline=time.monotonic()+120
        while time.monotonic()<deadline:
            f=run/'Reports/player-unity.log'
            if f.exists() and 'TRACKER_FIELD_READY drones=10' in f.read_text(errors='replace'):break
            if unity.poll() is not None:raise RuntimeError('Unity exited; see unity.out')
            time.sleep(1)
        else:raise RuntimeError('Unity readiness timeout')
        start('source env.sh; export YOPO_GROUP_CLOUD=1; exec roslaunch --port 11322 '+str(run/'phase6.launch')+' mode:=ten point:='+str(model.path)+' rviz:=false','control.out')
        deadline=time.monotonic()+45
        while time.monotonic()<deadline:
            if any(p.poll() is not None for p in processes):raise RuntimeError('Flight component exited before ROS master readiness')
            try:
                with socket.create_connection(('127.0.0.1',11322),timeout=1):break
            except OSError:time.sleep(.3)
        else:raise RuntimeError('ROS master did not become ready on port 11322')
        # ROS imports happen only after setting the server environment.
        import rospy
        from std_msgs.msg import String
        rospy.init_node('phase6_test_driver',anonymous=True,disable_signals=True)
        status={}
        rospy.Subscriber('/swarm10/status',String,lambda m:status.update(json.loads(m.data)),queue_size=1)
        pub=rospy.Publisher('/swarm10/command',String,queue_size=1)
        deadline=time.monotonic()+args.timeout;sent=0.;previous=None
        while time.monotonic()<deadline:
            stage=status.get('stage')
            brief=(stage,tuple(status.get('group_phases',[])),status.get('planner_mode'))
            if brief!=previous:print(brief,flush=True);previous=brief
            if any(p.poll() is not None for p in processes):raise RuntimeError('Flight component exited')
            if stage=='IDLE' and pub.get_num_connections() and time.monotonic()-sent>2:
                pub.publish(String('{"op":"phase6_start"}'));sent=time.monotonic()
            if stage=='LANDED':result.update(passed=True,status=status.copy());break
            if stage in ('FAILED','HOLD'):raise RuntimeError(status.get('reason',stage))
            time.sleep(.2)
        else:raise RuntimeError('Phase-six flight timeout; last status '+json.dumps(status))
    except Exception as exc:result['error']=str(exc)
    finally:
        descendants=[]
        for p in processes:
            try:descendants.extend(psutil.Process(p.pid).children(recursive=True))
            except psutil.NoSuchProcess:pass
        for p in reversed(processes):
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        for p in reversed(processes):
            try:p.wait(timeout=12)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        for child in descendants:
            try:child.terminate()
            except psutil.NoSuchProcess:pass
        _,alive=psutil.wait_procs(descendants,timeout=10)
        for child in alive:
            try:child.kill()
            except psutil.NoSuchProcess:pass
        (run/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
    return 0 if result['passed'] else 1

if __name__=='__main__':sys.exit(main())
