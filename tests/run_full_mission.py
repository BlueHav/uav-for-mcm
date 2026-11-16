"""Server-only supervisor for the unmodified production full-mission entry."""
import argparse,hashlib,json,os,signal,subprocess,time
from pathlib import Path
import psutil


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--project',required=True);ap.add_argument('--run',required=True)
    ap.add_argument('--timeout',type=float,default=4500.)
    args=ap.parse_args();root=Path(args.project).resolve();run=Path(args.run).resolve()
    run.mkdir(parents=True,exist_ok=False)
    runtime=root/'code/Swarm-formation/Runtime'
    inputs=list(runtime.glob('*.py'))+[root/'config/point.json',
        runtime/'point_controller.launch',runtime/'point_swarm_mission.launch',
        root/'unity/Build/MountainCity_Data/Managed/Assembly-CSharp.dll',
        root/'bash/run_mode.sh',root/'bash/control.sh',root/'bash/unity.sh',root/'tests/run_point_mission.py']
    (run/'source-manifest.json').write_text(json.dumps({str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},indent=2)+'\n')
    command=['bash','bash/run_mode.sh','ten',str(run)]
    (run/'run-contract.json').write_text(json.dumps(dict(scope='full production mission from configured stage-one spawn',
        command=command,phase_skipping=False,spawn_override=False,lag_injection=False,project=str(root)),indent=2)+'\n')
    env=os.environ.copy();env['ROS_LOG_DIR']=str(run/'roslog')
    result=dict(scope='full production mission from stage one',passed=False)
    process=None;entry_created=None;owned={};last={};telemetry=None;previous=None;control_seen=False
    def interrupted(*_):raise RuntimeError('Full mission supervisor interrupted')
    signal.signal(signal.SIGTERM,interrupted)
    def remember():
        if process is None:return []
        try:
            entry=psutil.Process(process.pid)
            if entry.create_time()!=entry_created:return []
            children=entry.children(recursive=True)
        except psutil.NoSuchProcess:return []
        for child in children:
            try:owned[(child.pid,child.create_time())]=child
            except psutil.NoSuchProcess:pass
        return children
    try:
        with (run/'supervised-entry.out').open('w') as log:
            process=subprocess.Popen(command,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        entry_created=psutil.Process(process.pid).create_time()
        result['entry_pid']=process.pid;deadline=time.monotonic()+args.timeout
        while process.poll() is None:
            children=remember()
            control_alive=False
            for child in children:
                try:
                    if any(a.endswith('/bin/roslaunch') for a in child.cmdline()):control_alive=True;control_seen=True
                except (psutil.NoSuchProcess,psutil.AccessDenied):pass
            if telemetry is None:
                candidates=list((run/'Reports/ten-runs').glob('*/telemetry.jsonl'))
                if candidates:telemetry=max(candidates,key=lambda p:p.stat().st_mtime)
            if telemetry is not None and telemetry.exists():
                with telemetry.open('rb') as stream:
                    stream.seek(max(0,telemetry.stat().st_size-2000000));lines=stream.read().splitlines()
                for line in reversed(lines[-3:]):
                    try:last=json.loads(line)['status'];break
                    except (ValueError,KeyError):pass
            else:
                try:last=json.loads((run/'driver.json').read_text())
                except (OSError,ValueError):pass
            brief=(last.get('stage'),tuple(last.get('group_phases',[])),last.get('planner_mode'),last.get('reason'))
            if brief!=previous:print(brief,flush=True);previous=brief
            if last.get('stage') in ('FAILED','HOLD'):raise RuntimeError(last.get('reason') or last['stage'])
            if control_seen and not control_alive and last.get('stage')!='LANDED':raise RuntimeError('Production control stack exited before landing')
            if time.monotonic()>deadline:raise RuntimeError('Full production mission timeout')
            time.sleep(1.)
        remember()
        try:last=json.loads((run/'driver.json').read_text())
        except (OSError,ValueError):pass
        result.update(entry_exit=process.returncode,status=last,passed=process.returncode==0 and last.get('stage')=='LANDED')
        if not result['passed']:result['error']=last.get('reason') or 'Production entry did not reach LANDED'
    except Exception as exc:result['error']=str(exc)
    finally:
        remember()
        if process is not None and process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=10)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        alive=[]
        for (pid,created),child in owned.items():
            try:
                if child.is_running() and child.create_time()==created:child.terminate();alive.append(child)
            except psutil.NoSuchProcess:pass
        _,alive=psutil.wait_procs(alive,timeout=12)
        for child in alive:
            try:
                if (child.pid,child.create_time()) in owned:child.kill()
            except (psutil.NoSuchProcess,StopIteration):pass
        (run/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(dict(passed=result['passed'],error=result.get('error'),stage=last.get('stage'),run=str(run))),flush=True)
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
