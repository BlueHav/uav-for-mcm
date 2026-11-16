"""Opt-in external command/planning-stall probe; never publishes commands."""
import json,os,signal,time
from pathlib import Path
import rospy
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import String

class ControlStallProbe:
    def __init__(self,scene):
        self.scene=Path(scene);self.last={};self.sent={};self.owners={};self.plan_last={};self.plan_sent={}
        self.boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        self.stream=(self.scene/'Reports/control-stalls.jsonl').open('w',buffering=1)
        self.subscribers=[rospy.Subscriber('/drone_%d/pos_cmd'%i,PositionCommand,self.command,callback_args=i,queue_size=1,tcp_nodelay=True) for i in range(10)]
        self.subscribers += [rospy.Subscriber('/drone_%d/mission/status'%i,String,self.status,callback_args=i,queue_size=1,tcp_nodelay=True) for i in range(10)]
        self.timer=rospy.Timer(rospy.Duration(.05),self.check)

    def command(self,msg,i):
        self.last[i]=(time.monotonic(),msg.header.stamp.to_sec(),rospy.Time.now().to_sec())

    def status(self,msg,i):
        try:
            trace=json.loads(msg.data).get('plan_trace')
            if not isinstance(trace,dict) or trace.get('enabled') is not True:
                self.plan_last.pop(i,None);return
            self.plan_last[i]=(time.monotonic(),trace)
        except (ValueError,TypeError,AttributeError):return

    def check(self,_):
        if not self.owners:
            try:
                entries=json.loads((self.scene/'Runtime/processes.json').read_text())
                self.owners={int(x['name'][7:]):x for x in entries if x['name'].startswith('planner')}
            except (OSError,ValueError,KeyError):return
        now=time.monotonic()
        for i,last in list(self.last.items()):
            gap=now-last[0]
            command_stall=.25<gap<5 and now-self.sent.get(i,0)>=2
            observation=self.plan_last.get(i);trace=None;token=None
            if observation is not None:
                received,candidate=observation
                try:
                    token=(candidate['stage'],float(candidate['started']))
                    age=float(candidate['age'])+now-received
                    if candidate['stage']!='idle' and 0<=now-received<2 and age>.2 and self.plan_sent.get(i)!=token:
                        trace=dict(candidate,observed_age=candidate['age'],age=age,status_receipt_age=now-received)
                except (KeyError,ValueError,TypeError):pass
            if not command_stall and trace is None:continue
            owner=self.owners.get(i)
            if not owner or owner.get('boot_id')!=self.boot:continue
            proc=Path('/proc')/str(owner['pid'])
            try:
                if proc.joinpath('stat').read_text().split(') ',1)[1].split()[19]!=owner['ticks']:continue
                cmd=proc.joinpath('cmdline').read_bytes().split(b'\0')
                if str(self.scene/'Runtime/mission_node.py').encode() not in cmd:continue
                # Signal only a process that advertises the opt-in handler's
                # environment and has already emitted a command after init.
                if b'YOPO_TRACE_CONTROL_STALLS=1' not in proc.joinpath('environ').read_bytes().split(b'\0'):continue
                caught=next(x.split(':',1)[1].strip() for x in proc.joinpath('status').read_text().splitlines() if x.startswith('SigCgt:'))
                if not int(caught,16)&(1<<(signal.SIGUSR1-1)):continue
                event=dict(kind='plan_stall' if trace is not None else 'command_stall',
                           stamp=rospy.Time.now().to_sec(),drone=i,pid=owner['pid'],receipt_gap=gap,last_header=last[1],last_receipt=last[2],last_delivery_age=last[2]-last[1])
                if trace is not None:event['plan_trace']=trace
                for name,path in (('cpu_stat','/sys/fs/cgroup/cpu.stat'),('cpu_pressure','/proc/pressure/cpu')):
                    event[name]=Path(path).read_text()
                self.stream.write(json.dumps(event)+'\n');os.kill(owner['pid'],signal.SIGUSR1)
                if command_stall:self.sent[i]=now
                if trace is not None:self.plan_sent[i]=token
            except (OSError,ValueError,StopIteration):continue
