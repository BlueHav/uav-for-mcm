"""Persistent isolated five-body Swarm-Formation optimization process."""
import json
import os
import select
import subprocess
import time
import numpy as np
from swarm_native_bridge import NativeTrajectory

class TerminalNative:
    def __init__(self,binary=('__UAV_PROJECT__/code/Swarm-formation/native/terminal_continuous_native'.replace("__UAV_PROJECT__", __import__("os").environ["UAV_PROJECT_ROOT"]))):
        self.legacy=os.path.basename(binary)=='terminal_swarm_native'
        self.process=subprocess.Popen([binary],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL)
        self.buffer=b'';self.serial=0
    def __enter__(self):return self
    def __exit__(self,*args):self.close()
    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait()
        self.process.stdin.close();self.process.stdout.close()
    def plan(self,p,v,a,goals,duration,timeout=2.,end_velocity=None,obstacles=(),reference=(),peers=(),terrain=(),check_gradient=False,flexible=False,reference_members=()):
        tail_v=np.zeros((5,3)) if end_velocity is None else end_velocity
        arrays=[np.asarray(x,float) for x in (p,v,a,goals,tail_v)]
        if any(x.shape!=(5,3) or not np.isfinite(x).all() for x in arrays):raise ValueError('Invalid five-body inputs')
        if self.legacy:
            if reference_members or flexible or end_velocity is not None or len(obstacles) or len(reference) or len(peers) or len(terrain):raise ValueError('Legacy optimizer has no continuing/environment protocol')
            arrays=arrays[:4]
        self.serial+=1;values=[self.serial,duration]
        if not self.legacy:
            obs=np.asarray(obstacles,float).reshape(-1,4)
            refs=np.asarray(reference,float).reshape(-1,3)
            peer=np.asarray(peers,float)
            if not len(peer):peer=np.empty((21,0,3))
            if (len(obs)>2560 or len(refs) not in (0,21,105) or peer.ndim!=3 or peer.shape[0]!=21
                    or peer.shape[2]!=3 or peer.shape[1]>5 or len(terrain) not in (0,5)
                    or not all(np.isfinite(x).all() for x in (obs,refs,peer)) or np.any(obs[:,3]<=0)):
                raise ValueError('Invalid bounded planning observation')
            if bool(reference_members)!=(len(refs)==105) or any(i not in range(5) for i in reference_members):
                raise ValueError('Invalid YOPO reference member set')
            member_flags=sum(1<<(2+i) for i in set(reference_members))
            values.extend([int(check_gradient) | (2 if flexible else 0) | member_flags,
                           len(obs),len(refs),peer.shape[1],len(terrain)])
        for i in range(5):
            for x in arrays:values.extend(x[i])
        if not self.legacy:
            values.extend(obs.ravel());values.extend(refs.ravel());values.extend(peer.ravel())
            for item in terrain:
                height=np.asarray(item['values'],float);origin=np.asarray(item['origin'],float);resolution=float(item['resolution'])
                if height.shape!=(31,31) or origin.shape!=(2,) or not np.isfinite(origin).all() or not .1<=resolution<=4.:
                    raise ValueError('Invalid observed terrain grid')
                values.extend([resolution,*origin]);values.extend(np.where(np.isfinite(height),height,1e9).ravel())
        self.process.stdin.write((' '.join(format(float(x),'.15g') for x in values)+'\n').encode());self.process.stdin.flush()
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            while b'\n' in self.buffer:
                line,self.buffer=self.buffer.split(b'\n',1)
                marker=line.find(b'TERMINAL ')
                if marker<0:continue
                data=json.loads(line[marker+9:])
                if data['serial']!=self.serial:raise ValueError('Native terminal token mismatch')
                if not data['accepted']:return None,data['stats']
                return [NativeTrajectory(x['durations'],x['coefficients'],data['stats']['milliseconds']) for x in data['plans']],data['stats']
            ready,_,_=select.select([self.process.stdout],[],[],max(0.,deadline-time.monotonic()))
            if ready:
                chunk=os.read(self.process.stdout.fileno(),65536)
                if not chunk:raise RuntimeError('Native terminal optimizer exited')
                self.buffer+=chunk
        raise TimeoutError('Native terminal optimizer exceeded deadline')
