"""Bounded pipe protocol for actual Swarm-Formation A* and MINCO optimization."""
import os,select,subprocess,time
import numpy as np

class NativeTrajectory:
    def __init__(self,durations,coefficients,milliseconds):
        self.durations=np.asarray(durations,float);self.coefficients=np.asarray(coefficients,float)
        if not np.isfinite(self.durations).all() or np.any(self.durations<=0) or not np.isfinite(self.coefficients).all():raise ValueError('Invalid native polynomial')
        self.duration=float(self.durations.sum());self.milliseconds=milliseconds
    def sample(self,t):
        if not np.isfinite(t) or t<0 or t>self.duration+1e-8:raise ValueError('Trajectory horizon expired')
        edges=np.cumsum(self.durations);i=min(len(edges)-1,int(np.searchsorted(edges,t,side='right')))
        local=float(t-(edges[i-1] if i else 0.));c=self.coefficients[i]
        return tuple(np.array([np.polyval(np.polyder(row,order),local) for row in c]) for order in (0,1,2))
    def positions(self,times):
        times=np.asarray(times,float).reshape(-1)
        if not np.isfinite(times).all() or np.any(times<0) or np.any(times>self.duration+1e-8):raise ValueError('Trajectory horizon expired')
        edges=np.cumsum(self.durations);ids=np.minimum(len(edges)-1,np.searchsorted(edges,times,side='right'))
        local=times-np.r_[0.,edges[:-1]][ids];coeff=self.coefficients[ids]
        result=np.zeros((len(times),3))
        for k in range(6):result=result*local[:,None]+coeff[:,:,k]
        return result
    def samples(self,times):
        """Batch identical polynomial P/V/A evaluation at all certificate ticks."""
        times=np.asarray(times,float).reshape(-1)
        if not np.isfinite(times).all() or np.any(times<0) or np.any(times>self.duration+1e-8):raise ValueError('Trajectory horizon expired')
        edges=np.cumsum(self.durations);ids=np.minimum(len(edges)-1,np.searchsorted(edges,times,side='right'))
        local=times-np.r_[0.,edges[:-1]][ids];coeff=self.coefficients[ids]
        result=np.empty((len(times),3,3))
        for order,factors in enumerate(([1.,1.,1.,1.,1.,1.],[5.,4.,3.,2.,1.],[20.,12.,6.,2.])):
            value=np.zeros((len(times),3))
            for k,factor in enumerate(factors):value=value*local[:,None]+coeff[:,:,k]*factor
            result[:,order]=value
        return result

class NativePlanner:
    def __init__(self,binary):
        self.process=subprocess.Popen([str(binary)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL)
        self.buffer=b'';self.serial=0;self.diagnostics=[]
    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait()
        self.process.stdin.close();self.process.stdout.close()
    def plan(self,grid,p,v,a,goal,end_velocity,cruise_z=-1,drone_id=2,peers=(),timeout=2.,max_speed=6.,certification_grid=None):
        self.serial+=1
        fields=[self.serial,*grid.ground.shape,grid.resolution,*(grid.origin[:2]*grid.resolution),300.,cruise_z,drone_id,len(peers),max_speed]
        arrays=[np.nan_to_num(grid.ground,nan=1e9).ravel(),p,v,a,goal,end_velocity]
        for ident,position,velocity in peers:arrays.extend(([ident],position,velocity))
        values=np.concatenate([np.asarray(item,float).ravel() for item in [fields]+arrays])
        self.process.stdin.write((' '.join(format(float(x),'.12g') for x in values)+'\n').encode());self.process.stdin.flush()
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            while b'\n' in self.buffer:
                line,self.buffer=self.buffer.split(b'\n',1)
                # Upstream optimizer emits an ANSI reset after its newline.
                marker=line.find(b'RESULT ')
                if marker<0:
                    self.diagnostics=(self.diagnostics+[line.decode(errors='replace')])[-10:];continue
                line=line[marker:]
                words=line.split();serial,ok,count=map(int,words[1:4]);ms=float(words[4])
                if serial!=self.serial:raise RuntimeError('Native result token mismatch')
                if not ok:self.diagnostics.append('Native rejected trajectory, ms='+str(ms));return None
                data=np.array([float(x) for x in words[5:]]).reshape(count,19)
                plan=NativeTrajectory(data[:,0],data[:,1:].reshape(count,3,6),ms)
                # Independently certify the whole path against the Python
                # footprint/peer model, including the endpoint.
                samples=plan.positions(np.linspace(0,plan.duration,max(2,int(np.ceil(plan.duration/.025))+1)))
                certificate=grid if certification_grid is None else certification_grid
                if certificate.points_clear(samples,radius=.35).all():return plan
                self.diagnostics.append('Python footprint rejected native path');return None
            ready,_,_=select.select([self.process.stdout],[],[],max(0,deadline-time.monotonic()))
            if ready:
                chunk=os.read(self.process.stdout.fileno(),65536)
                if not chunk:raise RuntimeError('Native planner exited')
                self.buffer+=chunk
        raise TimeoutError('Native planner deadline exceeded; pending output: '+repr(self.buffer[-500:]))
