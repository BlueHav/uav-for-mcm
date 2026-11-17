"""Common horizontal star geometry; all five members retain their slots."""
import numpy as np

UNIT_STAR=np.array([[0.,-1.],[-1.,0.],[0.,0.],[1.,0.],[0.,1.]])
COMPACT_RADIUS=3.3
TRAVEL_RADIUS=36.
AVOIDANCE_DEVIATION=8.0

def shape_state(positions):
    p=np.asarray(positions,float)
    relative=p[...,:,:2]-p[...,2:3,:2]
    radius=np.sum(relative*UNIT_STAR,axis=(-2,-1))/4.
    residual=relative-radius[...,None,None]*UNIT_STAR
    return radius,np.linalg.norm(residual,axis=-1).max(axis=-1)

def desired_radius(remaining):
    return COMPACT_RADIUS if remaining<=40. else TRAVEL_RADIUS

def formation_kept(plans,flexible=False):
    if len(plans)!=5:return False
    T=plans[0].duration
    times=np.linspace(0.,T,max(2,int(np.ceil(T/.02))+1))
    samples=np.stack([p.positions(times) for p in plans],axis=1)
    radius,error=shape_state(samples)
    # Fit ONE common quintic to radius P/V/A boundaries. The next finite
    # packet can start while the previous packet is expanding/contracting,
    # but still has a stationary fallback tail. Position-only star tests
    # would miss an arbitrary common radial excursion between endpoints.
    head=np.array([p.sample(0.) for p in plans]);tail=np.array([p.sample(T) for p in plans])
    if np.max(np.linalg.norm(tail[:,1:],axis=2))>.001:return False
    nominal=getattr(plans[0],'nominal_duration',None)
    if any(getattr(p,'nominal_duration',None)!=nominal for p in plans):return False
    boundaries=[0.,T] if nominal is None else [0.,nominal,T]
    expected=np.empty_like(radius)
    for begin,end in zip(boundaries[:-1],boundaries[1:]):
        if not 0<=begin<end<=T:return False
        h=np.array([p.sample(begin) for p in plans]);z=np.array([p.sample(end) for p in plans])
        r0,r1=shape_state(h[:,0])[0],shape_state(z[:,0])[0]
        rv,ra=[float(np.sum((h[:,j,:2]-h[2,j,:2])*UNIT_STAR)/4.) for j in (1,2)]
        ev,ea=[float(np.sum((z[:,j,:2]-z[2,j,:2])*UNIT_STAR)/4.) for j in (1,2)]
        dt=end-begin
        matrix=np.array([[dt**3,dt**4,dt**5],[3*dt**2,4*dt**3,5*dt**4],[6*dt,12*dt**2,20*dt**3]])
        c=np.linalg.solve(matrix,[r1-r0-rv*dt-ra*dt*dt/2.,ev-rv-ra*dt,ea-ra])
        selected=(times>=begin)&(times<=end);t=times[selected]-begin
        expected[selected]=r0+rv*t+ra*t*t/2.+c[0]*t**3+c[1]*t**4+c[2]*t**5
    if flexible:
        progress=tail[2,0,:2]-head[2,0,:2]
        if np.linalg.norm(progress)<.25:return False
        nominal_end=T if nominal is None else nominal
        times_in_motion=np.linspace(0.,nominal_end,41)[1:-1]
        forward=progress/np.linalg.norm(progress)
        if any(np.min([plan.sample(t)[1][:2]@forward for t in times_in_motion])<=0. for plan in plans):
            return False
    return bool(np.isfinite(samples).all() and error[-1]<=.1
                and np.max(error)<=(AVOIDANCE_DEVIATION if flexible else .25)
                and np.max(abs(radius-expected))<=(.65 if flexible else .15)
                and radius.min()>=3.0 and radius.max()<=TRAVEL_RADIUS+.25)
