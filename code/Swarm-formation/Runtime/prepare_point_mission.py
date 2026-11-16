"""Prepare isolated simulator inputs and validate fixed transitions before flight."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from point_mission_core import PointMission
from ten_core import segment_clearance
from formation_core import transition,min_separation
from cloud_config import configure_model,CRUISE_MINIMUM

def prepare(root):
    root=Path(root);m=PointMission(root/'point.json');runtime=Path(__file__).resolve().parent
    configure_model(m)
    p=m.spawn();checks=[]
    if min_separation(p)<2 or not np.isfinite(p).all():raise ValueError('Invalid spawn slots')
    tree=ET.parse(runtime/'ten_controller.launch')
    for group in tree.getroot().findall('group'):
        i=int(group.attrib['ns'].split('_')[-1]);node=group.find("node[@name='dynamics']")
        for name,value in [('simulator/init_state_x',p[i,0]),('simulator/init_state_y',p[i,1]),('initial_ground_z',m.ground(p[i:i+1,:2])[0])]:
            item=node.find("param[@name='%s']"%name);item.set('value',str(float(value)))
    tree.write(runtime/'point_controller.launch',encoding='unicode')
    for g in range(2):
        before=p[5*g:5*g+5].copy();before[:,2]=m.ground(before[:,:2])+m.assembly_agl
        assembly=m.assembly(g)
        for phase,end in [('AIR_ASSEMBLE',assembly)]+m.departure(g,assembly):
            ok=m.sweep_clear(before,end) and segment_clearance(before,end)>=1.9
            checks.append(dict(group=g,phase=phase,passed=bool(ok)))
            if not ok:raise ValueError('Departure geometry failed '+phase)
            before=end
        entry,_=m.descent(g)
        if getattr(m,'moving_merge',False):
            from moving_merge import MovingMerge
            initial=m.line(g,entry[2,:2],270.,m.direction)
            plan=MovingMerge(m,g,initial,np.tile(np.r_[m.direction*m.cruise_speed,0.],(5,1)))
            samples=[plan.sample(t) for t in np.linspace(0.,plan.duration,int(np.ceil(plan.duration/.2))+1)]
            positions=np.array([s[0] for s in samples]);velocities=np.array([s[1] for s in samples])
            flat=positions.reshape(-1,3)
            if np.min(flat[:,2]-m.ground(flat[:,:2]))<4.5:raise ValueError('Moving descent terrain clearance')
            separation=min(segment_clearance(a,b) for a,b in zip(positions[:-1],positions[1:]))
            if separation<2.4:raise ValueError('Moving descent body clearance')
            checks.append(dict(group=g,phase='MOVING_MERGE',duration_s=plan.duration,
                               peak_horizontal_mps=float(np.linalg.norm(velocities[:,:,:2],axis=2).max()),
                               min_body_distance=separation,passed=True))
        else:
            direction=entry[2,:2]-m.routes[g][-2];direction/=np.linalg.norm(direction)
            stopped=m.line(g,entry[2,:2]-100*direction,270.,direction)
            path=[stopped]+[q for _,q in m.descent_merge(g,stopped)]
            for part,(a,b) in enumerate(zip(path[:-1],path[1:])):
                if not m.sweep_clear(a,b,4.5) or segment_clearance(a,b)<2.4:raise ValueError('Descent corridor not clear')
                checks.append(dict(group=g,phase='DESCEND',part=part,passed=True))
        slots=m.landing_slots(g);top=slots.copy();top[:,2]+=2.
        if not m.sweep_clear(top,slots+np.array([0,0,.6]),.6):raise ValueError('Landing slot obstructed')
        checks.append(dict(group=g,phase='LAND',passed=True))
    out=root/'verification/2026-10-31';out.mkdir(parents=True,exist_ok=True)
    (out/'point-static.json').write_text(json.dumps(dict(hashes=m.hashes,profile='cloud-current',cruise_target=m.cruise_speed,cruise_minimum=CRUISE_MINIMUM,checks=checks,spawn=p.tolist(),landing=[m.landing_slots(g).tolist() for g in range(2)],scope='current departure, moving descent terrain/body geometry and landing slots; cross-group and stopping tests separate; live sensing and Unity flight still required'),indent=2))
    resources=root/'scene/Assets/Resources';resources.mkdir(parents=True,exist_ok=True)
    (resources/'MissionSpawn.json').write_text(json.dumps(dict(points=[dict(x=float(q[0]),y=float(q[1]),z=float(q[2])) for q in p])))
    print(json.dumps(dict(static_checks=len(checks),passed=True,hashes=m.hashes)))

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--root',default=('__UAV_PROJECT__'.replace("__UAV_PROJECT__", __import__("os").environ["UAV_PROJECT_ROOT"])));prepare(parser.parse_args().root)
