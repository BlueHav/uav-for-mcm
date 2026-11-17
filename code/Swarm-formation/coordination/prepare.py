import json,os,hashlib,xml.etree.ElementTree as E
from pathlib import Path
from shared_policy import CFG
from coordination_config import validate_config
B=Path(__file__).parent;R=Path(('__UAV_PROJECT__/code/Swarm-formation/Runtime'.replace("__UAV_PROJECT__", __import__("os").environ["UAV_PROJECT_ROOT"])));C=Path(('__UAV_PROJECT__/config/point.json'.replace("__UAV_PROJECT__", __import__("os").environ["UAV_PROJECT_ROOT"])));c=json.loads(C.read_text());W=Path(('__UAV_PROJECT__'.replace("__UAV_PROJECT__", __import__("os").environ["UAV_PROJECT_ROOT"])))/c['paths']['map']/c['paths']['map_files']['world'];w=json.loads(W.read_text());start=c['points']['start'];x=start['x'];y=start['y'];t=w['terrain'];bounds=w['bounds']
for k in ['world','metadata']:
 p=W.parent/c['paths']['map_files'][k]
 if hashlib.sha256(p.read_bytes()).hexdigest()!=c['map_fingerprints'][k]:raise ValueError('Map hash mismatch '+k)
def height(x,y):
 n=t['width'];u=(x-bounds['x_min'])/(bounds['x_max']-bounds['x_min'])*(n-1);v=(bounds['y_max']-y)/(bounds['y_max']-bounds['y_min'])*(t['height']-1);col=min(int(u),n-2);r=min(int(v),t['height']-2);u-=col;v-=r;h=t['heights'];a=h[r*n+col];b=h[r*n+col+1];d=h[(r+1)*n+col];e=h[(r+1)*n+col+1];return a+(b-a)*u+(d-a)*v if u+v<=1 else e+(d-e)*(1-u)+(b-e)*(1-v)
spawns=[[x,y-8,height(x,y-8)],[x,y+8,height(x,y+8)]]
root=E.parse(R/'point_controller.launch').getroot()
for child in list(root):
 if child.tag=='group' and child.get('ns') not in ('drone_0','drone_1'):root.remove(child)
for g in root.findall('group'):
 i=int(g.get('ns').split('_')[-1])
 for n in g.findall('node'):
  for p in n.findall('param'):
   key=p.get('name')
   if key=='initial_ground_z':p.set('value',str(spawns[i][2]))
   if key=='simulator/init_state_x':p.set('value',str(spawns[i][0]))
   if key=='simulator/init_state_y':p.set('value',str(spawns[i][1]))
E.ElementTree(root).write(B/'controllers.launch',encoding='unicode')
l=E.Element('launch');E.SubElement(l,'include',file=str(B/'controllers.launch'))
n=E.SubElement(l,'node',pkg='ros_tcp_endpoint',type='default_server_endpoint.py',name='unity_tcp_endpoint',required='true',output='screen',**{'launch-prefix':'/usr/bin/python3 -u'});E.SubElement(n,'param',name='tcp_ip',value='127.0.0.1');E.SubElement(n,'param',name='tcp_port',value='10012')
E.SubElement(l,'node',pkg='yopo_ten_mission',type='central_sensor_service.py',name='central_sensor_service',required='true',output='screen',**{'launch-prefix':'/usr/bin/python3 -u'})
for i in range(2):
 n=E.SubElement(l,'node',pkg='yopo_shared_state',type='mission_full.py',name='mission',ns='drone_'+str(i),required='true',output='screen',**{'launch-prefix':'/usr/bin/python3 '+str(B/'wait_for_unity.py')+' $(env YOPO_PYTHON) -u'});E.SubElement(n,'param',name='drone_id',value=str(i));E.SubElement(n,'param',name='count',value='1')
E.ElementTree(l).write(B/'full.launch',encoding='unicode')
manifest=dict(spawns=spawns,points=c['points'],routes=c['plan']['cruise_routes'],map_hash=c['map_fingerprints']['world'],cruise_altitude=270,pair_limits=[2,2000],stop_reserve=[3.5,1998.5],command_speed_cap=10.5,user_speed_limit=15,assembly_agl=2,landing_agl=5,terminal_end_agl=2,point_json_hash=hashlib.sha256(C.read_bytes()).hexdigest(),production_hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [R/'mission_node.py',R/'local_map.py',Path(('__UAV_PROJECT__/code/YOPO/mission_swarm.py'.replace("__UAV_PROJECT__", __import__("os").environ["UAV_PROJECT_ROOT"]))),Path(('__UAV_PROJECT__/code/YOPO/swarm_adapter.py'.replace("__UAV_PROJECT__", __import__("os").environ["UAV_PROJECT_ROOT"])))]})
manifest['scope']='Two actual Unity aircraft, two count1 executors, central direct-global-odom safety policy; no peer/PolyTraj';manifest.pop('pair_limits',None);manifest.pop('stop_reserve',None)
manifest['central_policy']=CFG
L=c['points']['landing'];region_center=[L['x'],L['y']]
entries={}
for i,r in enumerate(c['plan']['cruise_routes']):
 previous=r[-2];dx=L['x']-previous['x'];dy=L['y']-previous['y'];length=(dx*dx+dy*dy)**.5
 entries[i]=[L['x']-1200*dx/length,L['y']-1200*dy/length,270.]
handoff_xy={0:[L['x']+30,L['y']+100],1:[L['x']-70,L['y']-80]}
handoffs={i:xy+[height(*xy)+5] for i,xy in handoff_xy.items()}
groups=json.loads(os.environ.get('YOPO_GROUPS_JSON','[{"id":"A","members":[0],"route_id":0},{"id":"B","members":[1],"route_id":1}]'))
routes={i:dict(entry=entries[i],handoff=handoffs[i],end=[c['points'][key]['x'],c['points'][key]['y'],height(c['points'][key]['x'],c['points'][key]['y'])+2]) for i,key in enumerate(('end','end2'))}
configuration=validate_config(dict(groups=groups,routes=routes,region_center=region_center,region_radius_m=120.),deployment=True)
manifest['coordination_config']=configuration
manifest['parallel_descent']=dict(entries=entries,handoffs=handoffs,region_center=region_center,region_radius_m=120.,timing_metrics='Descriptive only: actual start and region passage differences, simultaneous movement fraction; no cross-group timing synchronization',scope='two_group_leader_proxies; independent phases and regional handoffs; cross-group coordination is distance safety only; future 2x5 flight not accepted')
manifest['deployment_role']='two_group_leader_proxies'
manifest['group_leader_proxies']={g['id']:dict(actual_drone_id=g['members'][0],intended_group_members=5) for g in configuration['groups']}
(B/'coordination-config.json').write_text(json.dumps(configuration,indent=2))
override=(B/'mission_runtime_override.py').resolve()
manifest['runtime_override']=dict(path=str(override),sha256=hashlib.sha256(override.read_bytes()).hexdigest(),hook='shape_desired: after AGL, before original jerk integration in control and final_sweep')
manifest['graphics_backend']=os.environ.get('YOPO_GRAPHICS_BACKEND','vulkan')
manifest['execution_source_hashes']={str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(B.glob('*.py'))+[B/'run.sh']}
(B/'test-manifest.json').write_text(json.dumps(manifest,indent=2));print(json.dumps(manifest))
