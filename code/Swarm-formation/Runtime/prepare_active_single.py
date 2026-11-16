"""Prepare isolated count1 execution files; never modifies the Unity scene."""
import argparse
import hashlib
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path
from point_mission_core import PointMission
from scene_heartbeat_contract import require_scene_heartbeat_build
from assembly_region import validate_single_descent
from active_single_executor import status_runtime_source,single_executor_source


def prepare(point,output):
    runtime=Path(__file__).resolve().parent;coordination=runtime.parent/'coordination'
    contract=require_scene_heartbeat_build(runtime.parents[2]/'unity/Build',runtime.parents[2]/'unity/Project')
    point=Path(point).resolve();output=Path(output).resolve()
    model=PointMission(point);model.forward_departure=True
    validation=validate_single_descent(model.config,model.world,0)
    output.mkdir(parents=True,exist_ok=False)
    (output/'perception').mkdir()
    for p in coordination.glob('*.py'):
        shutil.copy2(p,output/p.name);(output/p.name).chmod(0o755)
    override=output/'mission_runtime_override.py'
    override.write_text(status_runtime_source(override.read_text()))
    executor=output/'single_executor.py';executor.write_text(single_executor_source(executor.read_text()))
    for name in ('active_single_core.py','assembly_region.py','active_single_executor.py','active_forward_climb.py'):
        shutil.copy2(runtime/name,output/name)
    shutil.copy2(coordination/'package.xml',output/'package.xml')
    shutil.copy2(runtime/'active_single_node.py',output/'active_single_node.py');(output/'active_single_node.py').chmod(0o755)
    controllers=ET.parse(runtime/'point_controller.launch').getroot()
    for child in list(controllers):
        if child.tag=='group' and child.get('ns')!='drone_2':controllers.remove(child)
    spawn=model.spawn()[2];ground=float(model.ground(spawn[None,:2])[0])
    for group in controllers.findall('group'):
        for param in group.iter('param'):
            values={'initial_ground_z':ground,'simulator/init_state_x':spawn[0],'simulator/init_state_y':spawn[1]}
            if param.get('name') in values:param.set('value',str(values[param.get('name')]))
    ET.ElementTree(controllers).write(output/'controllers.launch',encoding='unicode')
    launch=ET.Element('launch');ET.SubElement(launch,'include',file=str(output/'controllers.launch'))
    def node(pkg,kind,name,prefix='/usr/bin/python3 -u',**attributes):
        return ET.SubElement(launch,'node',pkg=pkg,type=kind,name=name,required='true',output='screen',**{'launch-prefix':prefix},**attributes)
    endpoint=node('ros_tcp_endpoint','default_server_endpoint.py','unity_tcp_endpoint')
    for key,value in [('tcp_ip','127.0.0.1'),('tcp_port','10012')]:ET.SubElement(endpoint,'param',name=key,value=value)
    node('yopo_ten_mission','central_sensor_service.py','central_sensor_service')
    mission=node('yopo_shared_state','mission_full.py','mission','$(env YOPO_PYTHON) -u',ns='drone_2')
    for key,value in [('drone_id','2'),('count','1')]:ET.SubElement(mission,'param',name=key,value=value)
    coordinator=node('yopo_shared_state','active_single_node.py','active_single_coordinator')
    for key,value in [('point',str(point)),('output',str(output))]:ET.SubElement(coordinator,'param',name=key,value=value)
    ET.ElementTree(launch).write(output/'single.launch',encoding='unicode')
    (output/'deployment.json').write_text(json.dumps(dict(scene_heartbeat_contract=contract,active_ids=[2],scene_count=10,point=str(point),descent_validation=validation,scope='single-aircraft integration; flight acceptance pending',source_hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for folder in (runtime,coordination) for p in sorted(folder.glob('*.py'))},point_sha256=hashlib.sha256(point.read_bytes()).hexdigest(),map_hashes=model.hashes,deployed_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(output.glob('*.py'))}),indent=2))
    return output/'single.launch'

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--point',required=True);parser.add_argument('--output',required=True)
    args=parser.parse_args();print(prepare(args.point,args.output))
