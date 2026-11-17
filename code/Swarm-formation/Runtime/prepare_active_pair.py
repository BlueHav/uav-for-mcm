"""Prepare a frozen pair deployment; does not change Unity or single run outputs."""
import argparse,copy,hashlib,json,shutil
import xml.etree.ElementTree as ET
from pathlib import Path
from prepare_active_single import prepare as prepare_single
from point_mission_core import PointMission
from assembly_region import validate_single_descent
from pair_climb_geometry import pair_climb_geometry

def prepare(point,output):
    runtime=Path(__file__).resolve().parent;point=Path(point).resolve();output=Path(output).resolve()
    model=PointMission(point);model.forward_departure=True
    validation=validate_single_descent(model.config,model.world,1)
    prepare_single(point,output)
    for name in ('active_pair_node.py','active_pair_core.py','active_single_node.py','pair_climb_geometry.py','pair_climb_stopping.py'):
        shutil.copy2(runtime/name,output/name);(output/name).chmod(0o755)
    copied=output/'mission_runtime_override.py'
    text=copied.read_text();anchor='acceleration=[msg.acceleration.x,msg.acceleration.y,msg.acceleration.z])'
    if text.count(anchor)!=1:raise ValueError('Unknown atomic control reference layout')
    copied.write_text(text.replace(anchor,"acceleration=[msg.acceleration.x,msg.acceleration.y,msg.acceleration.z],shaped_desired=getattr(self,'last_control_shaping',{}).get('desired_after'))"))
    shared=output/'mission_full.py';text=shared.read_text()
    init_anchor="  rospy.Subscriber('/shared_pair/decision',String,self.central_command,queue_size=1,tcp_nodelay=True)"
    control_anchor="  with self.shaping_scope('control'):super().control(event)"
    if text.count(init_anchor)!=1 or text.count(control_anchor)!=1:raise ValueError('Unknown shared control proof publication layout')
    text=text.replace(init_anchor,"  self.parallel_proof_pub=rospy.Publisher('parallel_climb/control_reference',String,queue_size=1,tcp_nodelay=True)\n"+init_anchor)
    text=text.replace(control_anchor,control_anchor+"\n  reference=getattr(self,'last_control_reference',None)\n  if reference is not None and hasattr(self,'parallel_proof_pub'):self.parallel_proof_pub.publish(String(json.dumps(reference)))")
    shared.write_text(text)
    controllers=ET.parse(runtime/'point_controller.launch').getroot()
    for child in list(controllers):
        if child.tag=='group' and child.get('ns') not in ('drone_2','drone_7'):controllers.remove(child)
    spawns=model.spawn()
    for group in controllers.findall('group'):
        i=int(group.get('ns').split('_')[-1]);spawn=spawns[i];ground=float(model.ground(spawn[None,:2])[0])
        for param in group.iter('param'):
            values={'initial_ground_z':ground,'simulator/init_state_x':spawn[0],'simulator/init_state_y':spawn[1]}
            if param.get('name') in values:param.set('value',str(values[param.get('name')]))
    ET.ElementTree(controllers).write(output/'controllers.launch',encoding='unicode')
    launch=ET.parse(output/'single.launch').getroot()
    mission=next(n for n in launch.findall('node') if n.get('type')=='mission_full.py')
    peer=copy.deepcopy(mission);peer.set('ns','drone_7')
    next(p for p in peer.findall('param') if p.get('name')=='drone_id').set('value','7');launch.append(peer)
    coordinator=next(n for n in launch.findall('node') if n.get('type')=='active_single_node.py')
    coordinator.set('type','active_pair_node.py');coordinator.set('name','active_pair_coordinator')
    # Visible map context is sampled terrain, not measured radar returns.
    ET.SubElement(launch,'node',pkg='yopo_ten_mission',type='terrain_display_node.py',name='terrain_context',required='true',output='screen',**{'launch-prefix':'/usr/bin/python3 -u'})
    ET.SubElement(launch,'node',pkg='rviz',type='rviz',name='rviz',args='-d '+str(runtime/'pair.rviz'),output='screen',**{'launch-prefix':'/bin/bash '+str(runtime/'rviz_render.sh')})
    ET.ElementTree(launch).write(output/'pair.launch',encoding='unicode')
    manifest=json.loads((output/'deployment.json').read_text());manifest.update(active_ids=[2,7],parallel_climb_geometry=pair_climb_geometry(model),scope='pair integration; flight acceptance pending',second_descent_validation=validation,deployed_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(output.glob('*.py'))})
    (output/'deployment.json').write_text(json.dumps(manifest,indent=2));return output/'pair.launch'
if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--point',required=True);parser.add_argument('--output',required=True)
    args=parser.parse_args();print(prepare(args.point,args.output))
