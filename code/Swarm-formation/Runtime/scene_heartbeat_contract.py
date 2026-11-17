"""Reject old Unity players before preparing the strict scene-frame Runtime."""
import hashlib,json
from pathlib import Path

def require_scene_heartbeat_build(build,project=None):
    build=Path(build).resolve();manifest=build/'scene-heartbeat-build.json'
    if not manifest.is_file():raise ValueError('Unity scene_frame build required: missing '+str(manifest))
    value=json.loads(manifest.read_text())
    if type(value.get('schema_version')) is not int or value['schema_version']!=1 or value.get('frame_field')!='scene_frame' or value.get('sensor_frame_field')!='frames':raise ValueError('Unsupported Unity scene heartbeat contract')
    if type(value.get('sensor_service_contract')) is not int or value['sensor_service_contract']!=1 or value.get('yopo_input_topic')!='/drone_{id}/yopo/input_depth' or value.get('terrain_source')!='map_terrain_sample_not_radar':raise ValueError('Unified sensor and actual YOPO input build required')
    dll=build/'MountainCity_Data/Managed/Assembly-CSharp.dll'
    if not dll.is_file() or hashlib.sha256(dll.read_bytes()).hexdigest()!=value.get('dll_sha256'):raise ValueError('Unity scene heartbeat assembly hash mismatch')
    if project is not None:
        expected={'Assets/FleetHud.cs', 'Assets/YopoInputDepthFrame.cs', 'Assets/TrackerFieldYopoInputDepth.cs', 'Assets/Editor/YopoInputDepthAudit.cs', 'Assets/Editor/SceneHeartbeatBuild.cs', 'Assets/TrackerVerticalSensors.cs', 'Assets/TrackerAsyncDepth.cs', 'Assets/TrackerField.cs', 'Assets/SixLineAudit.cs', 'Assets/TrackerFieldSensors.cs', 'Assets/MapMarkers.cs'}
        hashes=value.get('source_hashes',[])
        if not isinstance(hashes,list) or len(hashes)!=len(expected) or {x.get('path') for x in hashes}!=expected:raise ValueError('Missing scene heartbeat source hashes')
        for item in hashes:
            source=Path(project)/item['path']
            if not source.is_file() or hashlib.sha256(source.read_bytes()).hexdigest()!=item.get('sha256'):raise ValueError('Unity scene heartbeat source/build mismatch: '+item['path'])
    return dict(sensor_service_contract=1,yopo_input_topic=value['yopo_input_topic'],terrain_source=value['terrain_source'],schema_version=1,frame_field='scene_frame',sensor_frame_field='frames',dll_sha256=value['dll_sha256'],build=str(build),source_hashes=value.get('source_hashes',[]))
