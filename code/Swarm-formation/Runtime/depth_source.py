"""Select one sensor producer and verify a compatible Unity player before launch."""
import hashlib
import json
import os
from pathlib import Path


def depth_source(environ=None):
    source = (os.environ if environ is None else environ).get('YOPO_DEPTH_SOURCE', 'unity')
    if source not in ('unity', 'pointcloud'):
        raise ValueError('YOPO_DEPTH_SOURCE must be unity or pointcloud')
    return source


def sensor_ports(count, source):
    depth_source({'YOPO_DEPTH_SOURCE': source})
    if source == 'pointcloud' and count != 10:
        raise ValueError('Pointcloud depth currently requires the ten-drone mission')
    return [11322, 10012, 10013]


def validate_player(root, source):
    depth_source({'YOPO_DEPTH_SOURCE': source})
    build = Path(root) / 'Build'
    player = build / 'MountainCity.x86_64'
    if not player.is_file():
        raise RuntimeError('Unity player missing: ' + str(player))
    if source == 'unity':
        return
    try:
        marker = json.loads((build / 'depth-source-capabilities.json').read_text())
        digest = hashlib.sha256(player.read_bytes()).hexdigest()
        assembly = build / 'MountainCity_Data/Managed/Assembly-CSharp.dll'
        assembly_digest = hashlib.sha256(assembly.read_bytes()).hexdigest()
        valid = (marker['schema'] == 1 and source in marker['sources'] and
                 marker['player_sha256'] == digest and marker['assembly_sha256'] == assembly_digest)
    except (OSError, ValueError, KeyError, TypeError):
        valid = False
    if not valid:
        raise RuntimeError('Pointcloud mode requires the verified BuildCloudDepthMission player; '
                           'refusing an incompatible player that could publish duplicate sensors')


def validate_scene_source(scene, source, count):
    """Validate a fresh actual scene heartbeat, including split-terminal startup."""
    depth_source({'YOPO_DEPTH_SOURCE': source})
    if scene.get('depth_source', 'unity') != source or scene.get('drone_count') != count:
        raise RuntimeError('Running Unity sensor source/count does not match the ROS mission')


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--scene-root', required=True)
    parser.add_argument('--count', type=int, default=10)
    args = parser.parse_args()
    source = depth_source()
    sensor_ports(args.count, source)
    validate_player(args.scene_root, source)
    print(source)
