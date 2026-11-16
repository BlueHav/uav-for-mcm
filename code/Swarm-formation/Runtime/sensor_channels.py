"""Split-channel layout for Unity/ROS sensor, state and display traffic.

Pure constants and topic builders so the rates and the group ownership are
defined once and can be unit tested without ROS, numpy or native libraries.

Channel policy (2026-11-09):
  * state/velocity/collision flags: compact 50 Hz channel, always all ten.
  * full scene evidence (mission strings, counters, map hash): 5 Hz.
  * ground height point: 20 Hz for every aircraft.
  * local terrain cloud: 3 Hz, produced by each group leader only and reused
    by the whole group.
  * terrain planning context: 5 Hz, produced by each group leader only and
    reused by the whole group.
  * depth images: only the cameras YOPO consumes (front/up/down); group
    leaders during the cloud cruise, all ten from the YOPO handoff on.
  * display clouds: separate /display namespace at 0.5-1 Hz.
"""
GROUP_SIZE = 5
GROUP_COUNT = 2
DRONE_COUNT = GROUP_SIZE * GROUP_COUNT
# Physical IDs of the two group leaders; groups are 0-4 and 5-9.
GROUP_LEADERS = (2, 7)
GROUP_LEADER_IDS = frozenset(GROUP_LEADERS)

STATE_RATE_HZ = 50.0
EVIDENCE_RATE_HZ = 5.0
GROUND_POINT_RATE_HZ = 20.0
GROUP_CLOUD_RATE_HZ = 3.0
CONTEXT_RATE_HZ = 5.0
DEPTH_RATE_HZ = 5.0
DISPLAY_LEADER_HZ = 1.0
DISPLAY_MEMBER_HZ = 0.5
STATE_MAX_AGE_S = 0.25


def _drone_id(drone_id):
    if isinstance(drone_id, bool) or not isinstance(drone_id, int):
        raise ValueError('Drone id must be an integer')
    if not 0 <= drone_id < DRONE_COUNT:
        raise ValueError('Drone id outside 0..9')
    return drone_id


def _group_id(group):
    if isinstance(group, bool) or not isinstance(group, int) or group not in range(GROUP_COUNT):
        raise ValueError('Group id must be 0 or 1')
    return group


def group_of(drone_id):
    return _drone_id(drone_id) // GROUP_SIZE


def group_leader(group):
    return GROUP_LEADERS[_group_id(group)]


def is_group_leader(drone_id):
    drone_id = _drone_id(drone_id)
    return drone_id == GROUP_LEADERS[group_of(drone_id)]


def group_members(group):
    base = GROUP_SIZE * _group_id(group)
    return tuple(range(base, base + GROUP_SIZE))


def group_ground_raw(group):
    return '/sensors/raw/swarm10/group_%d/ground' % _group_id(group)


def group_cloud_raw(group):
    return '/sensors/raw/swarm10/group_%d/local_ground' % _group_id(group)


def group_ground_topic(group):
    return '/swarm10/group_%d/ground' % _group_id(group)


def group_cloud_topic(group):
    return '/swarm10/group_%d/local_ground' % _group_id(group)


def group_context_topic(group, world_hash):
    if not isinstance(world_hash, str) or len(world_hash) != 64:
        raise ValueError('World hash must be the 64 character map fingerprint')
    return '/drone_%d/terrain_context/%s' % (group_leader(group), world_hash)


def own_ground_topic(drone_id):
    return '/drone_%d/ground' % _drone_id(drone_id)


def own_cloud_topic(drone_id):
    return '/drone_%d/local_ground' % _drone_id(drone_id)


def depth_profile(cloud_mission, planning_epoch, count=DRONE_COUNT):
    """Which aircraft render depth and at what rate.

    The point mission runs from the cloud cruise into the YOPO handoff. Until
    that handoff only the two group leaders need depth (cruise display and
    source reuse); from the warmup epoch on every aircraft needs its own YOPO
    input. Legacy non-cloud modes keep every aircraft streaming so their
    existing YOPO input does not regress.
    """
    if isinstance(count, bool) or not isinstance(count, int) or count not in (1, 2, 5, 10):
        raise ValueError('Unsupported drone count')
    if isinstance(planning_epoch, bool) or not isinstance(planning_epoch, int) or planning_epoch < 0:
        raise ValueError('Invalid planning epoch')
    if not cloud_mission:
        ids = tuple(range(count))
    elif planning_epoch >= 1:
        ids = tuple(range(DRONE_COUNT))
    else:
        ids = GROUP_LEADERS
    return dict(ids=ids, rate_hz=DEPTH_RATE_HZ)


def depth_enabled(cloud_mission, planning_epoch, drone_id, count=DRONE_COUNT):
    return _drone_id(drone_id) in depth_profile(cloud_mission, planning_epoch, count)['ids']


def display_rate(drone_id):
    return DISPLAY_LEADER_HZ if is_group_leader(drone_id) else DISPLAY_MEMBER_HZ
