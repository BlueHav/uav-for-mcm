"""Validation for the compact 50 Hz Unity swarm-state channel.

The state message carries only what the safety supervisor and the HUD need at
high rate: position, velocity, AGL, odometry age and the Unity collision /
bounds flags for the ten bodies. Mission strings and cumulative evidence stay
on the slower /scene/status channel. Parsing is strict; a malformed sample is
rejected instead of silently re-dating a stale pose.
"""
import json
import math

DRONE_COUNT = 10
STATE_MAX_AGE_S = 0.25


def _number(value, name):
    if isinstance(value, bool):
        raise ValueError('Boolean where number required: ' + name)
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise ValueError('Non-numeric field: ' + name)
    if not math.isfinite(result):
        raise ValueError('Non-finite field: ' + name)
    return result


def parse_swarm_state(text, received):
    """Parse one compact state sample.

    `received` must be the real local clock at receipt (rospy.Time.now()).
    Returns positions/velocities ordered by physical drone id, plus the union
    of the collision and bounds flags. Raises ValueError on any mismatch.
    """
    if not isinstance(text, str):
        raise ValueError('State payload must be text')
    try:
        data = json.loads(text)
    except (TypeError, ValueError):
        raise ValueError('Invalid state JSON')
    if not isinstance(data, dict):
        raise ValueError('State must be an object')
    raw_stamp = data.get('stamp')
    if raw_stamp is None:
        # Unity JsonUtility serializes long exactly; milliseconds avoid any
        # double precision question in the C# producer.
        stamp = _number(data.get('stamp_ms'), 'stamp_ms') / 1000.0
    else:
        stamp = _number(raw_stamp, 'stamp')
    received = _number(received, 'received')
    age = received - stamp
    # Unity stamps UTC seconds; ROS wall time is the same source within the
    # measured few-millisecond transport delay. Reject future or old samples.
    if not -.25 <= age <= 1.0:
        raise ValueError('State sample is stale or future: %.3f s' % age)
    frame = data.get('scene_frame')
    if isinstance(frame, bool) or not isinstance(frame, int) or frame < 0:
        raise ValueError('Invalid scene frame')
    drones = data.get('drones')
    if not isinstance(drones, list) or len(drones) != DRONE_COUNT:
        raise ValueError('State must contain exactly ten drones')
    by_id = {}
    for record in drones:
        if not isinstance(record, dict):
            raise ValueError('Drone record must be an object')
        identity = record.get('id')
        if isinstance(identity, bool) or not isinstance(identity, int):
            raise ValueError('Drone id must be an integer')
        if not 0 <= identity < DRONE_COUNT or identity in by_id:
            raise ValueError('Duplicate or out-of-range drone id')
        position = [_number(record.get(k), 'xyz') for k in ('x', 'y', 'z')]
        velocity = [_number(record.get(k), 'vxyz') for k in ('vx', 'vy', 'vz')]
        agl = _number(record.get('agl'), 'agl')
        odom_age = _number(record.get('odom_age'), 'odom_age')
        if odom_age < -1.0:
            raise ValueError('odometry age outside contract')
        if not isinstance(record.get('collision'), bool) or not isinstance(record.get('bounds'), bool):
            raise ValueError('Collision/bounds flags must be booleans')
        by_id[identity] = dict(id=identity, position=position, velocity=velocity, agl=agl,
                               odom_age=odom_age, collision=record['collision'], bounds=record['bounds'])
    ordered = [by_id[i] for i in range(DRONE_COUNT)]
    odom_max = max(record['odom_age'] for record in ordered)
    return dict(
        stamp=stamp,
        received=received,
        age=age,
        scene_frame=frame,
        positions=[record['position'] for record in ordered],
        velocities=[record['velocity'] for record in ordered],
        agl=[record['agl'] for record in ordered],
        odom_age=[record['odom_age'] for record in ordered],
        odom_max_age=odom_max,
        # The mirror is unsafe to use for a high-rate separation check once
        # its odometry source is older than the control period.
        odom_stale=odom_max > 0.5,
        collision=any(record['collision'] for record in ordered),
        bounds=any(record['bounds'] for record in ordered),
        collision_ids=[record['id'] for record in ordered if record['collision']],
        bounds_ids=[record['id'] for record in ordered if record['bounds']],
    )


def fresh(live, now, max_age=STATE_MAX_AGE_S):
    """True when a parsed sample was received within the live-state window."""
    if live is None:
        return False
    try:
        age = float(now) - float(live['received'])
    except (KeyError, TypeError, ValueError):
        return False
    return math.isfinite(age) and 0.0 <= age <= float(max_age)
