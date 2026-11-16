#!/usr/bin/env python3
"""Drive one point-mission run from the command line: assemble -> climb -> landed."""
import json
import sys
import time

import rospy
from std_msgs.msg import String

MODE = sys.argv[1] if len(sys.argv) > 1 else 'five'
BUDGET = float(sys.argv[2]) if len(sys.argv) > 2 else 3900.0
OUT = sys.argv[3] if len(sys.argv) > 3 else '/tmp/driver.json'
ACTIVE = tuple(range(5)) if MODE == 'five' else tuple(range(10))

status = {'stage': None, 'reason': None, 'stamp': 0.0}
peers = {}


def cb(msg):
    global status
    try:
        status = json.loads(msg.data)
    except ValueError:
        pass


def peer_cb(msg, i):
    try:
        peers[i] = json.loads(msg.data) or {}
    except ValueError:
        pass


def log(line):
    print('[%s] %s' % (time.strftime('%H:%M:%S'), line), flush=True)


def main():
    rospy.init_node('point_mission_driver', disable_signals=True)
    rospy.Subscriber('/swarm10/status', String, cb, queue_size=1)
    for i in ACTIVE:
        rospy.Subscriber('/drone_%d/execution/peer_state' % i, String, peer_cb, callback_args=i, queue_size=1)
    pub = rospy.Publisher('/swarm10/command', String, queue_size=1, latch=True)

    log('waiting for coordinator status (mode=%s, active=%s)' % (MODE, list(ACTIVE)))
    started = time.time()
    while time.time() - started < 180 and status['stage'] is None and not rospy.is_shutdown():
        time.sleep(0.5)
    if status['stage'] is None:
        log('no /swarm10/status received')
        return 2
    log('initial stage=%s reason=%s' % (status['stage'], status['reason']))
    if status['stage'] != 'IDLE':
        log('refusing to start from stage %s' % status['stage'])
        return 2

    # The measurement heartbeat only exists once every executor process is up.
    # Starting before that makes the followers see stale peers on takeoff.
    deadline = time.time() + 120
    while time.time() < deadline and not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()
        fresh = [i for i in ACTIVE if i in peers and -.1 <= now - peers[i].get('stamp', 0) <= 1.2]
        if len(fresh) == len(ACTIVE):
            break
        time.sleep(0.5)
    missing = [i for i in ACTIVE if i not in peers]
    log('peer heartbeats: %d/%d (missing %s)' % (len(ACTIVE) - len(missing), len(ACTIVE), missing))
    if missing:
        log('refusing to start without every active executor heartbeat')
        return 2

    time.sleep(3.0)
    log('publishing assemble')
    pub.publish(String('{"op":"assemble"}'))
    rospy.sleep(1.0)

    last = None
    climb_sent = False
    deadline = time.time() + BUDGET
    while time.time() < deadline and not rospy.is_shutdown():
        stage = status.get('stage')
        if stage != last:
            last = stage
            log('stage=%s phases=%s reason=%s' % (stage, status.get('group_phases'), status.get('reason')))
            with open(OUT, 'w') as handle:
                json.dump(status, handle, indent=2)
            if stage == 'ASSEMBLED' and not climb_sent:
                time.sleep(3.0)
                log('publishing climb')
                pub.publish(String('{"op":"climb"}'))
                climb_sent = True
            if stage in ('LANDED', 'FAILED'):
                break
        time.sleep(0.5)
    with open(OUT, 'w') as handle:
        json.dump(status, handle, indent=2)
    ok = status.get('stage') == 'LANDED'
    log('finished stage=%s ok=%s reason=%s' % (status.get('stage'), ok, status.get('reason')))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
