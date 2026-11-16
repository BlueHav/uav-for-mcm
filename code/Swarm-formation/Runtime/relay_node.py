#!/usr/bin/env python3
"""Publish a smooth slot entry followed by delayed group leader commands."""
import json
import math
import threading

import rospy
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import String

from relay_core import (CommandBuffer, RelayPlan, DELAY_S, GROUP_LEADERS,
                        followers, slot_bridge)


def _command(position, velocity, acceleration, yaw=0.0, yaw_dot=0.0):
    message = PositionCommand()
    message.header.stamp = rospy.Time.now()
    message.header.frame_id = 'world'
    message.position.x, message.position.y, message.position.z = position
    message.velocity.x, message.velocity.y, message.velocity.z = velocity
    message.acceleration.x, message.acceleration.y, message.acceleration.z = acceleration
    message.yaw = float(yaw)
    message.yaw_dot = float(yaw_dot)
    message.trajectory_flag = message.TRAJECTORY_STATUS_EMPTY
    return message


class Relay:
    LEAD_SECONDS = 1.
    DESCENT = ('DESCENT_MERGE', 'DESCEND', 'DESCENT_ASSEMBLED', 'MOVING_MERGE',
               'TERRAIN', 'FINAL', 'LAND', 'LANDING')

    def __init__(self):
        rospy.init_node('relay_node')
        self.delay = float(rospy.get_param('~delay', DELAY_S))
        if not self.delay > self.LEAD_SECONDS:
            raise ValueError('Relay delay must leave time for slot entry')
        self.lock = threading.RLock()
        self.buffers = {leader: CommandBuffer() for leader in GROUP_LEADERS}
        self.positions = {}
        self.owners = {}
        self.fault = None
        self.hop_state = {}
        self.stage = None
        self.run_id = None
        self.start = None
        self.plans = {}
        self.common_cuts = {}
        self.leaders_topic = {leader: '/drone_%d/pos_cmd' % leader for leader in GROUP_LEADERS}
        self.pubs = {i: rospy.Publisher('/drone_%d/pos_cmd' % i, PositionCommand,
                                        queue_size=1, tcp_nodelay=True)
                     for group in (0, 1) for i in followers(group)}
        self.command = rospy.Publisher('/swarm10/command', String, queue_size=1)
        self.status = rospy.Publisher('/swarm10/relay', String, queue_size=1, latch=True)
        self.leader_samples = {leader: 0 for leader in GROUP_LEADERS}
        for leader in GROUP_LEADERS:
            rospy.Subscriber(self.leaders_topic[leader], PositionCommand, self.leader_cb,
                             callback_args=leader, queue_size=1, tcp_nodelay=True)
        for identity in range(10):
            rospy.Subscriber('/drone_%d/odom' % identity, Odometry,
                             self.odom_cb, callback_args=identity, queue_size=1,
                             tcp_nodelay=True)
            if identity not in GROUP_LEADERS:
                rospy.Subscriber('/drone_%d/mission/status' % identity, String,
                                 self.owner_cb, callback_args=identity, queue_size=1)
        rospy.Subscriber('/swarm10/status', String, self.status_cb, queue_size=1, tcp_nodelay=True)
        self.last_publish = 0.0
        rospy.Timer(rospy.Duration(.02), self.tick)

    def odom_cb(self, msg, identity):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if all(math.isfinite(v) for v in (p.x, p.y, p.z, yaw)):
            with self.lock:
                self.positions[identity] = ([p.x, p.y, p.z], yaw, msg.header.stamp.to_sec())

    def owner_cb(self, msg, identity):
        try:
            state = json.loads(msg.data)
            self.owners[identity] = bool(state.get('relay', {}).get('active'))
        except (TypeError, ValueError, AttributeError):
            return

    def leader_cb(self, msg, leader):
        p, v, a = msg.position, msg.velocity, msg.acceleration
        try:
            with self.lock:
                self.buffers[leader].append(msg.header.stamp.to_sec(),
                                            [p.x, p.y, p.z], [v.x, v.y, v.z],
                                            [a.x, a.y, a.z], msg.yaw, msg.yaw_dot)
                self.leader_samples[leader] += 1
        except ValueError:
            pass

    def status_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if not isinstance(data, dict):
            return
        stage = data.get('stage')
        phases = data.get('group_phases') or []
        with self.lock:
            if stage != self.stage:
                self.stage = stage
                self.run_id = data.get('run_id')
                if stage == 'RUNNING' and self.start is None and data.get('relay_start') is not None:
                    self.start = float(data['relay_start'])
                    self.plans = {g: RelayPlan(self.start, self.delay, hop_duration=self.delay-self.LEAD_SECONDS)
                                  for g, phase in enumerate(phases) if phase != 'IDLE'}
            if self.start is None:
                return
            cuts=data.get('relay_cuts') or []
            for group, plan in self.plans.items():
                if group<len(cuts) and cuts[group] is not None:
                    self.common_cuts[group]=float(cuts[group])
                    continue
                if plan.cut is not None or len(phases) <= group:
                    continue
                phase = phases[group]
                if phase in self.DESCENT:
                    plan.cut = rospy.Time.now().to_sec()

    def leader_command(self, group, stamp):
        return self.buffers[GROUP_LEADERS[group]].at(stamp)

    def fail(self, reason):
        if not self.fault:
            self.fault = reason
            rospy.logerr('Relay %s: %s', self.run_id, reason)
        self.command.publish(String(json.dumps(dict(op='relay_fault', run_id=self.run_id, reason=reason))))

    def follow(self, group, rank, identity, plan, now):
        if now>=self.common_cuts.get(group,math.inf):
            return
        phase = plan.phase(rank, now)
        if phase == 'own':
            return
        source_stamp = plan.sample_time(rank, now) if phase == 'replay' else None
        with self.lock:
            observed = self.positions.get(identity)
            leader = GROUP_LEADERS[group]
            predecessor = leader if rank == 1 else followers(group)[rank-2]
            ahead = self.positions.get(predecessor)
            slot = self.leader_command(group, plan.start)
            sample = self.leader_command(group, source_stamp) if source_stamp is not None else None
        if not self.owners.get(identity):
            if now < plan.hop_start(rank):
                return
            raise ValueError('Member %d did not acknowledge relay ownership' % identity)
        if observed is None or not -.1 <= now-observed[2] <= .35:
            raise ValueError('Member %d odometry missing/stale' % identity)
        state = self.hop_state.get(identity)
        if state is None:
            state = dict(origin=(*observed[0], 0., 0., 0., 0., 0., 0., observed[1], 0.), phase='wait')
            self.hop_state[identity] = state
        if phase == 'wait':
            command = state['origin']
        elif phase == 'hop':
            if slot is None:
                raise ValueError('Missing departure command for group %d' % group)
            if state['phase'] != 'hop':
                if ahead is None or not -.1 <= now-ahead[2] <= .35 or math.dist(ahead[0], slot[:3]) < .3:
                    raise ValueError('Member %d predecessor has not vacated the departure slot' % identity)
                state['entered_hop'] = now
            command = slot_bridge(state['origin'], slot, now-plan.hop_start(rank), plan.hop_duration)
        else:
            if sample is None:
                raise ValueError('Missing delayed leader command for member %d' % identity)
            if state['phase'] != 'replay':
                state['entered_replay'] = now
                state['slot_error'] = math.dist(observed[0], slot[:3])
                if state['slot_error'] > .65:
                    raise ValueError('Member %d missed the departure slot: %.2f m' % (identity, state['slot_error']))
            command = sample
        state['phase'] = phase
        if phase == 'replay':
            state['source_stamp'] = source_stamp
        self.pubs[identity].publish(_command(command[:3], command[3:6], command[6:9], command[9], command[10]))

    def tick(self, _):
        now = rospy.Time.now().to_sec()
        with self.lock:
            stage, plans = self.stage, dict(self.plans)
        if stage == 'RUNNING' and not self.fault:
            try:
                for group, plan in plans.items():
                    for rank, identity in enumerate(followers(group), 1):
                        self.follow(group, rank, identity, plan, now)
            except ValueError as exc:
                self.fail(str(exc))
        if now - self.last_publish >= .2:
            self.last_publish = now
            if stage == 'RUNNING' and self.fault:
                self.fail(self.fault)
            self.publish_status(now)

    def publish_status(self, now):
        members = {}
        with self.lock:
            plans = dict(self.plans)
            start, run_id = self.start, self.run_id
            samples = dict(self.leader_samples)
        for group, plan in plans.items():
            for rank, identity in enumerate(followers(group), 1):
                members[str(identity)] = dict(group=group, rank=rank,
                                              hop_start=plan.hop_start(rank),
                                              replay_start=plan.replay_start(rank),
                                              cut=self.common_cuts.get(group,plan.cut_time(rank)))
                state = self.hop_state.get(identity, {})
                members[str(identity)].update({k: v for k, v in state.items() if k != 'origin'})
        value = dict(stamp=now, run_id=run_id, start=start, delay=self.delay,
                     cut={str(group): None if plan.cut is None else plan.cut
                          for group, plan in plans.items()},
                     leader_samples={str(k): v for k, v in samples.items()},
                     members=members)
        self.status.publish(String(json.dumps(value)))


if __name__ == '__main__':
    Relay()
    rospy.spin()
