"""Time-stamped trajectory execution with local, rolling brake checks."""
import numpy as np

from terminal_client import TerminalClient
from terminal_corridor import contains_stops
from terminal_environment import observe
from terminal_formation import environment_clear
from terminal_stopping import actual_stop_paths
from terminal_stream_client import ContinuousTerminalClient, prepare_packet, serialized


class DirectTerminalClient(ContinuousTerminalClient):
    direct_execution = True

    def __init__(self, mission, clock=None):
        super().__init__(mission, clock)
        self.independent_environment = True
        self.dispatched = None

    @serialized
    def receive(self, command, now):
        if not isinstance(command, dict) or self.m.failure:
            return
        if command.get('kind') == 'abort':
            self.abort('Joint coordinator cancelled trajectory')
            return
        slot = command
        if command.get('kind') == 'stream':
            slot = command.get('next') or command.get('active')
        if not isinstance(slot, dict) or slot.get('kind') != 'commit':
            return
        packet = slot.get('packet', {})
        if not self.ownership(packet) or packet.get('serial', 0) <= self.last_serial:
            return
        start = slot.get('start')
        if not isinstance(start, (int, float)) or not np.isfinite(start):
            return
        if self.dispatched is not None and self.dispatched['packet']['digest'] == packet['digest']:
            return
        if slot.get('predecessor') != (None if self.packet is None else self.packet['digest']):
            return
        if self.dispatched is not None:
            return
        self.dispatched = dict(packet=packet, start=start,
                               prepared=self.pool.submit(prepare_packet, packet, self.m.id % 5))
        self.received = now

    @serialized
    def capture_observation(self, now):
        pending = self.dispatched
        box = self.active_box
        if pending is not None:
            incoming = np.asarray(pending['packet']['envelopes'][self.m.id % 5])
            box = incoming if box is None else np.vstack((box, incoming))
        if box is None:
            return
        self.box = box
        observe(self, now)

    def promote(self, now):
        pending = self.dispatched
        if self.m.failure or pending is None or now < pending['start']:
            return
        if not pending['prepared'].done() or now > pending['start'] + .15:
            self.abort('Scheduled trajectory missed its local execution deadline')
            return
        packet, start = pending['packet'], pending['start']
        try:
            plan, box, velocity_limit = pending['prepared'].result()
        except (ValueError, KeyError, TypeError) as exc:
            self.abort('Invalid scheduled trajectory: ' + str(exc))
            return
        if self.packet is None:
            p, v, a = plan.sample(0.)
            if (not self.stationary() or max(np.linalg.norm(self.m.p-p), np.linalg.norm(self.m.refp-p)) >= .10
                    or np.linalg.norm(v) >= .035 or np.linalg.norm(a) >= .06):
                self.abort('Initial trajectory differs from measured hold')
                return
        else:
            t = min(self.plan.duration, start-self.start)
            if t <= 0. or np.max(np.linalg.norm(np.asarray(self.plan.sample(t))-plan.sample(0.), axis=1)) > 1e-5:
                self.abort('Scheduled trajectory is not continuous in position, velocity and acceleration')
                return
            if t == self.plan.duration and not self.done:
                self.abort('Scheduled post-tail trajectory precedes measured settlement')
                return
            self.previous_tracking = (self.plan, self.start, self.velocity_limit)
            self.switches += 1
        self.packet, self.plan, self.start = packet, plan, start
        self.box = self.active_box = box
        self.velocity_limit = velocity_limit
        self.last_serial = packet['serial']
        self.native_packets += 1
        self.accepted = self.committed = True
        self.done = False
        self.dispatched = None
        self.stop_inside = False

    @serialized
    def monitor(self, now):
        if self.clock is not None:
            now = float(self.clock())
        self.promote(now)
        if self.committed and (not self.stop_inside or now-self.stop_checked >= .04):
            self.check_braking(now)

    def check_braking(self, now):
        m = self.m
        observation = self.map_observation
        if (observation is None or observation.grid is None or not self.fresh_observation(now)
                or not all(-.05 <= now-stamp <= .6 for stamp in (observation.sensor_stamp, observation.checked))):
            self.abort('Live braking map or sensors expired')
            return False
        with m.lock:
            p, v, stamp = m.p.copy(), m.v.copy(), m.odom_stamp
            prior = getattr(m, 'last_control_reference', None)
        if not -.05 <= now-stamp <= .5:
            self.abort('Odometry expired before live braking check')
            return False
        if prior is not None and not -.05 <= now-prior['stamp'] <= .30:
            self.abort('Command gap exceeds modeled braking delay')
            return False
        target = self.plan.sample(min(self.plan.duration, max(0., now-self.start)))
        references = [(target[1], target[2])]
        if prior is not None:
            references.append((np.asarray(prior['velocity']), np.asarray(prior['acceleration'])))
        corridor = getattr(self.plan, 'stop_corridor', None)
        for velocity, acceleration in references:
            paths = actual_stop_paths(p, v, acceleration, velocity)
            if corridor is None or not contains_stops(corridor, paths):
                self.abort('Measured actual stopping path exceeds reserved envelope')
                return False
            if not all(environment_clear(observation.grid, path) for path in paths):
                self.abort('Live braking path overlaps occupied or unknown space')
                return False
        finished = now if self.clock is None else float(self.clock())
        if not -.05 <= finished-now <= .15:
            self.abort('Live braking check expired during calculation')
            return False
        self.stop_inside, self.stop_stamp, self.stop_checked = True, stamp, now
        return True

    @serialized
    def reference(self, now):
        if self.clock is not None:
            now = float(self.clock())
        self.promote(now)
        if not self.committed:
            return None
        # Recompute before publishing if the previous calculation has aged
        # out. The former command's measured braking path is checked as well.
        if not self.stop_inside or now-self.stop_checked >= .04:
            if not self.check_braking(now):
                return None
        value = TerminalClient.reference(self, now)
        if value is not None:
            self.local_output_digest = self.packet['digest']
        return value

    @serialized
    def local_execution_status(self, now):
        value = super().local_execution_status(now)
        value['direct_terminal'] = True
        value['active_start'] = self.start if value['active'] else None
        value['yopo_reference_members'] = [] if not value['active'] else self.packet.get('yopo_reference_members', [])
        return value
