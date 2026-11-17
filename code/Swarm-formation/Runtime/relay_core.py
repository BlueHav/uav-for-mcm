"""Timing and command interpolation for delayed group leader replay."""
DELAY_S = 3.0
GROUP_SIZE = 5
GROUP_LEADERS = (2, 7)
# Physical order: leader first, then followers by rank.
GROUP_ORDER = ((2, 0, 1, 3, 4), (7, 5, 6, 8, 9))
MAX_SPAN_S = 30.0
MAX_GAP_S = 0.5


def validated_control_reference(reference,now):
    """A complete, fresh P/V/A sample actually published to a controller."""
    import math
    if not isinstance(reference,dict):raise ValueError('Missing actual controller command')
    try:
        stamp=float(reference['stamp']);current=float(now)
        values={key:[float(x) for x in reference[key]] for key in ('position','velocity','acceleration')}
    except (KeyError,TypeError,ValueError,OverflowError):raise ValueError('Invalid actual controller command')
    if (not math.isfinite(stamp) or not math.isfinite(current) or not -.1<=current-stamp<=.3
            or any(len(v)!=3 or not all(math.isfinite(x) for x in v) for v in values.values())):
        raise ValueError('Expired/future/nonfinite actual controller command')
    return dict(stamp=stamp,**values)


def group_of(drone_id):
    if isinstance(drone_id, bool) or not isinstance(drone_id, int) or not 0 <= drone_id < 10:
        raise ValueError('Drone id outside 0..9')
    return drone_id // GROUP_SIZE


def leader_of(group):
    if isinstance(group, bool) or group not in (0, 1):
        raise ValueError('Group must be 0 or 1')
    return GROUP_LEADERS[group]


def followers(group):
    if isinstance(group, bool) or group not in (0, 1):
        raise ValueError('Group must be 0 or 1')
    return GROUP_ORDER[group][1:]


def rank_of(drone_id):
    """1..4 for a follower; 0 for the leader."""
    return GROUP_ORDER[group_of(drone_id)].index(drone_id)


def command(array):
    """Validate one command sample: p(3), v(3), a(3), yaw, yaw_dot = 11."""
    import math
    values = [float(v) for v in array]
    if len(values) != 11 or not all(math.isfinite(v) for v in values):
        raise ValueError('Command needs eleven finite values')
    return tuple(values)


class CommandBuffer:
    """Bounded, monotonic 50 Hz leader command history."""

    def __init__(self, max_span=MAX_SPAN_S, max_gap=MAX_GAP_S):
        if not max_span > 0 or not max_gap > 0:
            raise ValueError('Invalid buffer bounds')
        self.max_span = float(max_span)
        self.max_gap = float(max_gap)
        self.items = []

    def append(self, stamp, position, velocity, acceleration, yaw, yaw_dot):
        import math
        stamp = float(stamp)
        if not math.isfinite(stamp):
            raise ValueError('Non-finite command stamp')
        values = command(list(position) + list(velocity) + list(acceleration) + [yaw, yaw_dot])
        if self.items and stamp <= self.items[-1][0]:
            raise ValueError('Command stream is not chronological')
        self.items.append((stamp,) + values)
        cutoff = stamp - self.max_span
        while self.items and self.items[0][0] < cutoff:
            self.items.pop(0)

    def at(self, stamp):
        """Latest command at or before stamp, or None if absent/too old."""
        import math
        stamp = float(stamp)
        if not math.isfinite(stamp) or not self.items or stamp < self.items[0][0]:
            return None
        found = None
        for item in self.items:
            if item[0] <= stamp:
                found = item
            else:
                break
        if found is None or stamp - found[0] > self.max_gap:
            return None
        return found[1:]

    def latest_stamp(self):
        return self.items[-1][0] if self.items else None


class RelayPlan:
    """Timing for one group's four chained followers."""

    def __init__(self, start, delay=DELAY_S, cut=None, hop_duration=None):
        import math
        start = float(start)
        delay = float(delay)
        # Default: the whole delay window is the flight to the vacated slot.
        hop = float(delay if hop_duration is None else hop_duration)
        if not math.isfinite(start) or not delay > 0 or not 0 < hop <= delay:
            raise ValueError('Invalid relay start/delay/hop')
        if cut is not None:
            cut = float(cut)
            if not math.isfinite(cut) or cut < start:
                raise ValueError('Invalid relay cut')
        self.start = start
        self.delay = delay
        self.hop_duration = hop
        self.cut = cut

    def hop_start(self, rank):
        # The follower flies to the leader's vacated slot over the full delay
        # window; the next follower starts when this one begins its replay.
        self._rank(rank)
        return self.replay_start(rank) - self.hop_duration

    def replay_start(self, rank):
        self._rank(rank)
        return self.start + self.delay * rank

    def cut_time(self, rank):
        self._rank(rank)
        return None if self.cut is None else self.cut + self.delay * rank

    def phase(self, rank, now):
        self._rank(rank)
        now = float(now)
        if now < self.hop_start(rank):
            return 'wait'
        if now < self.replay_start(rank):
            return 'hop'
        if self.cut is not None and now >= self.cut_time(rank):
            return 'own'
        return 'replay'

    def sample_time(self, rank, now):
        """Leader stream time this follower replays at `now`."""
        self._rank(rank)
        return float(now) - self.delay * rank

    @staticmethod
    def _rank(rank):
        if isinstance(rank, bool) or not isinstance(rank, int) or not 1 <= rank <= 4:
            raise ValueError('Follower rank must be 1..4')


def slot_bridge(origin, target, elapsed, duration):
    """Join two commands with continuous position, velocity and acceleration."""
    import math

    u = min(1., max(0., elapsed / duration))
    position, velocity, acceleration = [], [], []
    for axis in range(4):
        index = axis if axis < 3 else 9
        velocity_index = axis + 3 if axis < 3 else 10
        p0, v0 = origin[index], origin[velocity_index] * duration
        p1, v1 = target[index], target[velocity_index] * duration
        if axis == 3:
            p1 = p0 + math.remainder(p1 - p0, 2. * math.pi)
        a0 = origin[axis + 6] * duration**2 / 2. if axis < 3 else 0.
        a1 = target[axis + 6] * duration**2 if axis < 3 else 0.
        dp, dv, da = p1 - p0 - v0 - a0, v1 - v0 - 2. * a0, a1 - 2. * a0
        c3, c4, c5 = 10.*dp - 4.*dv + .5*da, -15.*dp + 7.*dv - da, 6.*dp - 3.*dv + .5*da
        position.append(p0 + u*(v0 + u*(a0 + u*(c3 + u*(c4 + u*c5)))))
        velocity.append((v0 + u*(2.*a0 + u*(3.*c3 + u*(4.*c4 + u*5.*c5)))) / duration)
        acceleration.append((2.*a0 + u*(6.*c3 + u*(12.*c4 + u*20.*c5))) / duration**2)
    return (*position[:3], *velocity[:3], *acceleration[:3], position[3], velocity[3])
