"""Read-only flight telemetry helpers; deliberately independent of ROS/runtime."""
import math


class FlightClock:
    """Observe physical takeoff/landing once per run using the ROS time supplied.

    HOLD and FAILED do not end flight: only contact and disarm for every member
    do. A new run resets the clock, including when the new identifier is empty.
    """

    def __init__(self):
        self.run_id = None
        self.started = None
        self.ended = None
        self.elapsed = 0.

    def update(self, states, now, run_id):
        if run_id != self.run_id:
            self.run_id = run_id
            self.started = None
            self.ended = None
            self.elapsed = 0.
        now = float(now)
        if not math.isfinite(now):
            raise ValueError('Flight clock requires a finite ROS timestamp')
        # Require the actual ten member IDs, not merely any ten dictionary keys.
        members = [states.get(i, states.get(str(i))) for i in range(10)]
        airborne = any(s is not None and s.get('armed') is True and
                       s.get('contact') is False for s in members)
        if self.started is None and airborne:
            self.started = now
        if self.started is not None and self.ended is None:
            # A rewind in ROS time cannot create negative or decreasing duration.
            self.elapsed = max(self.elapsed, now - self.started, 0.)
            landed = all(s is not None and s.get('contact') is True and
                         s.get('armed') is False for s in members)
            if landed:
                self.ended = now
        return dict(flight_started_stamp=self.started, flight_elapsed_s=self.elapsed)


_PHASE_MODES = {
    'IDLE': 'GROUND_ROW',
    'TAKEOFF': 'TAKEOFF_ROW',
    'AIR_ASSEMBLE': 'SQUARE_FORMING',
    'ASSEMBLE': 'SQUARE_FORMING',
    'ASSEMBLED': 'SQUARE',
    'WAIT': 'SQUARE',
    'ALIGN_LIFT': 'SQUARE_TO_LINE',
    'ALIGN_MOVE': 'SQUARE_TO_LINE',
    'ALIGN_SETTLE': 'SQUARE_TO_LINE',
    'TRANSFER_READY': 'SQUARE_TO_LINE',
    'REFORM': 'SQUARE_TO_LINE',
    'CLIMB_READY': 'CLIMB_WAIT',
    'CLIMB': 'CLIMB_WAIT',
    'CRUISE_READY': 'CLIMB_WAIT',
    'LINE': 'LINE',
    'CRUISE': 'LINE',
    'CRUISED': 'LINE',
    'DESCENT_MERGE': 'LINE_TO_STAR',
    'MOVING_MERGE': 'LINE_TO_STAR',
    'STAR_LIFT': 'LINE_TO_STAR',
    'STAR_MOVE': 'LINE_TO_STAR',
    'STAR_SETTLE': 'LINE_TO_STAR',
    'DESCEND': 'STAR',
    'DESCENT_ASSEMBLED': 'STAR',
    'STAR': 'STAR',
    'STAR_HOVER': 'STAR',
    'YOPO_WARMUP': 'STAR',
    'YOPO': 'STAR',
    'TERRAIN': 'ADAPTIVE_STAR',
    'TERRAIN_FOLLOW': 'ADAPTIVE_STAR',
    'FINAL': 'STAR',
    'PRELAND': 'STAR',
    'LANDING': 'STAR',
    'LANDED': 'LANDED',
}


def formation_modes(stage, group_phases=None, planning_mode=None):
    """Describe each group's commanded formation separately from its phase.

    Fleet-wide takeoff, assembly and landing phases supersede engine phases
    which may still describe the previous segment. During RUNNING/HOLD/FAILED,
    each engine remains the source of its own formation state.
    """
    phases = list(group_phases or [])
    fleet_phases = {'IDLE', 'TAKEOFF', 'AIR_ASSEMBLE', 'ASSEMBLE',
                    'ASSEMBLED', 'PRELAND', 'LANDING', 'LANDED'}
    result = []
    for group in range(2):
        phase = stage if stage in fleet_phases else phases[group] if group < len(phases) else None
        mode = _PHASE_MODES.get(phase)
        if mode is None and not phase:
            mode = _PHASE_MODES.get(stage)
            if mode is None and planning_mode in ('YOPO', 'YOPO_WARMUP'):
                mode = 'STAR'
        result.append(mode or 'UNKNOWN')
    return result
