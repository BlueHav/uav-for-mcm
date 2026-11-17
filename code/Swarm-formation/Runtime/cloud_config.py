"""User-approved high cruise; terminal YOPO parameters remain separate."""
CRUISE_SPEED=20.
CLIMB_SPEED_LIMIT=16.
STAR_ENTRY_SPEED=4. # Retain stopping room before opening the relay's lateral lanes.
# Phase 3 requires sustained measured horizontal speed above 15 m/s.
CRUISE_MINIMUM=15.
CRUISE_SECONDS=60.
CRUISE_SPACING=15.
# Phase 2 chained relay: each follower replays the leader 3 s later than the
# previous one (3/6/9/12 s from the leader).
RELAY_DELAY=3.
CONTEXT_SPAN=400. # 24 m/s delayed stop plus 0.6 s source-pose age needs >181 m ahead.
CONTEXT_RESOLUTION=2.
NATIVE_HORIZON=80.
NATIVE_ACCELERATION=2.

def configure_model(model):
    model.cruise_speed=CRUISE_SPEED;model.spacing=CRUISE_SPACING
    model.relay=True;model.replay_delay=RELAY_DELAY
    model.high_speed_cloud=True
    model.group_delay=0.;model.synchronized_climb=True;model.member_departure_interval=5.
    model.forward_departure=True;model.descending_merge=True
    model.moving_merge=True

def reference_speed(ref):
    if ref.get('planner_mode')=='CLOUD' and ref.get('stage')=='DESCEND' and ref.get('moving_merge') is True:
        from moving_merge import CATCHUP_SPEED
        return CATCHUP_SPEED
    if ref.get('planner_mode')=='CLOUD' and ref.get('stage')=='CLIMB' and ref.get('relay_climb') is True:
        return CRUISE_SPEED
    return CRUISE_SPEED if ref.get('planner_mode')=='CLOUD' and ref.get('stage')=='CRUISE' else 6.
