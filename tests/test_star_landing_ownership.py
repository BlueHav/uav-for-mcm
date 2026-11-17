import json
import unittest
from pathlib import Path
import numpy as np
from point_mission_core import PointMission
from point_mission_engine import GroupFlight
from terminal_star_approach import StarApproach


class StarLandingOwnership(unittest.TestCase):
    def setup_flight(self):
        root=Path(__file__).resolve().parents[1]
        model=PointMission(root/'config/point.json');model.independent_terminal=True
        record=json.loads((root/'tests/fixtures/star-final-before-takeover.json').read_text())
        flight=GroupFlight(model,0);flight.phase='TERRAIN'
        p,v=np.asarray(record['p']),np.asarray(record['v'])
        q=model.landing_slots(0);q[:,2]=model.ground(q[:,:2])+model.end_agl
        boundary=[np.asarray(record['reference'][k]) for k in ('position','velocity','acceleration')]
        return flight,p,v,StarApproach(*boundary,q,'ownership-test')

    def test_recorded_settled_star_cannot_bypass_formation_takeover(self):
        flight,p,v,_=self.setup_flight()
        for now in (20.,23.):flight.terminal(p,v,now,False,False)
        self.assertEqual(flight.phase,'TERRAIN')

    def test_partial_group_acknowledgment_cannot_land(self):
        flight,p,v,plan=self.setup_flight()
        flight.star_approach=plan;flight.star_approach_start=10.
        flight.member_states=[dict(formation=dict(star_started=i!=4,star_token=plan.token)) for i in range(5)]
        for now in (20.,23.):flight.terminal(p,v,now,False,False)
        self.assertEqual(flight.phase,'TERRAIN')

    def test_all_members_complete_approach_then_allow_star_landing(self):
        flight,p,v,plan=self.setup_flight()
        flight.star_approach=plan;flight.star_approach_start=10.
        flight.member_states=[dict(formation=dict(star_started=True,star_token=plan.token)) for _ in range(5)]
        for now in (20.,23.):flight.terminal(p,v,now,False,False)
        self.assertEqual(flight.phase,'FINAL')

    def test_matching_ownership_cannot_skip_the_trajectory_duration(self):
        flight,p,v,plan=self.setup_flight()
        flight.star_approach=plan;flight.star_approach_start=10.
        flight.member_states=[dict(formation=dict(star_started=True,star_token=plan.token)) for _ in range(5)]
        for now in (10.1,12.2):flight.terminal(p,v,now,False,False)
        self.assertEqual(flight.phase,'TERRAIN')


if __name__=='__main__':unittest.main()
