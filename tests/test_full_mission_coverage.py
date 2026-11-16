import copy,unittest
from audit_full_mission import full_coverage_problems


class FullMissionCoverage(unittest.TestCase):
    def report(self):
        stages=['TAKEOFF','AIR_ASSEMBLE','ASSEMBLED','RUNNING','PRELAND','LANDING','LANDED']
        phases=['WAIT','CLIMB','CRUISE','MOVING_MERGE','DESCENT_ASSEMBLED','TERRAIN','FINAL']
        return dict(active_ids=list(range(10)),stages=[dict(stage=s) for s in stages],
            group_events=[[dict(phase=p) for p in phases] for _ in range(2)],
            cruise_evidence=dict(passed=True,checked_members=list(range(10)),required_straight_seconds=60.,
                minimum_horizontal_mps=15.,members={str(i):dict(passed=True,best_straight_seconds=70.) for i in range(10)}))

    def test_full_ordered_history_with_all_ten_cruise_members_is_accepted(self):
        self.assertEqual(full_coverage_problems(self.report()),[])

    def test_successful_phase_six_fixture_is_not_full_mission_evidence(self):
        r=self.report();r['stages']=r['stages'][3:];r['group_events']=[[dict(phase='TERRAIN'),dict(phase='FINAL')]]*2
        errors=full_coverage_problems(r)
        self.assertTrue(any('stage' in e.lower() for e in errors))
        self.assertTrue(any('group' in e.lower() for e in errors))

    def test_aggregate_pass_cannot_hide_a_missing_or_unqualified_cruise_member(self):
        for bad in ('missing','duration'):
            r=copy.deepcopy(self.report())
            if bad=='missing':del r['cruise_evidence']['members']['9']
            else:r['cruise_evidence']['members']['9']['best_straight_seconds']=59.9
            self.assertTrue(full_coverage_problems(r))


if __name__=='__main__':unittest.main()
