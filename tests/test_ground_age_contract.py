import unittest
from planning_mode import input_problem,YOPO

class GroundAgeContract(unittest.TestCase):
    def state(self,age):
        return dict(stamp=10.,odom_age=.02,plan_age=.1,map={'ground_stamp':10.-age},
                    frame_ages={'front':.2,'up':.2,'down':.2},yopo_ready=True)
    def test_inner_gate_matches_existing_point_seven_second_ground_contract(self):
        self.assertIsNone(input_problem(self.state(.65),9.8,10.,YOPO))
    def test_expired_or_future_measurements_still_fail(self):
        self.assertEqual(input_problem(self.state(.71),9.8,10.,YOPO),'local ground stale')
        self.assertEqual(input_problem(self.state(.1),9.29,10.,YOPO),'local ground stale')
        self.assertEqual(input_problem(self.state(.1),10.2,10.,YOPO),'local ground stale')
if __name__=='__main__':unittest.main()
