import unittest
from tree_distance import member_speed_limit

class MemberSpeedLimit(unittest.TestCase):
    def test_admitted_member_cap_reaches_the_individual_planner(self):
        ref={'speed':6.,'member_speed_caps':[6.,6.,2.1,0.,6.]}
        self.assertEqual(member_speed_limit(ref,2),2.1)
        self.assertEqual(member_speed_limit(ref,3),0.)
        self.assertEqual(member_speed_limit(ref,0),6.)

    def test_legacy_reference_keeps_its_configured_speed(self):
        self.assertEqual(member_speed_limit({'speed':6.},2),6.)

    def test_malformed_caps_are_rejected(self):
        for caps in ([6.]*4,[6.,6.,float('nan'),6.,6.],[6.,6.,-1.,6.,6.],[6.,6.,7.,6.,6.]):
            with self.assertRaises(ValueError):member_speed_limit({'speed':6.,'member_speed_caps':caps},2)

if __name__=='__main__':unittest.main()
