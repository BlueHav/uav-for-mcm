import unittest
from audit_phase6 import distance_audit_errors


class DistanceBoundsAudit(unittest.TestCase):
    def test_formation_can_keep_every_link_below_trigger_without_distance_cycle(self):
        self.assertEqual(distance_audit_errors({0:{},1:{}},[],{'0:0-3':20.},{'0:0-3':20.}),[])
        self.assertTrue(distance_audit_errors({0:{},1:{}},[],{'0:0-3':20.},{'0:0-3':20.},require_cycle=True))

    def test_missing_cycle_never_hides_a_distance_violation(self):
        self.assertIn('Measured XY link exceeded 50 m',distance_audit_errors({0:{},1:{}},[],{'0:0-3':50.01},{'0:0-3':49.}))
        self.assertIn('20 Hz controller XY link peak exceeded 50 m',distance_audit_errors({0:{},1:{}},[],{'0:0-3':49.},{'0:0-3':50.01}))


if __name__=='__main__':unittest.main()
