import unittest
from audit_smooth_terminal import speed_metrics

class SmoothMetrics(unittest.TestCase):
    def test_constant_flight_beats_repeated_stops_at_same_distance(self):
        smooth=speed_metrics([(i,3.) for i in range(61)])
        cycling=speed_metrics([(i,0. if i%10<5 else 6.) for i in range(61)])
        self.assertEqual(smooth['speed_total_variation_mps'],0.)
        self.assertGreater(cycling['speed_total_variation_mps'],20.)
        self.assertGreater(cycling['slow_entries'],4)
    def test_subthreshold_sensor_noise_is_not_a_stop(self):
        self.assertEqual(speed_metrics([(i,2.+.02*(-1)**i) for i in range(50)])['slow_entries'],0)
if __name__=='__main__':unittest.main()
