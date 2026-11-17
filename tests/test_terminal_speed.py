import unittest
import numpy as np
from terminal_speed import bounded_speed, select_continuous_velocity, stable_candidates
from terminal_stopping import actual_stop_paths


class TerminalSpeed(unittest.TestCase):
    def test_recovery_does_not_follow_alternating_caps(self):
        value=2.
        values=[]
        for cap in [6.,2.]*20:
            value=bounded_speed(value,cap,.1);values.append(value)
        self.assertLessEqual(max(values),2.061)
        self.assertGreaterEqual(min(values),2.)

    def test_smaller_safety_limit_is_immediate(self):
        self.assertEqual(bounded_speed(6.,1.5,.1),1.5)
        self.assertEqual(bounded_speed(6.,0.,.1),0.)
        with self.assertRaises(ValueError):bounded_speed(2.,float('nan'),.1)

    def test_reduced_command_preserves_heading_and_real_stop_clearance(self):
        p=np.zeros(3);v=np.array([2.,0.,0.]);a=np.zeros(3);desired=np.array([6.,0.,0.])
        lower=max(path[:,0].max() for path in actual_stop_paths(p,v,a,v))
        upper=max(path[:,0].max() for path in actual_stop_paths(p,v,a,desired))
        self.assertGreater(upper,lower)
        wall=(lower+upper)/2
        def clear(command):
            return all(np.all(path[:,0]<wall) for path in actual_stop_paths(p,v,a,command))
        result=select_continuous_velocity(desired,clear)
        self.assertIsNotNone(result)
        self.assertGreater(result[0],0)
        self.assertLess(result[0],6)
        self.assertTrue(clear(result))
        np.testing.assert_array_equal(result[1:],desired[1:])

    def test_no_clear_command_keeps_stop_authority(self):
        self.assertIsNone(select_continuous_velocity(np.ones(3),lambda _:False))

    def test_small_score_gain_does_not_switch_a_valid_heading(self):
        straight=(2.,np.array([1.,0.]))
        turn=(2.1,np.array([.8,.6]))
        self.assertIs(stable_candidates([turn,straight],np.array([1.,0.]))[0],straight)
        decisive=(3.,np.array([.8,.6]))
        self.assertIs(stable_candidates([decisive,straight],np.array([1.,0.]))[0],decisive)

if __name__=='__main__':unittest.main()
