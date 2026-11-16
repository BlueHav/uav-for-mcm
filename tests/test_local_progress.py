import unittest
import numpy as np
import mission_core

class LocalProgress(unittest.TestCase):
    def progress(self,*args,**kw):
        fn=getattr(mission_core,'made_local_progress',None)
        self.assertIsNotNone(fn,'Measured independent-flight restart is not represented in watchdog')
        return fn(*args,**kw)

    def test_actual_failure_motion_counts_before_half_metre(self):
        self.assertTrue(self.progress([809.015981,2479.387468,17.779],
            [808.807239,2479.552451,17.795],[-.4923,.3224,.0162],[-1.2,.9,0.],independent=True))

    def test_stationary_jitter_and_uncommanded_drift_are_not_progress(self):
        p=np.zeros(3)
        self.assertFalse(self.progress(p,[.002,0.,0.],[.01,0.,0.],[1.,0.,0.],independent=True))
        self.assertFalse(self.progress(p,[.2,0.,0.],[.3,0.,0.],[0.,0.,0.],independent=True))
        self.assertFalse(self.progress(p,[.2,0.,0.],[.3,0.,0.],[-1.,0.,0.],independent=True))

    def test_legacy_threshold_does_not_change(self):
        self.assertFalse(self.progress([0.,0.,0.],[.2,0.,0.],[.3,0.,0.],[1.,0.,0.]))
        self.assertTrue(self.progress([0.,0.,0.],[.6,0.,0.],[0.,0.,0.],[0.,0.,0.]))

if __name__=='__main__':unittest.main()
