import unittest
from types import SimpleNamespace
import numpy as np
from mission_node import Mission

class ScanDirectionMemory(unittest.TestCase):
    def mission(self):
        m=Mission.__new__(Mission)
        m.five=SimpleNamespace(reference={'independent_terminal':True})
        m.goal=np.array([100.,0.,5.]);m.yaw=0.;m.heading=0.
        m.refv=np.zeros(3);m.refa=np.zeros(3);m.desired=np.zeros(3)
        m.progress_stamp=0.;m.detour_scan_start=None;m.detour_scan_observed=None
        m.detour_scan_anchor=np.array([0.,0.,5.]);m.detour_scan_step=0
        m.detour=None
        return m

    def test_rejected_bypass_moves_to_next_observation_direction(self):
        m=self.mission();p=np.array([0.,0.,5.])
        m.detour=np.array([p,[12.,0.,5.]])
        m.recover_independent_route(p,np.zeros(3),10.)
        self.assertAlmostEqual(m.heading,np.pi/3.)
        self.assertEqual(m.detour_scan_step,1)

    def test_stopped_reentry_retains_scan_direction(self):
        m=self.mission();m.detour_scan_step=2
        m.scan_independent_detour(np.array([0.,0.,5.]),np.zeros(3),10.)
        self.assertAlmostEqual(m.heading,2.*np.pi/3.)

    def test_measured_relocation_starts_new_scan_episode(self):
        m=self.mission();m.detour_scan_step=2
        m.scan_independent_detour(np.array([1.,0.,5.]),np.zeros(3),10.)
        self.assertEqual(m.detour_scan_step,0)

if __name__=='__main__':unittest.main()
