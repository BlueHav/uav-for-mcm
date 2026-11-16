import unittest
from pathlib import Path
import numpy as np
from point_mission_core import PointMission,STAR
from moving_merge import RelayStarMerge

class RelayEntryBoundary(unittest.TestCase):
    def test_existing_cloud_merge_keeps_clearance_before_independent_yopo(self):
        model=PointMission(Path(__file__).resolve().parents[1]/'config/point.json')
        direction=np.array([-.5,-np.sqrt(.75)])
        pairs=np.triu_indices(5,1)
        for group in (0,1):
            xy=model.assembly_anchor(group)-1000.*direction+STAR*8.
            p=np.c_[xy,model.ground(xy)+60.]
            v=np.tile(np.r_[direction*6.5,-.35],(5,1))
            # Real polynomial/admission, no mocked collision or terrain guard.
            plan=RelayStarMerge(model,group,p,v,np.zeros((5,3)))
            q=np.array([plan.sample(t)[0] for t in np.linspace(0.,plan.duration,801)])
            self.assertGreaterEqual(float(np.min(np.linalg.norm(q[:,pairs[0]]-q[:,pairs[1]],axis=2))),2.8)
            expected=np.array([[0.,-3.3],[-3.3,0.],[0.,0.],[3.3,0.],[0.,3.3]])
            np.testing.assert_allclose(q[-1,:,:2]-model.assembly_anchor(group),expected,atol=1e-8)

if __name__=='__main__':unittest.main()
