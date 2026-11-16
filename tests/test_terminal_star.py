import unittest
from pathlib import Path
import numpy as np
from point_mission_core import PointMission

class TerminalStar(unittest.TestCase):
    def test_landing_preserves_leading_upstream_star(self):
        model=PointMission(Path(__file__).resolve().parents[1]/'config/point.json')
        # Literal first five type-6 points, scaled by two, with leader at ID 2.
        expected=np.array([[-6.,-3.],[6.,-3.],[0.,0.],[-3.,-6.],[3.,-6.]])
        for group in (0,1):
            forward=model.ends[group]-model.assembly_anchor(group)
            forward/=np.linalg.norm(forward)
            right=np.array([forward[1],-forward[0]])
            offsets=model.landing_slots(group)[:,:2]-model.ends[group]
            local=np.c_[offsets@right,offsets@forward]
            np.testing.assert_allclose(local,expected,atol=1e-8)

    def test_ground_acceptance_rejects_square_touchdown(self):
        from mission_core import BODY_OFFSET
        model=PointMission(Path(__file__).resolve().parents[1]/'config/point.json')
        states={}
        for g in (0,1):
            for i,p in enumerate(model.landing_slots(g)):
                states[5*g+i]=dict(phase='LANDED',contact=True,armed=False,position=p,velocity=[0.,0.,0.])
        self.assertTrue(model.landed(states))
        square=np.array([[-3.3,-3.3],[3.3,-3.3],[0.,0.],[-3.3,3.3],[3.3,3.3]])
        for g in (0,1):
            xy=model.ends[g]+square
            for i,q in enumerate(xy):states[5*g+i]['position']=np.r_[q,model.ground(q[None,:])[0]+BODY_OFFSET]
        self.assertFalse(model.landed(states))

if __name__=='__main__':unittest.main()
