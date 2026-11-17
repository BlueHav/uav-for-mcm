import time,unittest
from types import SimpleNamespace
from pathlib import Path
import numpy as np
from local_map import RollingLocalMap
from mission_node import Mission

class ScanMarginEscape(unittest.TestCase):
    def stopped_mission(self,grid,p,goal,height):
        m=Mission.__new__(Mission)
        m.map=grid;m.goal=np.asarray(goal,float);m.height=height
        m.count=10;m.id=2;m.p=np.asarray(p,float).copy();m.speed=6.;m.mode='AGL';m.phase='TERRAIN_FOLLOW';m.yaw=0.;m.heading=0.
        m.refv=np.zeros(3);m.refa=np.zeros(3);m.desired=np.zeros(3)
        m.five=SimpleNamespace(reference={'independent_terminal':True,'speed':6.})
        m.progress_stamp=time.monotonic()-5.;m.detour_scan_start=time.monotonic()-1.
        m.detour_scan_step=0;m.detour=None;m.buffer_escape_active=False
        return m

    def test_active_scan_does_not_hide_safe_retreat_from_new_margin_overlap(self):
        grid=RollingLocalMap(size=(12.,12.,10.),resolution=.5)
        p=np.array([.18,.25,2.25]);grid.update_pose(p)
        grid.state[:]=grid.FREE;grid.ground[:]=-2.75
        cell=np.array([1,0,4])-grid.origin
        grid.state[tuple(cell)]=grid.OCCUPIED
        before=grid.state.copy()
        self.assertTrue(grid.points_clear([p],radius=.30)[0])
        self.assertFalse(grid.points_clear([p],radius=.35)[0])
        m=self.stopped_mission(grid,p,[4.,4.,2.25],5.)
        m.horizontal_plan(p,np.zeros(3))
        self.assertTrue(m.buffer_escape_active,'Active yaw scan starved the measured safe body-margin retreat')
        self.assertGreater(np.linalg.norm(m.desired),0.)
        self.assertLessEqual(np.linalg.norm(m.desired),.300001)
        self.assertTrue(np.all(grid.escape_points_clear(m.chosen,p)))
        np.testing.assert_array_equal(grid.state,before)

    def test_archived_failure_observation_keeps_body_safe_retreat_available(self):
        with np.load(Path(__file__).parent/'fixtures/historical-margin-overlap.npz') as data:
            grid=RollingLocalMap(size=np.array(data['state'].shape)*.5,resolution=.5)
            grid.state=data['state'].copy();grid.origin=data['origin'].copy()
            grid.ground=data['ground'].copy();p=data['position'].copy();grid.position=p.copy()
            goal=data['goal'].copy()
        self.assertTrue(grid.points_clear([p],radius=.30)[0])
        self.assertFalse(grid.points_clear([p],radius=.35)[0])
        before=grid.state.copy()
        m=self.stopped_mission(grid,p,goal,float(p[2]-grid.ground_at(*p[:2])))
        m.horizontal_plan(p,np.zeros(3))
        self.assertTrue(m.buffer_escape_active)
        self.assertGreater(np.linalg.norm(m.desired),0.)
        self.assertTrue(np.all(grid.escape_points_clear(m.chosen,p)))
        np.testing.assert_array_equal(grid.state,before)

if __name__=='__main__':unittest.main()
