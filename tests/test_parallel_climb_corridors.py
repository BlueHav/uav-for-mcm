import unittest
from pathlib import Path
import numpy as np
from point_mission_core import PointMission
from point_mission_engine import GroupFlight
from cloud_config import configure_model


class ParallelClimbCorridors(unittest.TestCase):
    def test_assembled_leaders_keep_lateral_order_at_independent_progress(self):
        model=PointMission(Path(__file__).resolve().parents[1]/'config/point.json')
        configure_model(model)
        starts=[];axes=[]
        for group in range(2):
            flight=GroupFlight(model,group)
            p=model.assembly(group);v=np.zeros_like(p)
            flight.relay_climb(p,v,10.,False,False)
            starts.append(p[2,:2]);axes.append(flight.climb_axis)
        # Both climb/cruise lanes must remain on their assembly side for
        # asynchronous progress; endpoint-bearing rays used to intersect.
        initial=float((starts[1]-starts[0])@model.normal)
        self.assertGreater(initial,10.)
        for progress0 in np.linspace(0,4150,31):
            for progress1 in np.linspace(0,4150,31):
                separation=(starts[1]+progress1*axes[1]-starts[0]-progress0*axes[0])@model.normal
                self.assertGreaterEqual(separation,initial-1e-8)


if __name__=='__main__':unittest.main()
