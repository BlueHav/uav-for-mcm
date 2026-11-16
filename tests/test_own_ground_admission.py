import unittest
from types import SimpleNamespace
from mission_node import Mission
from sensor_inbox import LatestMeasurement


class OwnGroundAdmission(unittest.TestCase):
    def mission(self,mode):
        m=Mission.__new__(Mission);m.independent_terminal=True;m.planning_mode=mode
        m.ground_cloud=LatestMeasurement()
        return m

    def test_independent_member_keeps_own_crop_when_newer_leader_crop_arrives(self):
        m=self.mission('YOPO');own=SimpleNamespace(data=b'own')
        m.on_ground_cloud(own,own=True)
        m.on_ground_cloud(SimpleNamespace(data=b'far-away leader'),own=False)
        self.assertIs(m.ground_cloud.take(),own)

    def test_cloud_cruise_still_accepts_group_crop(self):
        m=self.mission('CLOUD');group=SimpleNamespace(data=b'group')
        m.on_ground_cloud(group,own=False)
        self.assertIs(m.ground_cloud.take(),group)

    def test_warmup_also_requires_own_domain(self):
        m=self.mission('YOPO_WARMUP')
        m.on_ground_cloud(SimpleNamespace(data=b'group'),own=False)
        self.assertIsNone(m.ground_cloud.take())

if __name__=='__main__':unittest.main()
