import unittest
from active_layout import ActiveLayout

class ActiveLayoutTests(unittest.TestCase):
    def test_four_modes_preserve_physical_ids(self):
        for mode, ids in [('single',(2,)),('pair',(2,7)),('five',tuple(range(5))),('ten',tuple(range(10)))]:
            layout=ActiveLayout(mode)
            self.assertEqual(layout.active_ids,ids)
            self.assertEqual(layout.scene_count,10)
            self.assertEqual(layout.formation,mode in ('five','ten'))
    def test_invalid_mode_rejected(self):
        for mode in ('',None,1,'three'):
            with self.assertRaises(ValueError):ActiveLayout(mode)
    def test_inactive_status_cannot_complete_missing_active_member(self):
        layout=ActiveLayout('pair')
        self.assertFalse(layout.complete({2:True,0:True},bool))
        self.assertTrue(layout.complete({2:True,7:True,0:False},bool))
        self.assertFalse(layout.complete({2:True,7:False},bool))
    def test_group_members(self):
        self.assertEqual(ActiveLayout('pair').members(1),(7,))
        self.assertEqual(ActiveLayout('five').group_ids,(0,))
        with self.assertRaises(ValueError):ActiveLayout('five').members(1)

if __name__=='__main__':unittest.main()
