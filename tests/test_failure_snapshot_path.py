import json,tempfile,unittest
from pathlib import Path
from ten_core import failure_directory

class FailureSnapshotPath(unittest.TestCase):
    def test_snapshot_uses_registered_session_report_directory(self):
        with tempfile.TemporaryDirectory() as folder:
            scene=Path(folder);reports=scene/'Reports';target=reports/'mission';target.mkdir(parents=True)
            (reports/'run-index.json').write_text(json.dumps({'run_id':'flight-01','directory':'mission'}))
            try:actual=failure_directory(scene,'flight-01')
            except (OSError,ValueError) as exc:self.fail('Valid session report directory was rejected: '+str(exc))
            self.assertEqual(actual,target)

    def test_old_run_or_outside_directory_cannot_receive_snapshot(self):
        with tempfile.TemporaryDirectory() as folder:
            scene=Path(folder);reports=scene/'Reports';reports.mkdir()
            for value,requested in [({'run_id':'other','directory':'mission'},'flight-01'),
                                    ({'run_id':'flight-01','directory':'../outside'},'flight-01')]:
                (reports/'run-index.json').write_text(json.dumps(value))
                try:
                    failure_directory(scene,requested)
                except ValueError:pass
                except OSError as exc:self.fail('Session index was ignored: '+str(exc))
                else:self.fail('Invalid snapshot destination accepted')

if __name__=='__main__':unittest.main()
