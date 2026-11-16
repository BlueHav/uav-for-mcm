import os
import subprocess
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]

class IndependentDeployment(unittest.TestCase):
    def test_env_uses_only_checkout_workspaces(self):
        env={'HOME':str(Path.home()),'PATH':'/usr/bin:/bin','YOPO_PYTHON':os.environ.get('YOPO_PYTHON','/usr/bin/python3')}
        command='source ./env.sh; printf "%s\\n" "$YOPO_ROOT" "$YOPO_CONTROLLER_WS" "$YOPO_TCP_WS" "$ROS_PACKAGE_PATH" "$ROS_MASTER_URI"'
        result=subprocess.run(['bash','-c',command],cwd=ROOT,env=env,text=True,capture_output=True,check=True)
        paths=result.stdout.strip().splitlines()
        self.assertEqual(paths[0],str(ROOT))
        self.assertEqual(paths[1],str(ROOT/'workspaces/controller'))
        self.assertEqual(paths[2],str(ROOT/'workspaces/tcp'))
        self.assertEqual(paths[-1],'http://127.0.0.1:11322')
        self.assertNotIn('gpufree-data/yopo-portable',result.stdout)
        self.assertNotIn('/root/swarm-uav/',result.stdout)

if __name__=='__main__':unittest.main()
