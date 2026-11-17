#!/usr/bin/env python3
"""ROS launch prefix: avoid connecting planners during Unity publisher re-registration."""
import portable_paths
import os,sys,time
from pathlib import Path
log=Path(os.environ.get('YOPO_SCENE_ROOT',(str(portable_paths.SCENE))))/'Reports/player-unity.log'
deadline=time.monotonic()+120
while time.monotonic()<deadline:
 if log.exists() and 'TRACKER_FIELD_READY' in log.read_text(errors='replace'):
  # Unity registers its cached topic set once more on TCP connection establishment.
  time.sleep(6)
  os.execv(sys.argv[1],sys.argv[1:])
 time.sleep(.25)
raise SystemExit('Unity did not become ready within120 s')
