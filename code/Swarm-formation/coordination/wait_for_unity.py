import os,sys,time
from pathlib import Path
log=Path(__file__).parent/'player-unity.log'
end=time.monotonic()+120
while time.monotonic()<end:
    if log.exists() and 'TRACKER_FIELD_READY drones=2' in log.read_text(errors='replace'):
        time.sleep(6)
        os.execv(sys.argv[1],sys.argv[1:])
    time.sleep(.25)
raise SystemExit('Two-drone Unity did not become ready within 120 s')
