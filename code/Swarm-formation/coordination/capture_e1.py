# Camera-only evidence helper. Never publishes flight/arm/task commands.
import json,os,re,subprocess,sys,time
from pathlib import Path
import rospy
from std_msgs.msg import String
B=Path(sys.argv[1]);rospy.init_node('e1_camera_evidence',anonymous=True);state={};E=[931.17,3241.94];setup=False;last=0.;log=(B/'camera-evidence.jsonl').open('w',buffering=1)
def callback(m):
 global state
 state=json.loads(m.data)
rospy.Subscriber('/drone_0/mission/status',String,callback,queue_size=1);pub=rospy.Publisher('/scene/view_command',String,queue_size=1)
pids=json.loads((B/'pids.json').read_text());end=time.monotonic()+7200
while not rospy.is_shutdown() and time.monotonic()<end and Path('/proc/%d'%pids['runner']).exists():
 if state.get('phase') in ('TERRAIN_FOLLOW','END_HOLD') and sum((state['position'][i]-E[i])**2 for i in (0,1))<220**2 and time.monotonic()-last>8:
  last=time.monotonic()
  try:
   windows=subprocess.check_output(['wmctrl','-lp'],text=True);window=next(l.split()[0] for l in windows.splitlines() if l.split()[2]==str(pids['unity']))
   if not setup:
    subprocess.run(['wmctrl','-ia',window],check=False);pub.publish(String('follow'));time.sleep(.4);subprocess.run(['xdotool','mousemove','--window',window,'720','450','click','--repeat','8','--delay','80','5'],check=False);setup=True;time.sleep(.3)
   info=subprocess.check_output(['xwininfo','-root'],text=True);fields={k:int(re.search(r'%s:\s+(-?\d+)'%k,info).group(1)) for k in ['Width','Height']};path=B/'screenshots'/('E1-live-%.3f.png'%time.time());display=os.environ['DISPLAY']+'+0,0'
   subprocess.run(['ffmpeg','-y','-hide_banner','-loglevel','error','-f','x11grab','-video_size','%dx%d'%(fields['Width'],fields['Height']),'-i',display,'-frames:v','1',str(path)],capture_output=True,timeout=8,check=True);log.write(json.dumps(dict(stamp=time.time(),path=str(path),state=state))+'\n')
   if state.get('phase')=='END_HOLD':break
  except Exception as exc:log.write(json.dumps(dict(stamp=time.time(),error=repr(exc)))+'\n')
 time.sleep(.5)
log.close()
