#!/usr/bin/env python3
"""Map the Unity window and tile it with this launch's RViz, including hidden windows."""
import json,os,subprocess,sys,time
from pathlib import Path
unity,launch=map(int,sys.argv[1:3]);deadline=time.monotonic()+70
def run(*args):return subprocess.check_output(args,text=True,stderr=subprocess.DEVNULL).strip()
def window(pid,title):
 try:return run('xdotool','search','--all','--pid',str(pid),'--name',title).split()[0]
 except (subprocess.CalledProcessError,IndexError):return None
while time.monotonic()<deadline:
 if not Path('/proc/%d'%unity).exists() or not Path('/proc/%d'%launch).exists():break
 rviz=None
 try:
  for pid in run('pgrep','-P',str(launch)).split():
   if Path('/proc/'+pid+'/comm').read_text().strip()=='rviz':rviz=int(pid);break
 except (OSError,subprocess.CalledProcessError):pass
 a=window(unity,'GeoMap10km');b=window(rviz,'RViz') if rviz else None
 if a and b:
  width,height=map(int,run('xdotool','getdisplaygeometry').split());half=width//2
  for wid,x in [(a,0),(b,half)]:
   subprocess.run(['xdotool','windowmap',wid],check=True)
   subprocess.run(['wmctrl','-ir',hex(int(wid)),'-b','remove,maximized_vert,maximized_horz,fullscreen'],check=True)
   subprocess.run(['wmctrl','-ir',hex(int(wid)),'-e','0,%d,0,%d,%d'%(x,half-8,height-65)],check=True)
   subprocess.run(['xdotool','windowraise',wid],check=True)
  report=dict(unity_window=a,rviz_window=b,display=os.environ.get('DISPLAY'),layout='side_by_side')
  (Path(os.environ['YOPO_SCENE_ROOT'])/'Reports/window-layout.json').write_text(json.dumps(report,indent=2))
  print('Unity and RViz visible side by side',flush=True);break
 time.sleep(.5)
else:print('Window layout timed out; inspect Unity/RViz startup logs',flush=True)
