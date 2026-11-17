"""Machine configuration shared by mission modules and diagnostic tools."""
import os
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[3]
ROOT = Path(os.environ.get('YOPO_ROOT', PROJECT))
SCENE = Path(os.environ.get('YOPO_SCENE_ROOT', ROOT / 'scene'))
PLANNER = Path(os.environ.get('YOPO_PLANNER_ROOT', PROJECT / 'code/YOPO'))
WEIGHTS = Path(os.environ.get('YOPO_WEIGHTS', PLANNER / 'saved/YOPO_1/epoch50.pth'))
OPS = Path(os.environ.get('YOPO_OPS_ROOT', ROOT / 'ops'))
CONTROLLER = Path(os.environ.get('YOPO_CONTROLLER_WS', ROOT / 'workspaces/controller'))
SWARM = Path(os.environ.get('YOPO_SWARM_WS', ROOT / 'workspaces/swarm'))
TCP = Path(os.environ.get('YOPO_TCP_WS', ROOT / 'workspaces/tcp'))
RVIZ = Path(os.environ.get('YOPO_RVIZ_WS', ROOT / 'workspaces/rviz'))
PYTHON = Path(os.environ.get('YOPO_PYTHON', ROOT / '.venv/bin/python'))
TOOLS = ROOT / 'tools'
DESKTOP_SCRIPT = TOOLS / 'start_desktop.sh'
UNITY_EDITOR = Path(os.environ.get('YOPO_UNITY_EDITOR', '/opt/unity/Editor/Unity'))
