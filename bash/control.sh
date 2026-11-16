#!/usr/bin/env bash
# Start mission control separately; takeoff remains a ROS command.
set -eo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ${1:-} == --help ]]; then echo 'bash bash/control.sh [single|pair|five|ten] [--check]; same SWARM_RUN_DIR as unity.sh'; exit 0; fi
MODE=${1:-ten}; [[ $# == 0 ]] || shift
CHECK=${1:-}; [[ $# -le 1 && ( -z "$CHECK" || "$CHECK" == --check ) ]] || exit 2
RUN=${SWARM_RUN_DIR:-$ROOT/.runs/session}
export YOPO_SCENE_ROOT="$RUN" YOPO_PLANNER_ROOT="$ROOT/code/YOPO"
source "$ROOT/env.sh"
export YOPO_DRONE_COUNT=10 YOPO_DEPTH_SOURCE=unity YOPO_POINT_MISSION=1 YOPO_CLOUD_MISSION=1 YOPO_GROUP_CLOUD=1 YOPO_GROUP_CONTEXT=1 YOPO_RELAY=1
case "$MODE" in five) export YOPO_ACTIVE_IDS=0,1,2,3,4 YOPO_INPROCESS_EXECUTOR=1;; ten) export YOPO_ACTIVE_IDS=0,1,2,3,4,5,6,7,8,9 YOPO_INPROCESS_EXECUTOR=1;; single) export YOPO_ACTIVE_IDS=2;; pair) export YOPO_ACTIVE_IDS=2,7;; esac
export ROS_MASTER_URI=http://127.0.0.1:11322 YOPO_TCP_PORT=10012 MC_REPORTS="$RUN/Reports"
export DISPLAY=${SWARM_DISPLAY:-:20} XAUTHORITY=${SWARM_XAUTHORITY:-/home/ubuntu/.Xauthority}
RUNTIME="$ROOT/code/Swarm-formation/Runtime"
export ROS_PACKAGE_PATH="$RUNTIME:$ROS_PACKAGE_PATH" PYTHONPATH="$RUNTIME:$ROOT/code/YOPO:$PYTHONPATH"
/usr/bin/python3 "$RUNTIME/active_layout.py" --mode "$MODE" --require-runtime
[[ -x "$YOPO_PYTHON" && -f "$RUNTIME/ray_carve.so" ]] || { echo 'Missing runtime Python/native dependency' >&2; exit 2; }
[[ "$CHECK" == --check ]] && { echo 'Dependency/mode check only; flight acceptance is still required'; exit 0; }
[[ -f "$RUN/Reports/player-unity.log" ]] || { echo 'Start unity.sh first with the same SWARM_RUN_DIR' >&2; exit 2; }
grep -q 'TRACKER_FIELD_READY drones=10' "$RUN/Reports/player-unity.log" || { echo 'Actual ten-aircraft Unity readiness has not been observed' >&2; exit 2; }
for port in 11322 10012 10013; do
 if timeout 1 bash -c "</dev/tcp/127.0.0.1/$port" 2>/dev/null; then echo "Port $port already occupied" >&2; exit 2; fi
done
echo 'Starting control in IDLE. No takeoff command is sent by this script.'
if [[ "$MODE" == pair ]]; then
 export YOPO_CLOUD_MISSION=0 YOPO_POINT_MISSION=0
 /usr/bin/python3 "$RUNTIME/prepare_active_pair.py" --point "$ROOT/config/point.json" --output "$RUN/active-pair"
 export ROS_PACKAGE_PATH="$RUN/active-pair:$ROS_PACKAGE_PATH" PYTHONPATH="$RUN/active-pair:$PYTHONPATH"
 exec roslaunch --port 11322 "$RUN/active-pair/pair.launch"
fi
if [[ "$MODE" == single ]]; then
 export YOPO_CLOUD_MISSION=0 YOPO_POINT_MISSION=0
 /usr/bin/python3 "$RUNTIME/prepare_active_single.py" --point "$ROOT/config/point.json" --output "$RUN/active-single"
 export ROS_PACKAGE_PATH="$RUN/active-single:$ROS_PACKAGE_PATH" PYTHONPATH="$RUN/active-single:$PYTHONPATH"
 exec roslaunch --port 11322 "$RUN/active-single/single.launch"
fi
exec roslaunch --port 11322 "$RUNTIME/point_swarm_mission.launch" mode:="$MODE" point:="$ROOT/config/point.json"
