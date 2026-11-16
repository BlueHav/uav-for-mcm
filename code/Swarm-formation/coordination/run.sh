#!/usr/bin/env bash
set -eo pipefail
BASE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export YOPO_ROOT=/root/gpufree-data/yopo-portable
export YOPO_SCENE_ROOT=/root/swarm-uav/code/Swarm-formation
source "$YOPO_ROOT/env.sh"
export ROS_PACKAGE_PATH="$BASE:/root/swarm-uav/code/Swarm-formation/Runtime:$ROS_PACKAGE_PATH"
export PYTHONPATH="$BASE:/root/swarm-uav/code/Swarm-formation/Runtime:/root/swarm-uav/code/YOPO:$PYTHONPATH"
export YOPO_PLANNER_ROOT=/root/swarm-uav/code/YOPO
export ROS_MASTER_URI=http://127.0.0.1:11322 YOPO_TCP_PORT=10012
export YOPO_DRONE_COUNT=2 YOPO_DEPTH_SOURCE=unity YOPO_CLOUD_MISSION=0 YOPO_POINT_MISSION=0 MC_CAPTURE=0
export MC_REPORTS="$BASE" DISPLAY=:20 XAUTHORITY=/home/ubuntu/.Xauthority
mkdir -p "$MC_REPORTS" "$XDG_RUNTIME_DIR"
/usr/bin/python3 "$BASE/prepare.py" > "$BASE/prepare.log"
for port in 11322 10012 10013; do
 if timeout 1 bash -c "</dev/tcp/127.0.0.1/$port" 2>/dev/null; then echo "Occupied port $port"; exit 2; fi
done
for native in ray_carve cloud_depth_raster; do
 if [[ "$native" == ray_carve ]]; then output=ray_carve.so; else output=cloud_depth_raster_native.so; fi
 if [[ ! -f "$YOPO_SCENE_ROOT/Runtime/$output" ]]; then g++ -O3 -std=c++11 -shared -fPIC "$YOPO_SCENE_ROOT/Runtime/$native.cpp" -o "$BASE/$output"; fi
done
case "${YOPO_GRAPHICS_BACKEND:-vulkan}" in vulkan) graphics_flag=-force-vulkan;; glcore) graphics_flag=-force-glcore;; *) echo 'Unsupported graphics backend' >&2; exit 2;; esac
ros_pid='';unity_pid='';observer_pid='';camera_pid=''
cleanup(){
 trap - EXIT INT TERM
 if [[ -n "$ros_pid" ]] && kill -0 "$ros_pid" 2>/dev/null; then kill -INT "$ros_pid" 2>/dev/null || true; fi
 if [[ -n "$unity_pid" ]] && kill -0 "$unity_pid" 2>/dev/null; then
  kill -TERM "$unity_pid" 2>/dev/null || true
  for attempt in {1..25}; do kill -0 "$unity_pid" 2>/dev/null || break; sleep .2; done
  if kill -0 "$unity_pid" 2>/dev/null && [[ "$(readlink -f /proc/$unity_pid/exe 2>/dev/null)" == "$(readlink -f "$BASE/Build/MountainCity.x86_64")" ]]; then kill -KILL "$unity_pid" 2>/dev/null || true; fi
 fi
 if [[ -n "$observer_pid" ]]; then kill -TERM "$observer_pid" 2>/dev/null || true; fi
 if [[ -n "$camera_pid" ]]; then kill -TERM "$camera_pid" 2>/dev/null || true; fi
 wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
: > "$BASE/player-unity.log"
: > "$BASE/pair-audit-0.jsonl"
: > "$BASE/pair-audit-1.jsonl"
roslaunch --port 11322 "$BASE/full.launch" > "$BASE/roslaunch.log" 2>&1 &
ros_pid=$!
sleep 3
"$BASE/Build/MountainCity.x86_64" "$graphics_flag" -job-worker-count 4 -screen-width 1440 -screen-height 900 -screen-fullscreen 0 -logFile "$BASE/player-unity.log" > "$BASE/player-stdout.log" 2>&1 &
unity_pid=$!
printf '{"roslaunch":%s,"unity":%s,"runner":%s}\n' "$ros_pid" "$unity_pid" "$$" > "$BASE/pids.json"
if [[ "${SHARED_SENSOR_PROBE:-0}" == 1 ]]; then /usr/bin/python3 "$BASE/sensor_observer.py" > "$BASE/sensor-observer.log" 2>&1 & observer_pid=$!; fi
if [[ "${SHARED_TEST_MODE:-short}" == full ]]; then /usr/bin/python3 "$BASE/capture_e1.py" "$BASE" > "$BASE/camera-helper.log" 2>&1 & camera_pid=$!; fi
set +e
timeout --signal=INT --kill-after=10 7200 /usr/bin/python3 "$BASE/coordinator.py" > "$BASE/acceptance.log" 2>&1
result=$?
if [[ -n "$observer_pid" ]]; then sleep 2; kill -TERM "$observer_pid" 2>/dev/null || true; fi
printf '%s\n' "$result" > "$BASE/exit-code"
exit "$result"
