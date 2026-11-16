#!/usr/bin/env bash
# User entry: Unity desktop + optional ROS launch. ROS nodes live only in the launch file.
set -eo pipefail
ENTRY_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${YOPO_ENV_FILE:-${YOPO_ROOT:-$ENTRY_DIR/../..}/env.sh}"
MODE="${1:---all}"
if [[ $# -gt 0 ]]; then shift; fi
case "$MODE" in --all|--unity-only|--ros-only|--env) ;; *) echo 'Usage: start_unity.sh [--all|--unity-only|--ros-only|--env] [roslaunch name:=value ...]'; exit 2;; esac

# A missing compiled workspace is an explicit startup error.
for setup in "$YOPO_ROS_SETUP" "$YOPO_CONTROLLER_WS/devel/setup.bash" "$YOPO_SWARM_WS/devel/setup.bash" "$YOPO_TCP_WS/devel/setup.bash" "$YOPO_RVIZ_WS/devel/setup.bash"; do
 [[ -r "$setup" ]] || { echo "Missing dependency: $setup; run ./yopo build" >&2; return 1 2>/dev/null || exit 1; }
done
export YOPO_DRONE_COUNT=10 MC_CAPTURE=0
export YOPO_DEPTH_SOURCE="${YOPO_DEPTH_SOURCE:-unity}"
if [[ "$MODE" == --env ]]; then return 0 2>/dev/null || exit 0; fi
/usr/bin/python3 "$ENTRY_DIR/depth_source.py" --scene-root "$YOPO_SCENE_ROOT" --count 10
MISSION_LAUNCH="$ENTRY_DIR/swarm_mission.launch"
if [[ "${YOPO_POINT_MISSION:-0}" == 1 ]]; then
 export YOPO_CLOUD_MISSION=1
 MISSION_LAUNCH="$ENTRY_DIR/point_swarm_mission.launch"
fi
if [[ ! -f "$YOPO_SCENE_ROOT/Runtime/ray_carve.so" || "$YOPO_SCENE_ROOT/Runtime/ray_carve.cpp" -nt "$YOPO_SCENE_ROOT/Runtime/ray_carve.so" ]]; then
  g++ -O3 -std=c++11 -shared -fPIC "$YOPO_SCENE_ROOT/Runtime/ray_carve.cpp" -o "$YOPO_SCENE_ROOT/Runtime/ray_carve.so.tmp.$$"
  mv -- "$YOPO_SCENE_ROOT/Runtime/ray_carve.so.tmp.$$" "$YOPO_SCENE_ROOT/Runtime/ray_carve.so"
fi
if [[ "$YOPO_DEPTH_SOURCE" == pointcloud ]]; then
 if [[ ! -f "$ENTRY_DIR/cloud_depth_raster_native.so" || "$ENTRY_DIR/cloud_depth_raster.cpp" -nt "$ENTRY_DIR/cloud_depth_raster_native.so" ]]; then
  g++ -O3 -std=c++11 -shared -fPIC "$ENTRY_DIR/cloud_depth_raster.cpp" -o "$ENTRY_DIR/cloud_depth_raster_native.so.tmp.$$"
  mv -- "$ENTRY_DIR/cloud_depth_raster_native.so.tmp.$$" "$ENTRY_DIR/cloud_depth_raster_native.so"
 fi
fi
require_free_ports() {
 local port
 local ports=(11322 10012 10013)
 for port in "${ports[@]}"; do
  if timeout 1 bash -c "</dev/tcp/127.0.0.1/$port" 2>/dev/null; then
   echo "Port $port is occupied; stop the previous instance before starting another." >&2; return 1
  fi
 done
}
if [[ "$MODE" == --ros-only ]]; then
 require_free_ports
 exec roslaunch --port 11322 "$MISSION_LAUNCH" "$@"
fi

mkdir -p "$YOPO_SCENE_ROOT/Runtime" "$MC_REPORTS"
exec 9>"$YOPO_SCENE_ROOT/Runtime/unity-entry.lock"
flock -n 9 || { echo 'This Unity entry is already running.' >&2; exit 1; }
# Do not kill or duplicate a legacy launcher instance.
if pgrep -f "^${YOPO_UNITY_BUILD:-$YOPO_SCENE_ROOT/unity/Build}/MountainCity.x86_64( |$)" >/dev/null; then
 echo 'Unity is already running. Stop the previous simulation before starting a new one.' >&2; exit 1
fi
if [[ "$MODE" == --all ]]; then require_free_ports; fi
if [[ "$YOPO_AUTO_DESKTOP" == 1 ]]; then bash "$YOPO_TOOLS/start_desktop.sh" 9>&-; fi
printf '10' > "$YOPO_SCENE_ROOT/Runtime/drone-count"
unity_pid=''; ros_pid=''
cleanup() {
 trap - EXIT INT TERM
 /usr/bin/python3 "$YOPO_TOOLS/entry_state.py" clear "$$" || true
 # These PIDs are child processes of this shell; no broad kill/killall is used.
 if [[ -n "$ros_pid" ]] && kill -0 "$ros_pid" 2>/dev/null; then
  timeout 3 rostopic pub -1 /swarm10/command std_msgs/String 'data: "{\"op\":\"hold\"}"' >/dev/null 2>&1 || true
  kill -INT "$ros_pid" 2>/dev/null || true
 fi
 if [[ -n "$unity_pid" ]]; then
  kill -TERM "$unity_pid" 2>/dev/null || true
  for attempt in {1..25}; do
   kill -0 "$unity_pid" 2>/dev/null || break
   sleep .2
  done
  # A stalled graphics process may ignore TERM. Check the exact owned binary
  # before forcing its exit so the next user launch is not blocked by it.
  if kill -0 "$unity_pid" 2>/dev/null && [[ "$(readlink -f "/proc/$unity_pid/exe" 2>/dev/null)" == "${YOPO_UNITY_BUILD:-$YOPO_SCENE_ROOT/unity/Build}/MountainCity.x86_64" ]]; then
   kill -KILL "$unity_pid" 2>/dev/null || true
  fi
 fi
 wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
/usr/bin/python3 "$YOPO_TOOLS/entry_state.py" write "$$"
export VK_ICD_FILENAMES="${VK_ICD_FILENAMES:-/etc/vulkan/icd.d/nvidia_icd.json}"
: > "$MC_REPORTS/player-unity.log"
"${YOPO_UNITY_BUILD:-$YOPO_SCENE_ROOT/unity/Build}/MountainCity.x86_64" -force-vulkan -job-worker-count 4 \
 -screen-width "${YOPO_WINDOW_WIDTH:-1440}" -screen-height "${YOPO_WINDOW_HEIGHT:-900}" -screen-fullscreen 0 -logFile "$MC_REPORTS/player-unity.log" 9>&- &
unity_pid=$!
if [[ "$MODE" == --all ]]; then
 roslaunch --port 11322 "$MISSION_LAUNCH" "$@" 9>&- &
 ros_pid=$!
 /usr/bin/python3 "$YOPO_SCENE_ROOT/Runtime/arrange_windows.py" "$unity_pid" "$ros_pid" 9>&- &
 echo 'Unity and ROS starting. Wait for IDLE and fresh sensors before clicking 2D Nav Goal. Ctrl+C closes this simulation.'
 # Bash 5.0 wait -n may wake for the layout helper even when IDs are supplied.
 while kill -0 "$unity_pid" 2>/dev/null && kill -0 "$ros_pid" 2>/dev/null; do sleep 1; done
 echo 'Unity or ROS exited; closing the paired simulation.' >&2
 exit 1
else
 if [[ "$YOPO_DEPTH_SOURCE" == pointcloud && "${YOPO_POINT_MISSION:-0}" == 1 ]]; then
  echo 'Unity started. In another terminal run: bash start_pointcloud.sh --ros-only'
 else
  echo 'Unity started. In another terminal run: bash start_unity.sh --ros-only'
 fi
 wait "$unity_pid"
fi
