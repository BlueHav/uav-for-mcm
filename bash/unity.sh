#!/usr/bin/env bash
# Start the fixed ten-aircraft scene; does not issue takeoff commands.
set -eo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ${1:-} == --help ]]; then echo 'bash bash/unity.sh [--check]; optional SWARM_RUN_DIR=/root/swarm-uav-session'; exit 0; fi
RUN=${SWARM_RUN_DIR:-$ROOT/.runs/session}
export YOPO_SCENE_ROOT="$RUN" YOPO_PLANNER_ROOT="$ROOT/code/YOPO"
source "$ROOT/env.sh"
export YOPO_DRONE_COUNT=10 YOPO_DEPTH_SOURCE=unity YOPO_TCP_PORT=10012 MC_CAPTURE=0
export YOPO_DEPTH_HZ="${YOPO_DEPTH_HZ:-15}"
# This flag only selects the player's depth schedule. control.sh owns ROS
# planning mode. Keep all ten front streams available to the existing selector.
export YOPO_CLOUD_MISSION=0
export DISPLAY=${SWARM_DISPLAY:-:20} XAUTHORITY=${SWARM_XAUTHORITY:-/home/ubuntu/.Xauthority}
export YOPO_POINT_FILE="${YOPO_POINT_FILE:-$ROOT/config/point.json}"
BUILD=${YOPO_UNITY_BUILD:-$ROOT/unity/Build}
[[ -x "$BUILD/MountainCity.x86_64" ]] || { echo 'Unity player missing' >&2; exit 2; }
case ${YOPO_GRAPHICS_BACKEND:-glcore} in glcore) flag=-force-glcore;; vulkan) flag=-force-vulkan;; *) exit 2;; esac
if [[ ${1:-} == --check ]]; then echo "Unity player: $BUILD; requested count=10 (actual scene must be verified)"; exit 0; fi
[[ $# == 0 ]] || exit 2
mkdir -p "$RUN/Reports" "$XDG_RUNTIME_DIR"
exec 9>"$RUN/unity.lock"
flock -n 9 || { echo 'Scene already running in this session' >&2; exit 2; }
if [[ -e "$RUN/Reports/player-unity.log" ]]; then echo 'Existing evidence: choose a new SWARM_RUN_DIR for both terminals' >&2; exit 2; fi
[[ -e "$RUN/Runtime" ]] || ln -s "$ROOT/code/Swarm-formation/Runtime" "$RUN/Runtime"
export MC_REPORTS="$RUN/Reports"
echo "Session: $RUN; ten-aircraft scene only; no takeoff requested"
exec "$BUILD/MountainCity.x86_64" "$flag" -job-worker-count 4 -screen-width 1440 -screen-height 900 -screen-fullscreen 0 -logFile "$MC_REPORTS/player-unity.log"
