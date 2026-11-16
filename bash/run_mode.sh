#!/usr/bin/env bash
# Orchestrate one point-mission run: Unity scene, control stack, command driver.
set -uo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
MODE=${1:-ten}
RUN=${2:-$ROOT/.runs/${MODE}-$(date +%Y%m%d-%H%M%S)}
mkdir -p "$RUN"
mkdir -p "$ROOT/.runs"
echo "$RUN" > "$ROOT/.runs/last-${MODE}-run"
echo "RUN=$RUN MODE=$MODE" | tee "$RUN/orchestrator.log"

export SWARM_RUN_DIR="$RUN"
export YOPO_DEPTH_HZ="${YOPO_DEPTH_HZ:-5}"
# The Unity player needs the same mission flags as the ROS stack: the point
# mission selects the leader-only cloud-cruise depth profile and the bounded
# terrain display; the legacy single/pair builds keep the full legacy scene.
case "$MODE" in
  five|ten) export YOPO_POINT_MISSION=1 YOPO_CLOUD_MISSION=1 ;;
  *) export YOPO_POINT_MISSION=0 YOPO_CLOUD_MISSION=0 ;;
esac
cd "$ROOT"
nohup bash bash/unity.sh > "$RUN/unity.out" 2>&1 &
echo "unity pid $!" | tee -a "$RUN/orchestrator.log"

for i in $(seq 1 180); do
  if grep -q 'TRACKER_FIELD_READY drones=10' "$RUN/Reports/player-unity.log" 2>/dev/null; then
    echo "unity ready after ${i}s" | tee -a "$RUN/orchestrator.log"; break
  fi
  sleep 1
done
if ! grep -q 'TRACKER_FIELD_READY drones=10' "$RUN/Reports/player-unity.log" 2>/dev/null; then
  echo 'unity never became ready' | tee -a "$RUN/orchestrator.log"; exit 2
fi

nohup bash bash/control.sh "$MODE" > "$RUN/control.out" 2>&1 &
echo "control pid $!" | tee -a "$RUN/orchestrator.log"

source "$ROOT/env.sh"
export ROS_MASTER_URI=http://127.0.0.1:11322
export PYTHONPATH="$ROOT/code/Swarm-formation/Runtime:$ROOT/code/YOPO:${PYTHONPATH:-}"

sleep 20
/usr/bin/python3 "$ROOT/tests/run_point_mission.py" "$MODE" 3900 "$RUN/driver.json" > "$RUN/driver.log" 2>&1
result=$?
echo "driver exit $result" | tee -a "$RUN/orchestrator.log"
echo "RUN_DIR=$RUN"
exit "$result"
