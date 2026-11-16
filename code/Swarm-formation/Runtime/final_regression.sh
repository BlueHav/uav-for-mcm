#!/usr/bin/env bash
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)/env.sh"
set -eo pipefail
OPS=${YOPO_OPS_ROOT}
ROOT=${YOPO_SCENE_ROOT}
source "$YOPO_ROOT/env.sh"
export ROS_MASTER_URI=http://127.0.0.1:11322 ROS_IP=127.0.0.1
wait_ready() {
 for attempt in $(seq 1 30); do
  if bash "$OPS/bin/status.sh" --ready > "$ROOT/Reports/regression-ready.json" 2>&1; then return; fi
  sleep 1
 done
 return 1
}
echo 'REGRESSION SINGLE'
bash "$OPS/bin/stop.sh"
bash "$OPS/bin/start.sh" 1
wait_ready
/usr/bin/python3 "$ROOT/Runtime/verify_commands.py"
cp "$ROOT/Reports/command-verification.json" "$ROOT/Reports/single-regression-after-five.json"
echo 'REGRESSION TWO'
bash "$OPS/bin/check_short_flight.sh"
cp "$(cat "$OPS/config/last-run")/module-smoke.json" "$ROOT/Reports/two-regression-after-five.json"
echo 'REGRESSION FIVE FINAL BUILD'
bash "$OPS/bin/check_five.sh"
echo 'ALL THREE MODE REGRESSIONS PASSED'
