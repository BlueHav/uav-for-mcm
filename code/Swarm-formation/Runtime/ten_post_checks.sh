#!/usr/bin/env bash
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)/env.sh"
set -eo pipefail
source "$YOPO_ROOT/env.sh"
export ROS_MASTER_URI=http://127.0.0.1:11322 ROS_IP=127.0.0.1
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu/blas:/usr/lib/x86_64-linux-gnu/lapack:${LD_LIBRARY_PATH:-}
cd ${YOPO_SCENE_ROOT}/Runtime
ops=${YOPO_OPS_ROOT}
/usr/bin/python3 verify_mode_start.py 10
/usr/bin/python3 verify_ten_timeout.py
for count in 1 2 5; do
 bash "$ops/bin/stop.sh"
 bash "$ops/bin/start.sh" "$count"
 /usr/bin/python3 verify_mode_start.py "$count"
done
bash "$ops/bin/stop.sh"
bash "$ops/bin/start_ten.sh"
/usr/bin/python3 verify_ten_modules.py
bash "$ops/bin/show.sh" unity
echo 'TEN POST CHECKS COMPLETE'
