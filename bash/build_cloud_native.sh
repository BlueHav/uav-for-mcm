#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$root/code/Swarm-formation/native"
ros=${YOPO_ROS_ROOT:-/opt/ros/noetic}
swarm=${YOPO_SWARM_WS:-/root/gpufree-data/yopo-portable/workspaces/swarm}
boost=${YOPO_BOOST_INCLUDE:-/usr/include}
output=$(mktemp "$PWD/.cloud_swarm_native.XXXXXXXX")
trap 'if [[ -e "$output" ]]; then rm -- "$output"; fi' EXIT
g++ -O3 -DNDEBUG -std=c++14 -Iadapter/include -Iupstream/traj_opt/include \
    -Iupstream/path_searching/include -Iupstream/traj_utils/include \
    -Iupstream/plan_env/include -Iupstream/swarm_graph/include \
    -I"$swarm/devel/include" -I"$boost" -I/usr/include/eigen3 -I"$ros/include" \
    adapter/cloud_swarm_native.cpp upstream/traj_opt/src/poly_traj_optimizer.cpp \
    upstream/path_searching/src/dyn_a_star.cpp upstream/swarm_graph/src/swarm_graph.cpp \
    -L"$ros/lib" -Wl,-rpath-link,"$ros/lib" -lroscpp -lrosconsole -lrostime -lcpp_common -pthread \
    -o "$output"
chmod 755 "$output"
mv -f -- "$output" cloud_swarm_native
