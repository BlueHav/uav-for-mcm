#!/usr/bin/env bash
# System ROS/Python are dependencies; project paths belong to this checkout.
export UAV_PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export YOPO_ROOT="$UAV_PROJECT_ROOT"
export YOPO_PLANNER_ROOT="$UAV_PROJECT_ROOT/code/YOPO"
export YOPO_CONTROLLER_WS="$UAV_PROJECT_ROOT/workspaces/controller"
export YOPO_TCP_WS="$UAV_PROJECT_ROOT/workspaces/tcp"
export YOPO_SWARM_WS="$UAV_PROJECT_ROOT/workspaces/controller"
export YOPO_RVIZ_WS="$UAV_PROJECT_ROOT/workspaces/controller"
export YOPO_PYTHON="${YOPO_PYTHON:-$UAV_PROJECT_ROOT/.venv/bin/python}"
export YOPO_ROS_SETUP="${YOPO_ROS_SETUP:-/opt/ros/noetic/setup.bash}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11322}" ROS_IP=127.0.0.1
_combine_nounset=0; [[ $- == *u* ]] && _combine_nounset=1
set +u
source "$YOPO_ROS_SETUP"
for _combine_ws in "$YOPO_CONTROLLER_WS" "$YOPO_TCP_WS"; do
  [[ ! -r "$_combine_ws/devel/setup.bash" ]] || source "$_combine_ws/devel/setup.bash" --extend
done
export YOPO_POINT_FILE="${YOPO_POINT_FILE:-$UAV_PROJECT_ROOT/config/point.json}"
export YOPO_SCENE_ROOT="${SWARM_RUN_DIR:-$UAV_PROJECT_ROOT/.runs/session}"
export YOPO_UNITY_BUILD="$UAV_PROJECT_ROOT/unity/Build"
export YOPO_ENV_FILE="$UAV_PROJECT_ROOT/env.sh"
export YOPO_WEIGHTS="$YOPO_PLANNER_ROOT/saved/YOPO_1/epoch50.pth"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11322}" ROS_IP=127.0.0.1
export PYTHONPATH="$UAV_PROJECT_ROOT/code/Swarm-formation/Runtime:$YOPO_PLANNER_ROOT:$UAV_PROJECT_ROOT/code/tcp-src/ROS-TCP-Endpoint/src:${PYTHONPATH:-}"
export ROS_PACKAGE_PATH="$UAV_PROJECT_ROOT/code/Swarm-formation/Runtime:$UAV_PROJECT_ROOT/code/tcp-src/ROS-TCP-Endpoint:${ROS_PACKAGE_PATH:-}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/combine-runtime-$(id -u)}"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 OMP_WAIT_POLICY=PASSIVE KMP_BLOCKTIME=0 DISABLE_ROS1_EOL_WARNINGS=1
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu/blas:/usr/lib/x86_64-linux-gnu/lapack:${LD_LIBRARY_PATH:-}"
(( _combine_nounset == 0 )) || set -u
unset _combine_ws _combine_nounset
