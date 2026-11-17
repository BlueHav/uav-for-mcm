#!/usr/bin/env bash
# Compatibility entry; all supported startup paths belong to this checkout.
set -eo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)
MODE=${1:---all}
[[ $# == 0 ]] || shift
[[ $# == 0 ]] || { echo 'Use bash/control.sh for launch mode options' >&2; exit 2; }
case "$MODE" in
 --env) source "$ROOT/env.sh" ;;
 --unity-only) exec bash "$ROOT/bash/unity.sh" ;;
 --ros-only) exec bash "$ROOT/bash/control.sh" ten ;;
 --all) echo 'Start bash bash/unity.sh, then bash bash/control.sh ten in a second terminal (same SWARM_RUN_DIR).' >&2; exit 2 ;;
 *) echo 'Usage: start_unity.sh [--all|--unity-only|--ros-only|--env]' >&2; exit 2 ;;
esac
