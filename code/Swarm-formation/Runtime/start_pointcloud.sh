#!/usr/bin/env bash
# Independent candidate entry; the image baseline remains selectable separately.
set -eo pipefail
ENTRY_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export YOPO_ROOT="${YOPO_ROOT:-/root/gpufree-data/yopo-portable}"
export YOPO_SCENE_ROOT="$(cd -- "$ENTRY_DIR/.." && pwd)"
export YOPO_POINT_MISSION=1 YOPO_CLOUD_MISSION=1 YOPO_DEPTH_SOURCE=pointcloud
exec bash "$ENTRY_DIR/start_unity.sh" "$@"
