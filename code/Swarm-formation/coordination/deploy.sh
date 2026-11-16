#!/usr/bin/env bash
set -euo pipefail
SOURCE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
MODE=${1:-short}
OUT=${2:-/root/migration-20261108/parallel-shared-state/$MODE-1}
if [[ -e "$OUT/acceptance-report.json" ]]; then echo 'Existing acceptance evidence; choose a fresh output directory.' >&2;exit 2;fi
mkdir -p "$OUT/perception" "$OUT/screenshots"
cp "$SOURCE"/*.py "$SOURCE/package.xml" "$SOURCE/run.sh" "$OUT/"
cp "$SOURCE/../analysis/audit_parallel_descent.py" "$OUT/"
cp /root/migration-20261108/parallel-shared-state/preflight/geometry-preflight.json "$OUT/geometry-preflight.json"
ln -s /root/migration-20261107/full-pair-mission/Build "$OUT/Build"
export SHARED_TEST_MODE="$MODE"
exec bash "$OUT/run.sh"
