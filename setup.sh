#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$ROOT/env.sh"
test -x "$YOPO_PYTHON" || { echo "Missing YOPO Python: $YOPO_PYTHON"; exit 2; }
command -v roslaunch >/dev/null
(cd "$ROOT/assets"; sha256sum -c SHA256SUMS)
mkdir -p "$ROOT/unity"
cat "$ROOT"/assets/unity.tar.gz.part* | tar xzf - -C "$ROOT/unity"
chmod +x "$ROOT/unity/Build/MountainCity.x86_64"
find "$ROOT/code/Swarm-formation/Runtime" -name '*.py' -exec chmod +x {} +
chmod +x "$ROOT/code/Swarm-formation/Runtime/quadrotor_simulator_elapsed" "$ROOT/code/Swarm-formation/native/"*native
bash "$ROOT/bash/unity.sh" --check
bash "$ROOT/bash/control.sh" ten --check
echo 'Setup OK. Run bash bash/unity.sh then bash bash/control.sh ten in a second terminal.'
