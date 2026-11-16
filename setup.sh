#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$ROOT/env.sh"
test -x "$YOPO_PYTHON" || { echo 'Set YOPO_PYTHON to a Python 3.8 environment with code/YOPO/requirements.txt installed.' >&2; exit 2; }
"$YOPO_PYTHON" -c 'import torch, numpy, scipy, cv2, rospy; print("Inference Python:", __import__("sys").executable)'
(cd "$ROOT/assets"; sha256sum -c SHA256SUMS)
mkdir -p "$ROOT/unity"
cat "$ROOT"/assets/unity.tar.gz.part* | tar xzf - -C "$ROOT/unity"
# Apply the verified script rebuild to this branch's bundled base Player.
cp "$ROOT/assets/unity-managed/Assembly-CSharp.dll" "$ROOT/unity/Build/MountainCity_Data/Managed/Assembly-CSharp.dll"
cp "$ROOT/assets/unity-managed/build.json" "$ROOT/unity/Build/managed-script-build.json"
chmod +x "$ROOT/unity/Build/MountainCity.x86_64"
mkdir -p "$YOPO_CONTROLLER_WS/src" "$YOPO_TCP_WS/src"
for package in "$ROOT"/code/controller-src/*/package.xml "$ROOT"/code/controller-src/utils/*/package.xml; do
  [[ -f "$package" ]] || continue
  directory=$(dirname "$package")
  ln -sfn "$directory" "$YOPO_CONTROLLER_WS/src/$(basename "$directory")"
done
ln -sfn "$ROOT/code/tcp-src/ROS-TCP-Endpoint" "$YOPO_TCP_WS/src/ROS-TCP-Endpoint"
(cd "$YOPO_CONTROLLER_WS"; catkin_make -j"${COMBINE_BUILD_JOBS:-4}" -DPYTHON_EXECUTABLE=/usr/bin/python3)
(cd "$YOPO_TCP_WS"; catkin_make -j"${COMBINE_BUILD_JOBS:-4}" -DPYTHON_EXECUTABLE=/usr/bin/python3)
source "$ROOT/env.sh"
RUNTIME="$ROOT/code/Swarm-formation/Runtime"
g++ -O3 -std=c++11 -shared -fPIC "$RUNTIME/ray_carve.cpp" -o "$RUNTIME/ray_carve.so"
g++ -O3 -std=c++11 -shared -fPIC "$RUNTIME/cloud_depth_raster.cpp" -o "$RUNTIME/cloud_depth_raster_native.so"
find "$ROOT/code/Swarm-formation/Runtime" -name '*.py' -exec chmod +x {} +
chmod +x "$ROOT/code/Swarm-formation/Runtime/quadrotor_simulator_elapsed" "$ROOT/code/Swarm-formation/native/"*native
command -v patchelf >/dev/null || { echo 'Install patchelf to remove legacy binary RPATHs.' >&2; exit 2; }
patchelf --remove-rpath "$RUNTIME/quadrotor_simulator_elapsed"
bash "$ROOT/bash/unity.sh" --check
bash "$ROOT/bash/control.sh" ten --check
echo 'Setup OK. Run bash bash/unity.sh then bash bash/control.sh ten in a second terminal.'
