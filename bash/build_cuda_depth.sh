#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
runtime="$root/code/Swarm-formation/Runtime"
plugins="$root/unity/Project/Assets/Plugins/x86_64"
nvcc=${YOPO_NVCC:-nvcc}
architecture=${YOPO_CUDA_ARCH:-sm_89}
temporary=
trap '[[ -z "$temporary" ]] || rm -f -- "$temporary"' EXIT

build() {
    local source=$1 output=$2
    temporary=$(mktemp "${output}.XXXXXX")
    "$nvcc" -O2 "-arch=$architecture" --fmad=false -Xcompiler -fPIC -shared "$source" -o "$temporary"
    chmod 755 "$temporary"
    mv -f -- "$temporary" "$output"
    temporary=
}

mkdir -p "$plugins"
build "$runtime/ray_carve_cuda.cu" "$runtime/ray_carve_cuda.so"
build "$root/unity/native/cuda_depth_ipc.cu" "$plugins/libyopo_cuda_depth_ipc.so"
player_plugins="$root/unity/Build/MountainCity_Data/Plugins/x86_64"
[[ -d "$player_plugins" ]] || { echo 'Run setup.sh to unpack the Player first' >&2; exit 2; }
cp "$plugins/libyopo_cuda_depth_ipc.so" "$player_plugins/libyopo_cuda_depth_ipc.so"
