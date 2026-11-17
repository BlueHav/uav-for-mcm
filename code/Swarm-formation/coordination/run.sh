#!/usr/bin/env bash
# Legacy pair-control entry. Start this checkout's Unity in a separate terminal.
set -eo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)
exec bash "$ROOT/bash/control.sh" pair "$@"
