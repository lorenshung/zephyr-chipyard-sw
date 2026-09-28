#!/usr/bin/env bash
# Bootstrap source dependencies only. Install west/SDK separately.
set -euo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
git -C "$repo" submodule update --init --recursive modelblaster zephyr_ws/zephyr samples/drone_control/tinympc
if [[ ! -f "$repo/zephyr_ws/.west/config" ]]; then
  west init -l "$repo/zephyr_ws/manifest"
fi
cd "$repo/zephyr_ws"
west config manifest.path manifest
west config manifest.file west.yml
west update "$@"
expected=$(git -C "$repo" rev-parse HEAD:zephyr_ws/zephyr)
actual=$(git -C "$repo/zephyr_ws/zephyr" rev-parse HEAD)
[[ "$expected" == "$actual" ]] || { echo "RTOS manifest/gitlink mismatch" >&2; exit 1; }
