#!/usr/bin/env bash
set -euo pipefail
robot_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="$robot_root/build/track_animation"

if ! command -v cmake >/dev/null || ! command -v g++ >/dev/null; then
  echo 'Для анимации гусениц нужны cmake и g++ (sudo apt install cmake g++).' >&2
  exit 1
fi
if [[ ! -f "$build_dir/CMakeCache.txt" ]]; then
  cmake -S "$robot_root/tools/track_animation" -B "$build_dir" \
    -DCMAKE_BUILD_TYPE=Release
fi
cmake --build "$build_dir" -j 2
