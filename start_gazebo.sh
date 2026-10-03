#!/usr/bin/env bash
set -euo pipefail
robot_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
fi
bash "$robot_root/tools/build_track_animation.sh"
export GZ_SIM_RESOURCE_PATH="$robot_root/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$robot_root/build/track_animation${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
exec gz sim -r -v 3 "$robot_root/worlds/test_world.sdf" "$@"
