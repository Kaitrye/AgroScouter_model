#!/usr/bin/env bash
set -euo pipefail
robot_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo 'ROS 2 Jazzy не найден в /opt/ros/jazzy.' >&2
  exit 1
fi
set +u
source /opt/ros/jazzy/setup.bash
set -u
bash "$robot_root/tools/build_track_animation.sh"
export GZ_SIM_RESOURCE_PATH="$robot_root/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$robot_root/build/track_animation${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
exec ros2 launch "$robot_root/launch/sim.launch.py" "$@"
