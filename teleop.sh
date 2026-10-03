#!/usr/bin/env bash
set -eo pipefail
robot_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
exec python3 "$robot_root/tools/teleop.py" --ros-args -p use_sim_time:=true
