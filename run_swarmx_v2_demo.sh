#!/usr/bin/env bash
set -eo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -r /opt/ros/lyrical/setup.bash || ! -r "$workspace/install/setup.bash" ]]; then
  echo "Build the workspace with ROS 2 Lyrical before starting SWARMX V2." >&2
  exit 1
fi
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/swarmx-v2-${UID}.lock"
if ! flock -n 9; then
  echo "SWARMX V2 is already running. Stop its Ubuntu Terminal with Ctrl+C first." >&2
  exit 1
fi
if pgrep -f '[g]z sim' >/dev/null; then
  echo "Gazebo is already running. Stop that simulation before starting V2." >&2
  exit 1
fi
source /opt/ros/lyrical/setup.bash
source "$workspace/install/setup.bash"
exec ros2 launch edge_ai_nav swarmx_v2_demo.launch.py "$@"
