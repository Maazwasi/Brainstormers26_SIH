#!/usr/bin/env bash
# EXPERIMENTAL profile. Do not present as accepted before physical gates pass.
set -eo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if pgrep -f '[g]z sim' >/dev/null; then
  echo "Stop the existing Gazebo before starting the enlarged variant." >&2
  exit 1
fi
source /opt/ros/lyrical/setup.bash
source "$workspace/install/setup.bash"
bridge_config="$workspace/install/edge_ai_nav/share/edge_ai_nav/config/five_amr_bridge_large.yaml"
if [ ! -r "$bridge_config" ]; then
  echo "Missing installed Gazebo bridge config: $bridge_config" >&2
  echo "Run: cd $workspace && colcon build --symlink-install --packages-select amr_simulation edge_ai_nav" >&2
  exit 1
fi
# The lock-owning wrapper retains its FD, but --close prevents Gazebo helper
# children from inheriting it and holding a stale lock after ROS has stopped.
exec flock --nonblock --close "${XDG_RUNTIME_DIR:-/tmp}/swarmx-v2-${UID}.lock" \
  ros2 launch edge_ai_nav swarmx_v2_large.launch.py "$@"
