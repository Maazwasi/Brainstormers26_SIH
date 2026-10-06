#!/usr/bin/env bash
# Run from Ubuntu Terminal alongside the V2 fleet with use_rviz:=true.
set -eo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/lyrical/setup.bash
source "$workspace/install/setup.bash"
exec ros2 launch edge_ai_nav swarmx_v2_slam_view.launch.py "$@"
