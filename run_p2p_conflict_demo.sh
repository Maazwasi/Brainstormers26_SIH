#!/usr/bin/env bash
set -eo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$workspace"
source /opt/ros/lyrical/setup.bash
source install/setup.bash
set -u
exec ros2 run edge_ai_nav p2p_conflict_demo
