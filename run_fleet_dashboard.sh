#!/usr/bin/env bash
set -eo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$workspace"
source /opt/ros/lyrical/setup.bash
if [[ -f "${NAV2_WS:-$HOME/nav2_ws}/install/setup.bash" ]]; then
  source "${NAV2_WS:-$HOME/nav2_ws}/install/setup.bash"
fi
source install/setup.bash
export ROS_LOG_DIR=/tmp/sih_fleet_dashboard_logs
mkdir -p "$ROS_LOG_DIR"
exec ros2 run edge_ai_nav fleet_dashboard
