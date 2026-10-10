#!/usr/bin/env bash
# Persistent SWARMX V2 web control service. Simulation children are launched
# in a separate process group and never own this dashboard process.
set -eo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$workspace"
source /opt/ros/lyrical/setup.bash
source "$workspace/install/setup.bash"
export ROS_LOG_DIR=/tmp/swarmx_control_dashboard_logs
mkdir -p "$ROS_LOG_DIR"
config="$workspace/install/amr_simulation/share/amr_simulation/config/warehouse_sih_v2_large.yaml"
if [[ ! -r "$config" ]]; then
  echo "Missing installed V2 configuration. Build the workspace first." >&2
  exit 1
fi
exec ros2 run edge_ai_nav fleet_dashboard --ros-args \
  -p port:=8091 \
  -p config_file:="$config"
