#!/usr/bin/env bash
set -eo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$workspace"
exec 9>/tmp/sih_stage6.lock
flock -n 9 || { echo 'Stage 6 is already running; stop its Ubuntu Terminal launch first.'; exit 1; }
source /opt/ros/lyrical/setup.bash
if [[ -f "${NAV2_WS:-$HOME/nav2_ws}/install/setup.bash" ]]; then
  source "${NAV2_WS:-$HOME/nav2_ws}/install/setup.bash"
fi
source install/setup.bash
exec ros2 launch edge_ai_nav stage6_negotiation.launch.py "$@"
