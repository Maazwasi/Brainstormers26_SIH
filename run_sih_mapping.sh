#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/lyrical/setup.bash
[[ -f /home/maaz-wasi/nav2_ws/install/setup.bash ]] && source /home/maaz-wasi/nav2_ws/install/setup.bash
cd "$WORKSPACE_DIR"
colcon build --symlink-install --packages-select amr_simulation edge_ai_nav
source "$WORKSPACE_DIR/install/setup.bash"
echo "MAPPING MODE: ALPHA LiDAR -> SLAM Toolbox occupancy map (no fleet coordinator)"
exec ros2 launch edge_ai_nav mapping_demo.launch.py "$@"
