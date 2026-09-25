#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/lyrical/setup.bash"
NAV2_SETUP="/home/maaz-wasi/nav2_ws/install/setup.bash"
MODE="${1:-decentralized}"
shift || true

case "$MODE" in
  decentralized|centralized) ;;
  mapping)
    exec "$WORKSPACE_DIR/run_sih_mapping.sh" "$@"
    ;;
  *)
    echo "Usage: $0 {decentralized|centralized|mapping} [launch arguments]" >&2
    exit 2
    ;;
esac

source "$ROS_SETUP"
[[ -f "$NAV2_SETUP" ]] && source "$NAV2_SETUP"
cd "$WORKSPACE_DIR"

echo "Building SIH five-AMR MVP..."
colcon build --symlink-install --packages-select amr_simulation edge_ai_nav
source "$WORKSPACE_DIR/install/setup.bash"

echo "Launching mode=$MODE"
if [[ "$MODE" == "decentralized" ]]; then
  echo "DECENTRALIZED: NO CENTRAL COORDINATOR REQUIRED"
else
  echo "CENTRALIZED BASELINE: delay parameters are explicitly SIMULATED"
fi
exec ros2 launch edge_ai_nav five_amr_demo.launch.py mode:="$MODE" "$@"
