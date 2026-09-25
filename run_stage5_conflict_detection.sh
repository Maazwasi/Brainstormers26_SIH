#!/usr/bin/env bash
set -eo pipefail
cd /home/maaz-wasi/amr_ws
# Prevent duplicate launch trees even if a previous terminal is still open.
exec 9>/tmp/sih_stage5.lock
flock -n 9 || { echo 'Stage 5 is already running; stop its Ubuntu Terminal launch first.'; exit 1; }
source /opt/ros/lyrical/setup.bash
source /home/maaz-wasi/nav2_ws/install/setup.bash
source install/setup.bash
exec ros2 launch edge_ai_nav stage5_conflict_detection.launch.py "$@"
