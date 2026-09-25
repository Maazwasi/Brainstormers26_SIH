#!/usr/bin/env bash
set -eo pipefail
cd /home/maaz-wasi/amr_ws
exec 9>/tmp/sih_stage6.lock
flock -n 9 || { echo 'Stage 6 is already running; stop its Ubuntu Terminal launch first.'; exit 1; }
source /opt/ros/lyrical/setup.bash
source /home/maaz-wasi/nav2_ws/install/setup.bash
source install/setup.bash
exec ros2 launch edge_ai_nav stage6_negotiation.launch.py "$@"
