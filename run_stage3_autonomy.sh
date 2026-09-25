#!/usr/bin/env bash
set -eo pipefail
cd /home/maaz-wasi/amr_ws
source /opt/ros/lyrical/setup.bash
source /home/maaz-wasi/nav2_ws/install/setup.bash
source install/setup.bash
exec ros2 launch edge_ai_nav stage3_autonomy.launch.py "$@"
