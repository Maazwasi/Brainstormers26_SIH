#!/usr/bin/env bash
set -eo pipefail
cd /home/maaz-wasi/amr_ws
source /opt/ros/lyrical/setup.bash
source /home/maaz-wasi/nav2_ws/install/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/sih_fleet_dashboard_logs
mkdir -p "$ROS_LOG_DIR"
exec ros2 run edge_ai_nav fleet_dashboard
