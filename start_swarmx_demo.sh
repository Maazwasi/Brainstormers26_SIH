#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! command -v x-terminal-emulator >/dev/null 2>&1; then
  echo "x-terminal-emulator was not found. Use the manual commands in README.md."
  exit 1
fi

if [[ ! -x "$workspace/run_fleet_demo_ros.sh" || ! -x "$workspace/run_fleet_dashboard.sh" ]]; then
  echo "Required SWARMX launch scripts are missing or not executable."
  exit 1
fi

x-terminal-emulator \
  --title='SWARMX Gazebo + ROS' \
  -e bash -lc "cd '$workspace'; ./run_fleet_demo_ros.sh; exec bash" &

sleep 3

x-terminal-emulator \
  --title='SWARMX Dashboard' \
  -e bash -lc "cd '$workspace'; ./run_fleet_dashboard.sh; exec bash" &

echo "SWARMX startup requested in two Ubuntu Terminal windows."
echo "Dashboard: http://localhost:8090/fleet"
