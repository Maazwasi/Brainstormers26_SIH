#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$workspace_dir"

source /opt/ros/lyrical/setup.bash
source install/local_setup.bash

exec python3 validate_two_amr_negotiation.py --timeout 300 --keep-open --start-delay 15
