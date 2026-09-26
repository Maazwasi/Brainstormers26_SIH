#!/usr/bin/env bash
set -eo pipefail
cd /home/maaz-wasi/amr_ws
exec ./run_stage6_negotiation.sh enabled:=false use_rviz:=true
