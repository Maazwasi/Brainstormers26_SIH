#!/usr/bin/env bash
set -eo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$workspace"
exec ./run_stage6_negotiation.sh enabled:=false use_rviz:=true
