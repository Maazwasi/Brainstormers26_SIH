#!/usr/bin/env bash
set -eo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
world="$workspace/src/amr_simulation/worlds/warehouse_sih_v2.sdf"
if [[ "${1:-}" == "--headless" ]]; then
  exec gz sim -s -r "$world"
fi
exec gz sim -r "$world"
