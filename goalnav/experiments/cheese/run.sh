#!/usr/bin/env bash
# Run from any directory, with localsim/requirements.txt available.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
exec python3 -m localsim.goalrun \
  --seed 1 --severity 0.7 --duration 90 --colour --camera-hz 2.8 --camera-latency 0.4 \
  --set "goals_file=$PWD/config/cheese_goals.yaml" \
  --tune cruise_height=2.25 --tune handover_height=2.15 \
  --tune return_enabled=false --tune pad_disarm=true \
  --tune heading_until=over_barrier \
  --out localsim/out/cheese/seed1 "$@"
