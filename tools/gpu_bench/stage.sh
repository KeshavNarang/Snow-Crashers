#!/usr/bin/env bash
# Copy the GPU stereo benchmark to skybeta-03's /tmp and into the skybeta container's /tmp (both
# wiped at reboot). Installed team_ws is not touched. Run from the repo root of a checkout whose
# tools/drone has credentials:   DRONE=../Snow-Crashers/tools/drone tools/gpu_bench/stage.sh
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"; repo="$(cd "$here/../.." && pwd)"
drone="${DRONE:-$repo/tools/drone}"
stage="$(mktemp -d)"; trap 'rm -rf "$stage"' EXIT
mkdir -p "$stage/gpubench/overwall_nav_pkg/test"
cp "$here"/*.py "$stage/gpubench/"
cp -r "$repo/team_ws/src/overwall_nav/overwall_nav" "$stage/gpubench/overwall_nav_pkg/"
cp "$repo/team_ws/src/overwall_nav/test/"{synthetic.py,test_matcher.py} "$stage/gpubench/overwall_nav_pkg/test/"
find "$stage" -name __pycache__ -prune -exec rm -rf {} +
"$drone" run 'mkdir -p /tmp/gpubench'
"$drone" push "$stage/gpubench/." /tmp/gpubench/
"$drone" run '[ "$(hostname)" = skybeta-03 ] || exit 2
cp /usr/lib/python3/dist-packages/vpi.cpython-310-aarch64-linux-gnu.so /tmp/gpubench/
docker exec skybeta-stack mkdir -p /tmp/gpubench
docker cp /tmp/gpubench/. skybeta-stack:/tmp/gpubench/
docker exec skybeta-stack bash -c "PYTHONPATH=/tmp/gpubench python3 -c \"import vpi; print(\\\"staged, vpi\\\", vpi.__version__)\""'
