#!/usr/bin/env bash
# Inside the skybeta-stack container: run the evals (including the OpenCV ones that are skipped elsewhere), then
# the read-only geometry check against whatever depth configuration is running.
#   docker exec skybeta-stack bash /root/team_ws/tools/depth_camera/run_geometry.sh [n_samples]
set -u
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash >/dev/null 2>&1
echo "== evals (container python, OpenCV $(python3 -c 'import cv2; print(cv2.__version__)' 2>/dev/null || echo missing))"
python3 -m unittest 2>&1 | tail -3
echo "== geometry check"
python3 geometry_check.py --n "${1:-5}" --out /root/team_ws/depth_state/geometry.json
