#!/usr/bin/env bash
# Pre-flight check for overwall_nav (wall + pad), one command, read only: nothing is armed,
# started, restarted or changed. Run it on the drone, drone on the floor at the start spot:
#
#   ~/team_ws/preflight.sh                     (from the laptop: tools/drone run '~/team_ws/preflight.sh')
#
# Prints PASS / WARN / FAIL per check and GO or NO-GO at the end. Takes ~25 s.
# Run it once more with the mode switch on Altitude: the last check wants ALTCTL.
set -uo pipefail

[[ "$(hostname)" == skybeta-03 ]] || { echo "NO-GO: this is $(hostname), not skybeta-03. Log out now."; exit 2; }
docker inspect skybeta-stack >/dev/null 2>&1 || { echo "NO-GO: the stack is not running (wait ~3 min after power-up)"; exit 1; }

fails=0 warns=0
say() {   # status name detail
  printf '  %-4s  %-26s %s\n' "$1" "$2" "$3"
  [[ $1 == FAIL ]] && fails=$((fails + 1))
  [[ $1 == WARN ]] && warns=$((warns + 1))
  return 0
}
r() { docker exec skybeta-stack bash -c "source skybeta_env.sh >/dev/null 2>&1; $1"; }

echo "== $(hostname) pre-flight, $(date '+%H:%M:%S'), up $(uptime -p | sed 's/up //')"

# -- what will fly ------------------------------------------------------------------------
if grep -q '^COMPETITOR_PKG=overwall_nav$' /etc/skybeta/drone.env && grep -q '^COMPETITOR_LAUNCH=overwall.launch.py$' /etc/skybeta/drone.env; then
  say PASS "mission" "overwall_nav / overwall.launch.py"
else
  say FAIL "mission" "drone.env runs $(grep -h '^COMPETITOR_' /etc/skybeta/drone.env | tr '\n' ' ')"
fi
mode=$(docker exec skybeta-stack grep -h 'i_pipeline_type' \
  /root/skybeta_ws/install/oakd_vslam_bringup/share/oakd_vslam_bringup/config/oakd_vslam.yaml 2>/dev/null | awk '{print $2}')
[[ $mode == rgbd ]] && say PASS "camera pipeline" "rgbd (colour on)" \
  || say FAIL "camera pipeline" "'${mode:-?}': no colour, the pad can't be seen (needs rgbd + host/run.sh build + power-cycle)"

# -- the organizers' sensor check (PX4 link, PX4 position, cuVSLAM, camera tilt, IMU ...) ----
echo "-- sensor_check (12 s, keep the drone still)"
sc=$(r "timeout 40 ros2 run overwall_nav sensor_check" 2>&1)
lines=$(printf '%s\n' "$sc" | grep -E '^\s+(PASS|WARN|FAIL)\s' || true)
if [[ -z $lines ]]; then
  say FAIL "sensor_check" "no result: $(printf '%s\n' "$sc" | tail -2 | tr '\n' ' ')"
else
  while IFS= read -r line; do
    status=$(awk '{print $1}' <<<"$line")
    rest=$(sed -E 's/^\s+(PASS|WARN|FAIL)\s+//' <<<"$line")
    say "$status" "${rest:0:26}" "${rest:26}"
  done <<<"$lines"
fi

# -- what sensor_check does not cover: odometry sane, colour + pad_detect, crashes, the switch --
echo "-- position, colour, pad detector (6 s)"
probe=$(r "timeout 30 python3 - <<'EOF'
import json, math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
rclpy.init()
n = Node('preflight_probe')
c = {'rgb': 0, 'pad': 0, 'front': 0}
odom, state = [], {}
n.create_subscription(Image, '/oak/rgb/image_raw', lambda m: c.__setitem__('rgb', c['rgb'] + 1), qos_profile_sensor_data)
n.create_subscription(String, '/overwall/pad', lambda m: c.__setitem__('pad', c['pad'] + 1), 10)
n.create_subscription(String, '/overwall/front', lambda m: c.__setitem__('front', c['front'] + 1), 10)
n.create_subscription(Odometry, '/visual_slam/tracking/odometry',
                      lambda m: odom.append((m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z)), 10)
n.create_subscription(String, '/skybeta/state', lambda m: state.update(json.loads(m.data)), 10)
t0 = time.time()
while time.time() - t0 < 6.0:
    rclpy.spin_once(n, timeout_sec=0.1)
dt = time.time() - t0
out = {k: round(v / dt, 1) for k, v in c.items()}
out['odom_hz'] = round(len(odom) / dt, 1)
if odom:
    first, last = odom[0], odom[-1]
    out['odom_from_origin_m'] = round(math.dist(last, state.get('origin_enu') or (0, 0, 0)), 2)
    out['odom_drift_m'] = round(math.dist(first, last), 3)
    out['odom'] = [round(v, 2) for v in last]
out['nav_state'] = state.get('nav_state_name')
print('PROBE ' + json.dumps(out))
EOF" 2>&1 | grep '^PROBE ' | sed 's/^PROBE //')
if [[ -z $probe ]]; then
  say FAIL "probe" "could not read the topics"
else
  get() { python3 -c "import json,sys; v=json.loads(sys.argv[1]).get(sys.argv[2]); print('' if v is None else v)" "$probe" "$1"; }
  ge() { python3 -c "import sys; print(int(float(sys.argv[1] or -1) >= float(sys.argv[2])))" "$1" "$2"; }
  le() { python3 -c "import sys; print(int(sys.argv[1] != '' and float(sys.argv[1]) <= float(sys.argv[2])))" "$1" "$2"; }
  d0=$(get odom_from_origin_m); dd=$(get odom_drift_m)
  if [[ -z $d0 ]]; then say FAIL "odometry position" "no odometry"
  elif [[ $(le "$d0" 0.5) == 1 && $(le "$dd" 0.05) == 1 ]]; then say PASS "odometry position" "$(get odom) m, $d0 m from the origin, moved $dd m in 6 s"
  else say FAIL "odometry position" "$(get odom) m ($d0 m from the origin, moved $dd m in 6 s): VIO lost. Power-cycle without touching it"; fi
  v=$(get rgb);   [[ $(ge "$v" 5) == 1 ]] && say PASS "colour camera" "$v Hz" || say FAIL "colour camera" "${v:-0} Hz (~9 expected): no pad search"
  v=$(get pad);   [[ $(ge "$v" 2) == 1 ]] && say PASS "pad_detect" "$v Hz" || say FAIL "pad_detect" "${v:-0} Hz (~4 expected)"
  v=$(get front); [[ $(ge "$v" 2) == 1 ]] && say PASS "stereo_depth" "$v Hz" || say FAIL "stereo_depth" "${v:-0} Hz (~4 expected)"
  nav=$(get nav_state)
  [[ $nav == ALTCTL ]] && say PASS "mode switch" "ALTCTL: arm, then Position within 3 s" \
    || say WARN "mode switch" "PX4 is in '${nav:-?}': put the switch on Altitude and run this again (must say ALTCTL)"
fi

crash=$(docker exec skybeta-stack tmux capture-pane -p -J -S -2000 -t skybeta:competitor 2>/dev/null \
  | grep -E 'Traceback|process has died|colcon build FAILED' | tail -1)
[[ -z $crash ]] && say PASS "our nodes" "no crash in the log" || say FAIL "our nodes" "$crash"

echo
if (( fails )); then
  echo "NO-GO: $fails FAIL, $warns WARN. Fix the FAILs; do not arm."
  exit 1
fi
echo "GO ($warns WARN, read them; 'obstacle check' WARN is the wall itself, 27 in ahead)."
echo "Pilot: switch on Altitude (mode switch must read ALTCTL above), arm, Position within 3 s. Throttle to the middle takes over."
