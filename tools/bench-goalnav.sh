#!/usr/bin/env bash
# goalnav bench check on skybeta-03, props off, READ-ONLY (~1 min). Run ON the drone:
#   tools/fly bench        (copies this script over and runs it)
# PASS / WARN / FAIL per check, then GO or NO-GO. Writes and changes nothing.
set -uo pipefail
fails=0 warns=0
say() { printf '  %-4s  %-22s %s\n' "$1" "$2" "$3"; [[ $1 == FAIL ]] && fails=$((fails + 1)); [[ $1 == WARN ]] && warns=$((warns + 1)); true; }
r() { docker exec skybeta-stack bash -c "source skybeta_env.sh >/dev/null 2>&1; $1" 2>/dev/null; }
hz() { r "timeout 6 ros2 topic hz $1" | awk '/average rate/ {v=$3} END {print v+0}'; }
ge() { awk -v a="$1" -v b="$2" 'BEGIN {exit !(a >= b)}'; }

echo "== goalnav bench, $(hostname), $(date +%T), up $(uptime -p | sed 's/up //')"
up=$(cut -d. -f1 /proc/uptime)
(( up >= 300 )) && say PASS warm-up "up $((up / 60)) min" || say WARN warm-up "up $((up / 60)) min: rates run low for ~5 min after boot"
launch=$(grep -h '^COMPETITOR_LAUNCH=' /etc/skybeta/drone.env | cut -d= -f2)
grep -q '^COMPETITOR_PKG=drone_ref$' /etc/skybeta/drone.env && say PASS mission "drone_ref / $launch" \
  || say FAIL mission "$(grep -h '^COMPETITOR_' /etc/skybeta/drone.env | tr '\n' ' ')"
mode=$(docker exec skybeta-stack grep -h -m1 -E '^[[:space:]]*i_pipeline_type:' \
  /root/skybeta_ws/install/oakd_vslam_bringup/share/oakd_vslam_bringup/config/oakd_vslam.yaml 2>/dev/null | awk '{print $2}')
[[ ${mode^^} == RGBSTEREO ]] && say PASS camera "RGBSTEREO" || say FAIL camera "'${mode:-?}' (want RGBSTEREO)"
for p in goalnav_node stereo_node flight_interface; do
  pgrep -f "lib/[^ ]*/$p" >/dev/null && say PASS "$p" running || say FAIL "$p" "not running (ros: ~/team_ws/logs/<newest>/ros)"
done
v=$(hz /goalnav/stereo/points); ge "$v" 3 && say PASS "depth (Jetson)" "$v Hz" || say FAIL "depth (Jetson)" "$v Hz (need >= 3; sim assumes 2.8)"
v=$(hz /oak/rgb/image_raw); ge "$v" 5 && say PASS colour "$v Hz" || say FAIL colour "$v Hz (pad search needs colour)"
v=$(hz /visual_slam/tracking/odometry); ge "$v" 20 && say PASS cuVSLAM "$v Hz" || say FAIL cuVSLAM "$v Hz"
st=$(r "timeout 5 ros2 topic echo --once --full-length /skybeta/state")
grep -q '"px4_link": true' <<<"$st" && say PASS "PX4 link" connected || say FAIL "PX4 link" "down: battery power-cycle (flight controller + Jetson)"
ev=$(r "timeout 5 ros2 topic echo --once --qos-reliability best_effort /fmu/out/estimator_status_flags")
lp=$(r "timeout 5 ros2 topic echo --once --qos-reliability best_effort /fmu/out/vehicle_local_position")
grep -q '^cs_ev_pos: true' <<<"$ev" && grep -q '^xy_valid: true' <<<"$lp" && say PASS "PX4 fuses VIO" "cs_ev_pos, xy_valid" \
  || say FAIL "PX4 fuses VIO" "no (cs_ev_pos/xy_valid false): no position, do not fly"
vo=$(r "timeout 5 ros2 topic echo --once /fmu/in/vehicle_visual_odometry")
age=$(awk '/^timestamp:/ {t=$2} /^timestamp_sample:/ {s=$2} END {if (t && s) printf "%d", (t - s) / 1000}' <<<"$vo")
[[ -n $age ]] && (( age < 150 )) && say PASS "vision age" "$age ms" || say FAIL "vision age" "${age:-?} ms (PX4 rejects stale vision)"
gs=$(r "timeout 6 ros2 topic echo --once --full-length /goalnav/status")
state=$(grep -o '"state": "[A-Z_]*"' <<<"$gs" | cut -d'"' -f4)
[[ -n $state ]] && say PASS "goalnav status" "$state" || say FAIL "goalnav status" "none"
grep -q '"floor_z": null' <<<"$gs" && say WARN "floor (goalnav)" "not set yet (set on arming / rangefinder)" || say PASS "floor (goalnav)" "set"
last=$(ls -t ~/team_ws/logs 2>/dev/null | grep -v overwall | head -1)
[[ -n $last && -d ~/team_ws/logs/$last/bag ]] && say PASS "flight log" "~/team_ws/logs/$last (bag recording)" || say FAIL "flight log" "no bag folder"
echo
(( fails == 0 )) && echo "GO for props-on: $warns WARN" || echo "NO-GO: $fails FAIL, $warns WARN"
