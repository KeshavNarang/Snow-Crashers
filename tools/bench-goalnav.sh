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
# Fusion and vision age over 30 s, not one snapshot (2026-11-08: one snapshot showed fusion dropped
# while 18/18 samples over 20 s were fused). Want PX4 fusing >= 95 % and median vision age < 150 ms.
echo "  ....  sampling PX4 fusion and vision age for 30 s"
r "timeout 30 ros2 topic echo --qos-reliability best_effort --field cs_ev_pos /fmu/out/estimator_status_flags > /tmp/bench_fuse.txt &
   timeout 30 ros2 topic echo /fmu/in/vehicle_visual_odometry > /tmp/bench_vio.txt; wait"
fz=$(docker exec skybeta-stack grep -cx True /tmp/bench_fuse.txt); fn=$(docker exec skybeta-stack grep -cxE 'True|False' /tmp/bench_fuse.txt)
pct=$(awk -v a="$fz" -v b="$fn" 'BEGIN {print (b > 0) ? int(100 * a / b) : 0}')
(( fn > 0 && pct >= 95 )) && say PASS "PX4 fuses VIO (30 s)" "$pct % of $fn samples" \
  || say FAIL "PX4 fuses VIO (30 s)" "$pct % of $fn samples (need >= 95 %): no steady position, do not fly"
ages=$(docker exec skybeta-stack awk '/^timestamp:/ {t=$2} /^timestamp_sample:/ {printf "%d\n", (t - $2) / 1000}' /tmp/bench_vio.txt | sort -n)
n=$(wc -l <<<"$ages"); med=$(sed -n "$(( (n + 1) / 2 ))p" <<<"$ages"); p90=$(sed -n "$(( (n * 9 + 9) / 10 ))p" <<<"$ages")
[[ -n $med ]] && (( med < 150 )) && say PASS "vision age (30 s)" "median $med ms, 90 % under $p90 ms ($n samples)" \
  || say FAIL "vision age (30 s)" "median ${med:-?} ms, 90 % under ${p90:-?} ms (need median < 150)"
lp=$(r "timeout 5 ros2 topic echo --once --qos-reliability best_effort /fmu/out/vehicle_local_position")
grep -q '^xy_valid: true' <<<"$lp" && say PASS "PX4 position" "xy_valid" || say FAIL "PX4 position" "xy_valid false"
# CPU per process (one 2 s sample): the camera/cuVSLAM containers need their share or vision is late.
cpu=$(top -b -d 2 -n 2 | awk '/^top -/ {n++} n == 2 && $1 ~ /^[0-9]+$/ {print $9, $12}' \
      | grep -E 'compone|goalnav|stereo_|ros2|flight_|python3' | sort -rn | head -6 | awk '{printf "%s %s%%  ", $2, $1}')
load=$(cut -d' ' -f1 /proc/loadavg)
say INFO "CPU (load $load)" "$cpu"
gs=$(r "timeout 6 ros2 topic echo --once --full-length /goalnav/status")
state=$(grep -o '"state": "[A-Z_]*"' <<<"$gs" | cut -d'"' -f4)
[[ -n $state ]] && say PASS "goalnav status" "$state" || say FAIL "goalnav status" "none"
grep -q '"floor_z": null' <<<"$gs" && say WARN "floor (goalnav)" "not set yet (set on arming / rangefinder)" || say PASS "floor (goalnav)" "set"
last=$(ls -t ~/team_ws/logs 2>/dev/null | grep -v overwall | head -1)
[[ -n $last && -d ~/team_ws/logs/$last/bag ]] && say PASS "flight log" "~/team_ws/logs/$last (bag recording)" || say FAIL "flight log" "no bag folder"
echo
(( fails == 0 )) && echo "GO for props-on: $warns WARN" || echo "NO-GO: $fails FAIL, $warns WARN"
