#!/usr/bin/env bash
# Choose which team package + launch file the drone runs. Sets the two settings and returns:
# it does NOT start or restart the stack. The next power-up (battery out 10 s, in) runs it.
#
#   ~/team_ws/run_mission.sh <package> <launch file>
#   ~/team_ws/run_mission.sh snowcrashers_nav low_test.launch.py        0.5 m test
#   ~/team_ws/run_mission.sh snowcrashers_nav competitor.launch.py      2 m mission
#
# Changes only COMPETITOR_PKG / COMPETITOR_LAUNCH in /etc/skybeta/drone.env (asks for the
# sudo password, same as the login). Never PX4 parameters.
set -uo pipefail

PKG="${1:-}"
LAUNCH="${2:-}"
SRC="${HOME}/team_ws/src"
ENV_FILE=/etc/skybeta/drone.env

fail() { printf 'STOP: %s\n' "$*" >&2; exit 1; }

[[ -n "${PKG}" && -n "${LAUNCH}" ]] || fail "usage: $0 <package> <launch file>   e.g. $0 snowcrashers_nav low_test.launch.py"
[[ "$(hostname)" == skybeta-* ]] || fail "this is not a skybeta drone ($(hostname))"

# The package and launch file must exist; anything else in src/ that does not build stops everything.
[[ -f "${SRC}/${PKG}/package.xml" ]] || fail "no package ${SRC}/${PKG} (copy it there first)"
[[ -f "${SRC}/${PKG}/launch/${LAUNCH}" ]] \
  || fail "no launch file ${SRC}/${PKG}/launch/${LAUNCH}. Found: $(ls "${SRC}/${PKG}/launch" 2>/dev/null | tr '\n' ' ')"
others=$(find "${SRC}" -mindepth 1 -maxdepth 1 ! -name "${PKG}" -printf '%f ' 2>/dev/null)
[[ -z "${others}" ]] || echo "note: ${SRC} also holds: ${others}(if any of it does not build, nothing launches)"

sudo sed -i "s/^COMPETITOR_PKG=.*/COMPETITOR_PKG=${PKG}/; s/^COMPETITOR_LAUNCH=.*/COMPETITOR_LAUNCH=${LAUNCH}/" "${ENV_FILE}" \
  || fail "could not edit ${ENV_FILE}"
echo "$(hostname) will run:"
grep '^COMPETITOR_' "${ENV_FILE}"

if [[ "$(systemctl is-enabled skybeta-stack 2>/dev/null)" != enabled ]]; then
  echo "!! autostart is OFF, so nothing runs at power-up. Turn it on once (starts the stack now):"
  echo "   cd ~/skybeta && sudo host/autostart.sh enable"
fi
echo "Applies at the next power-up: battery out 10 s, in, wait ~3 min."
