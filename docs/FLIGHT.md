# Running on skybeta-03

The simple up/forward/land mission has completed several real flight tests,
confirmed by the user. The full goalnav/cheese route has different readiness:
see [current status](STATUS.md). No PX4 parameter changes are required by the
existing simple mission or the simulated cheese experiment.

## Startup and deployment

Code lives in `~/team_ws/src`. The stack's competitor window builds that workspace
and launches the selected package. `team_ws/run_mission.sh PACKAGE FILE.launch.py`
changes `COMPETITOR_PKG` and `COMPETITOR_LAUNCH` in `/etc/skybeta/drone.env`.
**It does not build, start, restart or enable autostart.** It prints a warning
and the enable command if the service is not enabled.

Autostart was recorded enabled in later hardware sessions. Check its current
status rather than reinstalling it because an old guide says it is absent.
`host/autostart.sh enable` installs/enables the service and starts/restarts it;
that is a separate action from selecting a mission.

Changing the launch selection takes effect at the next stack start. The usual
on-site activation path is a physical power cycle. Replacing only team nodes
inside an existing stack can avoid restarting the agent, but requires knowing
which processes are active and ensuring there is only one controller/interface.
A power cycle is not intrinsically required for every source edit.

**Stock PX4 v1.16.2 can lose its FC-to-Jetson link after the agent/stack stops.**
Later sessions sometimes restarted successfully, but also reproduced the failure.
Do not promise that a software restart will recover it. Check the link and use
an FC power cycle when needed; do not stop the stack without authorization.

## How flight starts

The mission and exactly one `flight_interface` must already be running and
publishing fresh setpoints before the pilot switches modes.

1. Confirm PX4 link, valid local position, landed state, selected mission and
   perception. For the pad mission, real RGB images/calibration and real mat
   detection are mandatory; mono/depth alone or fabricated RGB values do not qualify. For a new mission/configuration, perform a props-off rehearsal.
2. Pilot arms in **Altitude** mode on the ground.
3. Pilot switches to **Position** promptly while still on the ground.
4. With `auto_offboard_on_arm: true`, the interface requests Offboard if its
   checks pass. Goalnav starts when it sees armed + Offboard.

Arming alone does not start autonomy. Arming while already in Position does
not trigger it. The supplied flight interface has no arm service: the pilot
arms; this is an interface design, not a claim that a Jetson cannot technically
send an arm command. The simulator substitutes fake pilot actions.

The documented “within 3 seconds” is an operational recommendation. The
interface does not impose a three-second arm-to-switch deadline; PX4's preflight
auto-disarm timeout and the remaining takeoff time still matter. A mode request
has its own two-second retry window. Inspect actual installed parameters rather
than treating historical defaults as measurements from today's boot.

Pilot stick/mode takeover ends autonomy for that arming. Emergency kill stops
the motors; it is distinct from controlled takeover and landing. Keep a human
pilot with RC control for every flight.

## Landing and logs

The current mission commands descent and waits for PX4's landing detector and
automatic disarm; the interface does not provide arm/re-arm services. The cheese
experiment ends on the mat; the normal goalnav route defaults to a brief armed
pad rest and return, so its settings must not be confused with the shortcut.

Container/tmux screen history can be lost on restart. Goalnav's recorder writes
MCAP, ROS logs and `run.json` under the mounted `~/team_ws/logs/<timestamp>/`,
which survives container recreation. Check recording and available disk space;
full stereo-image recording can consume gigabytes in minutes.

The detailed historical checklist is [DRONE_SETUP.md](../team_ws/DRONE_SETUP.md).
