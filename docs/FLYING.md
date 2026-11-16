# Flying our software on skybeta-03
Run the commands from the repo folder in **Git Bash** on your laptop.

This guide flies goalnav, the full-course mission. To fly by hand with everything recorded, see
[MANUAL_FLIGHT.md](MANUAL_FLIGHT.md). How the start-up and the pilot's handover work underneath
is in [How the start works](#how-the-start-works) at the end.

## Safety rules (always)

- Only ever connect to **skybeta-03** (`tools/fly` stops if it's anything else).
- **Props off** for bench tests. A person holds the **RC kill switch** for every flight.
- Never run two missions or controllers at once.
- Don't change PX4 parameters, failsafes, the geofence, the password, `host/doctor.sh` or tailscale.
- Never commit, paste or print the drone password.

## Before you start (once)

- Your laptop is on the **same Wi-Fi/LAN** as the drone.
- Know the drone's login password (user `dronehacks`). The organizers gave it to the team; ask
  them if nobody has it. Each `tools/fly` command asks for it once, hidden, and keeps it in memory
  only. (If a `.secrets/skybeta-03.env` file exists in the repo folder, it's used instead and you
  won't be asked.)
- `git checkout main && git pull` so you fly the latest code.

## Flying the full course (goalnav)

Camera: the team image `RGBSTEREO` (colour + mono pair). On-camera depth (`rgbd`) left PX4 without
VIO fusion (vision 665 ms old, 2026-11-07 23:49). Depth comes from goalnav's `stereo_node` on the
Jetson (goalnav.launch.py, camera:=rgbstereo). Slow: `safe` profile, 0.4 m/s; 0.5 s on the pad, then
the return. Over USB, put `DRONE_HOST=192.168.55.1` in front of each command (Wi-Fi needs nothing).

Do steps 1-2 **off the pad** so nobody waits on the warm-up.

1. `tools/fly deploy "<your name>"`: copies goalnav, makes sure the camera runs RGBSTEREO, picks
   goalnav. Then reboot the Jetson (`sudo systemctl reboot` on it: a battery cycle does not reboot it
   while it has other power). Wait ~3 min to boot, then **5 min more** before measuring.
2. `tools/fly bench` (props off, ~1.5 min, read-only): needs PX4 fusion >= 95 % over 30 s, median
   vision age < 150 ms, depth >= 3 Hz. It also prints CPU per process. NO-GO: stop, do not take the pad.
3. On the course: check the props clear barrier panel 1's stand at the launch mark (its struts and
   feet now face the launch corridor). If tight, place the drone further north on the mark.
4. On the pad: `tools/fly check`, then the pilot: Altitude, arm, Position. Kill switch in hand.
   goalnav holds still whenever PX4 loses vision (6 s: it comes down where it is).
5. After: `tools/fly logs`, `python3 tools/postflight.py flight-logs/<run>`, `tools/fly done`.

## The four commands

| Command | What it does |
|---|---|
| `tools/fly deploy "<your name>"` | Takes the drone lock, copies goalnav onto the drone (the old copy is kept in `~/drone_ref.prev-<time>`), checks it builds, makes sure the camera runs **RGBSTEREO** (if it doesn't, builds the team camera image, which takes several minutes), and picks goalnav as the mission (asks for the sudo password, which is the login password). |
| *power-cycle the drone* | Battery out 10 s, back in, wait about **3 minutes**. |
| `tools/fly check` | **Both health checks in one go:** the organizers' `host/doctor.sh`, then our pre-flight, then goalnav's status. Drone on the launch mark, props off. |
| `tools/fly logs` | After a flight: copies the newest flight's log folder from the drone to `./flight-logs/` on your laptop, checks every file arrived (same file count and bytes as on the drone; a cut-short copy is redone, a complete one isn't copied twice), then prints the flight summary (`tools/postflight.py`: mission states, PX4, sensor rates, errors; needs `pip install rosbags` once). `tools/fly logs list` shows recent flights; `tools/fly logs <folder>` pulls an older one, e.g. a failed run. Wait until the flight has ended: the recording (with video) can be several GB. |
| `python3 tools/postflight.py flight-logs/<run>` | One-minute summary: goalnav states and events, PX4 link/mode/VIO fusion/height, depth and colour rates, node errors; saves `summary.json` (needs `pip install rosbags`). |
| `tools/fly done` | Puts Luke's overwall_nav mission back (asks sudo) and releases the lock. Takes effect at the next power-up. |

### Reading `tools/fly check`

- **The organizers' doctor (part 1) should PASS everything.** If it reports a FAIL, read it and
  sort it out before flying (ask the organizers if it's their side).
- **Expected FAILs in our pre-flight (part 2):** "mission" (it names `drone_ref /
  goalnav.launch.py`; the script was written for overwall_nav), and "pad_detect" and "wall depth
  rate" (overwall_nav's own programs, which don't run with goalnav). Everything else should PASS,
  especially **PX4 position**, **PX4 arming checks**, **battery** and **odometry position**.
- **Camera must be RGBSTEREO** (colour + the black-and-white pair). That's the team camera config
  (`team_ws/camera/oakd_vslam.yaml`); `deploy` installs it if needed. RGBSTEREO has **no on-camera
  depth**, so goalnav now makes its own depth from the black-and-white pair (its `stereo_node`,
  started by the launch with `camera:=rgbstereo`, the default) and finds the helipad in colour. The
  older depth + colour config (`team_ws/camera/oakd_vslam.depth.yaml`, goalnav `camera:=rgbd`) made PX4
  stop fusing VIO on skybeta-03 (no valid position, can't fly Position/Offboard), so don't use it; see
  `docs/logs/2026-11-07-overwall-pad.md`.
- **PX4 link must be up.** If it's down, power-cycle the Pixhawk.
- It must print a **goalnav status** at the end. If it says "NO goalnav status", don't fly: run
  `deploy` again and power-cycle.
- Run it once more with the mode switch on **Altitude**: the last pre-flight check wants that.

## Flying

**Each flight:** pilot sets the mode switch to **Altitude**, arms, then switches to **Position**.
That hands control to goalnav. The kill switch stays in hand the whole time. The flight interface
limits the drone to 0.3-2.5 m altitude and 1.5 m/s.

**First flights should be staged:** the drone lands and disarms after one part of the route,
using goalnav's `tuning` launch setting with `stop_after`:

| `stop_after` | The drone stops after... |
|---|---|
| `corridor_1` | taking off and flying up the first corridor |
| `m1_end` | going around the end of the first wall |
| `north_run` | flying east past the third wall |
| `tunnel_exit` | coming out of the tunnel |
| `window_back` | going through the window |
| (none) | the full mission: lands on the helipad, then flies back over the barrier to the launch mark |

Example: `tuning:="{stop_after: corridor_1, max_speed: 0.4}"`. **Not built into `tools/fly`
yet:** `run_mission.sh` only picks the package and launch file, so ask whoever set up the goalnav
launch how the drone's start-up passes `tuning` through.

After each flight run `tools/fly logs`, and write what happened (and the log folder name) in
`docs/logs/` so the next person can pick it up.

## Test in the simulator first

Optional, on your laptop: see [SIMULATION.md](SIMULATION.md).

## If something goes wrong

| Problem | What to do |
|---|---|
| `skybeta-03.local` not found | Wrong network. Join the drone's Wi-Fi/LAN. |
| "LOCKED" from deploy | Someone else is using the drone (`tools/drone-lock status`). Wait or ask them. |
| "the stack is not running" | Wait ~3 min after power-up and try again. |
| Build error "Duplicate package names" | An old `drone_ref` is still in `~/team_ws/src`. `deploy` moves it out; check with `tools/drone run 'ls ~/team_ws/src'`. |
| No goalnav status | The mission didn't start: `deploy` again, then power-cycle. |
| Lands without finding the helipad | Camera colour is off (preflight "colour camera" FAIL): see "Reading tools/fly check". |

More detail: `docs/logs/2026-11-07-goalnav-fly.md` (how goalnav was first deployed by hand),
`docs/logs/2026-11-07-planner-rebuilt.md` (the route and its sim results),
`skybetav1/docs/competitor_guide.md` (the organizers' rules).

## How the start works

The simple up/forward/land mission has completed several real flight tests,
confirmed by the user. The full goalnav route has different readiness:
see [current status](STATUS.md). No PX4 parameter changes are required by the
existing simple mission or goalnav.

### Startup and deployment

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

### How flight starts

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

### Landing and logs

The current mission commands descent and waits for PX4's landing detector and
automatic disarm; the interface does not provide arm/re-arm services. The goalnav
route rests 0.5 s on the pad from its own touchdown and climbs before PX4's 2 s
auto-disarm, then returns.

Container/tmux screen history can be lost on restart. Goalnav's recorder writes
MCAP, ROS logs and `run.json` under the mounted `~/team_ws/logs/<timestamp>/`,
which survives container recreation. Check recording and available disk space;
full stereo-image recording can consume gigabytes in minutes.

The detailed historical checklist is [DRONE_SETUP.md](../team_ws/DRONE_SETUP.md).
