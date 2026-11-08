# Flying our software on skybeta-03
Run the commands from the repo folder in **Git Bash** on your laptop.

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

## The cheese run (over the barrier, find the H, land)

Camera: the team image `RGBSTEREO` (colour + mono pair). On-camera depth (`rgbd`) left PX4 without
VIO fusion (vision 665 ms old, 2026-11-07 23:49). Depth comes from goalnav's `stereo_node` on the
Jetson (started by goalnav.launch.py). Slow: `safe` profile, 0.4 m/s; 0.5 s on the pad before the return.
**Not yet bench-checked on the drone** with Jetson depth: before the first flight, after 5 min warm-up,
check PX4 fuses VIO (`preflight.sh`: PX4 position), `/goalnav/stereo/points` at 3 Hz or more, and the
real mat seen by the colour camera (props off).

Over USB, put `DRONE_HOST=192.168.55.1` in front of each command (Wi-Fi needs nothing).

1. `tools/fly camera "<your name>"`: copies the team camera config (`RGBSTEREO`: colour + the
   mono pair) and builds the team image. Power-cycle, wait ~3 min.
2. `tools/fly deploy "<your name>" cheese`: copies goalnav and picks `cheese.launch.py`
   (colour pad camera, cruise 2.25 m, no return, disarms on the pad). Power-cycle, wait ~3 min.
3. `tools/fly check`: the camera pipeline must PASS (RGBSTEREO), PX4 link up, goalnav status printed.
4. Pilot: Altitude, arm, Position. Kill switch in hand. Then `tools/fly logs`, `tools/fly done`.

## The four commands

| Command | What it does |
|---|---|
| `tools/fly deploy "<your name>"` | Takes the drone lock, copies goalnav onto the drone (the old copy is kept in `~/drone_ref.prev-<time>`), checks it builds, and picks goalnav as the mission (asks for the sudo password, which is the login password). |
| *power-cycle the drone* | Battery out 10 s, back in, wait about **3 minutes**. |
| `tools/fly check` | Pre-flight check plus goalnav's status. Drone on the launch mark, props off. |
| `tools/fly logs` | After a flight: copies the newest flight log folder to `./flight-logs/`. |
| `tools/fly done` | Puts Luke's overwall_nav mission back (asks sudo) and releases the lock. Takes effect at the next power-up. |

### Reading `tools/fly check`

- **"mission" FAIL is expected.** The pre-flight script was written for overwall_nav. It should
  name `drone_ref / goalnav.launch.py`.
- **Camera config (read this before flying goalnav).** The active team camera config
  (`team_ws/camera/oakd_vslam.yaml`) is the organizers' stock one plus colour (`RGBSTEREO`): it has
  **no on-camera depth**, so goalnav's `/oak/stereo/image_raw` has no publisher and goalnav cannot
  see walls. The depth + colour config goalnav was built for is kept as
  `team_ws/camera/oakd_vslam.depth.yaml`, but with it PX4 stopped fusing VIO on skybeta-03
  (no valid position: it cannot fly Position/Offboard). Check `PX4 position` in preflight before
  any flight; see `agents/logs/2026-11-07-overwall-pad.md`.
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
`agents/logs/` so the next person can pick it up.

## If something goes wrong

| Problem | What to do |
|---|---|
| `skybeta-03.local` not found | Wrong network. Join the drone's Wi-Fi/LAN. |
| "LOCKED" from deploy | Someone else is using the drone (`tools/drone-lock status`). Wait or ask them. |
| "the stack is not running" | Wait ~3 min after power-up and try again. |
| Build error "Duplicate package names" | An old `drone_ref` is still in `~/team_ws/src`. `deploy` moves it out; check with `tools/drone run 'ls ~/team_ws/src'`. |
| No goalnav status | The mission didn't start: `deploy` again, then power-cycle. |
| Lands without finding the helipad | Camera colour is off (preflight "colour camera" FAIL): see "Reading tools/fly check". |

More detail: `agents/logs/2026-11-07-goalnav-fly.md` (how goalnav was first deployed by hand),
`agents/logs/2026-11-07-planner-rebuilt.md` (the route and its sim results),
`skybetav1/docs/competitor_guide.md` (the organizers' rules).
