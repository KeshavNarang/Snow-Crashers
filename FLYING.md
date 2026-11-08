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

## The four commands

| Command | What it does |
|---|---|
| `tools/fly deploy "<your name>"` | Takes the drone lock, copies goalnav onto the drone (the old copy is kept in `~/drone_ref.prev-<time>`), checks it builds, makes sure the camera runs **RGBSTEREO** (if it doesn't, builds the team camera image, which takes several minutes), and picks goalnav as the mission (asks for the sudo password, which is the login password). |
| *power-cycle the drone* | Battery out 10 s, back in, wait about **3 minutes**. |
| `tools/fly check` | **Both health checks in one go:** the organizers' `host/doctor.sh`, then our pre-flight, then goalnav's status. Drone on the launch mark, props off. |
| `tools/fly logs` | After a flight: copies the newest flight log folder to `./flight-logs/`. |
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
  `agents/logs/2026-11-07-overwall-pad.md`.
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

## Test in the simulator first (optional, on your laptop)

From the `goalnav` folder (`cd goalnav`):

```bash
python -m localsim.goalrun --seed 1 --severity 0.7     # opens a 3D viewer of the run when done
```

It flies the same software against a simulated copy of the course, with the camera modelled as on
the drone (RGBSTEREO). **The stepladder** sits where the team last saw it, which is currently its
**original spot** between the barrier and the mat. If it moves again, change one word in
`goalnav/localsim/messy_ladder.py`: `DEFAULT_POSE = 'original'` or `'moved'` (the south-west net
corner). For one run only, add `--ladder-pose moved`. This only affects the simulator, not the drone.

Latest check (2026-11-08, ladder in its original spot, 25 runs): **24 of 25** land on the helipad
with no contact before landing. Simulator results don't guarantee the real drone will do the same.

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
