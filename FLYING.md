# Flying our software on skybeta-03

Plain steps to put our navigation software ("goalnav") on the drone and fly it. Run every
command from the repo folder in **Git Bash** on your laptop unless it says otherwise.

What flies: our route planner (around the first wall, through the tunnel, through the window,
land on the helipad), from `main`, combined with `agent/goalnav-fly`, which holds the launch
file that starts it through the organizers' flight interface. In the simulator it lands on the
pad in 18 of 20 runs. **It has never flown on the real drone, so go step by step.**

---

## 0. Safety rules (always)

- Only ever connect to **skybeta-03**. Logging into another team's drone disqualifies us.
- **Props off** for every bench test. A person holds the **RC kill switch** for every flight.
- Never run two missions or controllers at once.
- Don't change PX4 parameters, failsafes, the geofence, the `dronehacks` password, `host/doctor.sh` or tailscale.
- Never commit, paste or print the password. It lives in `.secrets/skybeta-03.env`.

## 1. Before you start (once)

1. Your laptop is on the **same Wi-Fi/LAN** as the drone (it is found as `skybeta-03.local`).
2. The password file `.secrets/skybeta-03.env` exists in your repo folder. If it doesn't, ask the team.
3. Check you reach the right drone. It must print `skybeta-03`:
   ```bash
   tools/drone run 'hostname'
   ```
   If the name doesn't resolve, you're on the wrong network. Don't try other addresses.

## 2. Build the code to fly (on your laptop)

Combine `main` (our planner) with `agent/goalnav-fly` (the drone launch setup) in a branch of your own:

```bash
git fetch origin
git checkout -b fly-test origin/main
git merge origin/agent/goalnav-fly
```

The only conflict is `agents/STATUS.md`, a table of who is doing what. Keep every row from both
sides, delete the `<<<<<<<`, `=======` and `>>>>>>>` lines, then:

```bash
git add agents/STATUS.md && git commit --no-edit
```

## 3. Take the drone lock

So nobody else uses the drone at the same time:

```bash
tools/drone-lock status                                      # must say "free"
tools/drone-lock acquire "<your name>" "goalnav flight test"
```

Release it when you're done (step 10).

## 4. Copy the code onto the drone

The drone keeps our package at `~/team_ws/src/drone_ref`. Move the old copy **out of** `team_ws`
(two copies there break the build), then copy the new one in:

```bash
tools/drone run 'mv ~/team_ws/src/drone_ref ~/drone_ref.prev-$(date +%Y%m%d-%H%M) 2>/dev/null; echo moved'
tools/drone push goalnav ~/team_ws/src/drone_ref
tools/drone push team_ws/run_mission.sh ~/team_ws/run_mission.sh
tools/drone push team_ws/preflight.sh ~/team_ws/preflight.sh
```

Optional build check (about a minute). It should end without errors:

```bash
tools/drone run 'docker exec skybeta-stack bash -c "source skybeta_env.sh >/dev/null; team_build.sh --packages-select drone_ref"'
```

## 5. Tell the drone to run goalnav

This only changes which mission starts at the next power-up. It asks for the drone's sudo
password (the same as the login), so use an interactive shell:

```bash
tools/drone shell
~/team_ws/run_mission.sh drone_ref goalnav.launch.py
exit
```

Then **power-cycle**: battery out 10 s, back in, and wait about **3 minutes** for everything to start.
The drone builds our package by itself at power-up.

## 6. Pre-flight check (drone on the floor at the launch mark, props off)

```bash
tools/drone run '~/team_ws/preflight.sh'
```

It prints PASS/WARN/FAIL per check and GO / NO-GO at the end (about 25 s). Read it like this:

- **"mission" FAIL is expected.** The script was written for the other mission (overwall_nav). It
  should name `drone_ref / goalnav.launch.py`.
- **"camera pipeline" must PASS (rgbd, colour on).** Without colour the drone can't find the
  helipad: it flies the course, then gives up and descends where it is. The camera setting
  belongs to the organizers, so ask them to turn colour on if it fails.
- **PX4 link must be up.** If it's down, power-cycle the Pixhawk and wait for "PX4 link up".
- Run it once more with the mode switch on **Altitude**: the last check wants that.

## 7. Bench test (props OFF)

Watch goalnav's status while you carry or hold the drone:

```bash
tools/drone run 'docker exec skybeta-stack bash -c "source skybeta_env.sh >/dev/null; timeout 10 ros2 topic echo /goalnav/status"'
```

You should see status messages (state `IDLE` before arming, plus a floor height). If nothing
shows up, don't fly: check step 4 and step 5.

## 8. First flights: staged, one piece at a time

Make the drone stop, land and disarm after a chosen point of the route, using the launch
setting `tuning` with `stop_after`. Re-run step 5 with the setting added, then power-cycle:

```bash
tools/drone shell
~/team_ws/run_mission.sh drone_ref goalnav.launch.py      # see below for adding tuning
exit
```

Points of the route to stop after, in order (easiest first):

| `stop_after` | The drone stops after... |
|---|---|
| `corridor_1` | taking off and flying up the first corridor |
| `m1_end` | going around the end of the first wall |
| `north_run` | flying east past the third wall |
| `tunnel_exit` | coming out of the tunnel |
| `window_back` | going through the window |
| (none) | the full mission: lands on the helipad, then flies back over the barrier to the launch mark |

The setting goes in as `tuning:="{stop_after: corridor_1}"`, and you can add a lower speed:
`tuning:="{stop_after: corridor_1, max_speed: 0.4}"`. `run_mission.sh` only picks the package and
launch file, so if you're not sure how to pass `tuning` through it, ask whoever set up
`agent/goalnav-fly`.

**Each flight:** pilot sets the mode switch to **Altitude**, arms, then switches to **Position**.
That hands control to goalnav (Offboard). The kill switch stays in hand the whole time. The
flight interface limits the drone to 0.3-2.5 m altitude and 1.5 m/s.

## 9. After each flight: get the logs

Each run saves a recording and logs on the drone in `~/team_ws/logs/<date-time>/`:

```bash
tools/drone run 'ls -t ~/team_ws/logs | head -3'
tools/drone pull ~/team_ws/logs/<that folder> ./flight-logs/
```

Write what happened (and the folder name) in `agents/logs/` so the next person can pick it up.

## 10. When you're done

Put the team's other mission back if needed, and release the lock:

```bash
tools/drone shell
~/team_ws/run_mission.sh overwall_nav overwall.launch.py    # Luke's mission, if that should fly next
exit
tools/drone-lock release
```

(A mission change takes effect at the next power-up.)

---

### If something goes wrong

| Problem | What to do |
|---|---|
| `skybeta-03.local` not found | Wrong network. Join the drone's Wi-Fi/LAN. |
| preflight says "the stack is not running" | Wait ~3 min after power-up and try again. |
| Build error "Duplicate package names" | An old `drone_ref` is still inside `~/team_ws`: move it out (step 4). |
| No `/goalnav/status` | The mission didn't start: redo step 5 and power-cycle. |
| PX4 link down | Power-cycle the Pixhawk, wait for "PX4 link up". |
| Lands without finding the helipad | The camera colour (rgbd) is off: see step 6. |

More detail: `agents/logs/2026-11-07-goalnav-fly.md` (how goalnav was first deployed),
`agents/logs/2026-11-07-planner-rebuilt.md` (the route and its sim results), and
`skybetav1/docs/competitor_guide.md` (the organizers' rules for the drone).
