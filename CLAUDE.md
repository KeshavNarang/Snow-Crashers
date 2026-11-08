# Snow Crashers: agent guide

Stanford DroneHacks 2026 entry. Read `README.md` (human notes, scoring), `THOUGHTS.md`,
`goalnav/README.md` and `goalnav/docs/GOALNAV.md` first. Official rules:
`goalnav/docs/organizer/`. Don't edit `README.md` (human-owned). Use git, not jj.

## Current focus: the auxiliary sensor (SENSOR WORKSTREAM)

We are adding **one auxiliary sensor** (rule 4: < US$100 market value, < 50 g, powered from the
drone battery, declared at inspection with part number, price link and mass; no GNSS, no
off-drone beacons, no lasers above Class 1) to win "technically impressive" points and help
the course. Sensor choice/status lives in `agents/sensor/README.md`. If you are a session
touching sensor code, mounts, drivers or fusion, read that first and log your work there.

## Live pipeline

The pipeline that actually flies on skybeta-03 is `team_ws/src/overwall_nav` (and `snowcrashers_nav`), run through the organizers' `flight_interface`. `goalnav/` is simulator-only so far. Depth is computed on the Jetson CPU from the mono pair (about 3 Hz, ~0.4 s old); see `agents/logs/2026-11-07-depth-bottleneck.md`.

## Evals first (the workflow)

**Always run the evals before any code is written. No exceptions** (a one-line config change included,
if it can change behaviour). Writing code before the eval exists is the one thing this repo does not do.
For every fix or feature:
0. **Run the existing suite first** to get a baseline, so you know what was green before you touched
   anything (`cd team_ws/src/overwall_nav && python -m unittest discover -s test`; mission tests need
   `PYTHONPATH=../../../skybetav1/src/skybeta_flight_interface`). Profile before optimising: measure where
   the time goes, then write the eval for that.
1. **Write the eval first** and run it on the current code. It must fail for the right reason.
   For a bug, build a case that reproduces what was seen on the drone, and add a check that the
   old behaviour really is the bug, so the eval cannot pass by accident.
2. **Then build the fix**, the smallest change that turns the evals green. Don't change a test to
   fit the code.
3. **Keep the old behaviour covered** (backward compatibility) and add a case for what must not
   happen (e.g. the fix hiding a real fault).
4. **Run the whole suite** before committing, and again on the drone's own Python if the change
   touches the Jetson (tests under `team_ws/src/overwall_nav/test/`, `tools/drone_tools/`).
   For a speed-up, keep a FROZEN copy of the old function in the test and assert same answers plus a
   speed ratio in the same process (see `test/test_floorfit_speed.py`).
5. **Measure on the drone** only after the evals pass, and record numbers in `agents/logs/`.
   A measurement taken in the first ~5 minutes after boot is warm-up, not steady state.
Use a git worktree (`git worktree add ../<name> -b agent/<topic> origin/main`) rather than
switching branches in the shared checkout; other sessions work in it.

## Our test drone

`skybeta-03.local`, user `dronehacks`. **Only ever connect to this drone.** SSHing into any other
team's drone means disqualification (`skybetav1/README.md` rules). Run `hostname` after login and
confirm `skybeta-03`. Don't edit/disable `host/doctor.sh`, the status agent, or tailscale. Don't
change the `dronehacks` password or PX4 safety settings (failsafes, kill switch, geofence, battery).

Credentials are in `.secrets/skybeta-03.env` (gitignored, local only). **Never commit, print,
paste into docs/PRs, or copy the password elsewhere.** If the file is missing, ask the user.

```
tools/drone run 'hostname; ros2 topic list'   # one-off command
tools/drone shell                              # interactive
tools/drone push LOCAL REMOTE / pull REMOTE LOCAL
```

The drone is on the local network (`.local` mDNS), so you must be on the same Wi-Fi/LAN. If
the hostname does not resolve, say so; do not guess other hosts.

## Safety (non-negotiable)

- Read-only inspection by default (topics, logs, params). No arming, no motor commands, no
  flight scripts, no PX4 param changes without the user explicitly approving that action.
- Props off on the bench. A human with RC kill switch watches every flight.
- Never run two missions/controllers at once; don't publish to `/fmu/in/*` directly (see
  `skybetav1/docs/competitor_guide.md`).
- Rules that affect code: no stored run data fed to later runs (6.4), all compute onboard (5.4),
  no hard-coded course (hurts generality; an AI code review screens the repo).

## Working together (multiple agents)

- Coordination board: `agents/STATUS.md`. Before starting, read it. Claim a task by adding a row
  (owner, files, status). On finishing or stopping, update it. Keep it short.
- One workstream per branch: `agent/<topic>` (e.g. `agent/sensor-driver`). Small commits,
  rebase on `main`, don't force-push others' branches. Check `git status` before editing; others
  may have uncommitted work in the tree.
- Hardware is a shared, single resource: use `tools/drone-lock`; see
  `agents/PROTOCOL.md` for the lock, logging and handoff rules.
- Keep findings in files (`agents/sensor/`, `agents/logs/`), not only in chat, so the next
  session can pick up.
- Simulate first: `goalnav/localsim` (see `goalnav/README.md`). Run `python3 -m pytest` where
  tests exist before pushing.

## Drone lock etiquette
`tools/drone-lock status` before anything beyond a read-only look. If another agent holds the lock, don't
take it over and don't add load during its measurements (benchmarks are CPU-sensitive): wait, or ask the user.
