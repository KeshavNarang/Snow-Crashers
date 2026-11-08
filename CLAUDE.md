# Snow Crashers: agent guide

**Current project guidance:** start at [docs/README.md](docs/README.md).
[docs/STATUS.md](docs/STATUS.md) supersedes stale deployment, camera and competition-rule claims below.

Stanford DroneHacks 2026 entry. Read `README.md` (human notes, scoring), `THOUGHTS.md`,
`goalnav/README.md` and `goalnav/docs/GOALNAV.md` first. Official rules:
`goalnav/docs/organizer/`. Don't edit `README.md` (human-owned). Use git, not jj.

## Auxiliary sensor workstream (historical competition constraints)

The original auxiliary-sensor plan used **one auxiliary sensor** (historical rule 4: < US$100 market value, < 50 g, powered from the
drone battery, declared at inspection with part number, price link and mass; no GNSS, no
off-drone beacons, no lasers above Class 1) to win "technically impressive" points and help
the course. Sensor choice/status lives in `agents/sensor/README.md`. If you are a session
touching sensor code, mounts, drivers or fusion, read that first and log your work there.

## Live pipeline

The user confirms several successful simple `snowcrashers_nav` flights; wall crossing is also reported in the logs. Goalnav was deployed and bench-run props-off, but a successful goalnav course/cheese flight is not established. A newer branch records a team image with on-camera depth (~28 Hz), replacing earlier CPU-stereo assumptions. See [current status](docs/STATUS.md) and [camera contracts](docs/CAMERAS.md); verify the live configuration before deployment.

## Required RGB for pad finding

**Real RGB is non-negotiable for the pad mission. The user confirms enabling
RGB is allowed under competition rules.** Do not treat old organizer-permission
claims as a blocker, or ship mono/depth-only as a ready pad mission. Resolve
VIO/performance issues with RGB retained. Never fake RGB-sensor values, images,
calibration, timestamps or detection results. Report missing/stale data honestly;
simulation images must be clearly identified and never used as live evidence.
See [docs/CAMERAS.md](docs/CAMERAS.md#required-for-the-real-pad-mission).

## Evals first (the workflow)

**Always run the evals before any code is written** that changes behaviour (code, or config that can
change what the drone does). Docs, comments, logs and renames need no new eval. Writing behaviour code
before its eval exists is the one thing this repo does not do. **Evals are a means, not the work: they are
bounded by "Budgets and anti-gaming" below.** For every fix or feature:
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

## Budgets and anti-gaming (hard limits)

Evals exist to catch a real failure, not to look thorough. These limits are hard: at a limit, STOP, write
what is done and what is left in `agents/logs/`, and ask the user before going on. Note the start time in
the log when you begin a task.

**Time (wall clock, whole task from first command to merged PR, evals included)**
| Task | Limit | of which evals + benchmarks at most |
|---|---|---|
| Docs / config note / log | 15 min | none required |
| Bug fix | 45 min | 15 min |
| Speed-up or refactor | 60 min | 20 min |
| New feature or tool | 90 min | 30 min |
| Research / scoring candidates | 30 min | n/a |
| One drone bench session (lock held) | 30 min | n/a, release the lock at the end |
| Landing a PR (merge, conflicts, cleanup) | 10 min | n/a |

**Size**
- At most ~8 tests per change, one per distinct requirement or failure seen on the drone. No sweeps
  "for coverage". A parametrised case counts once only if each row is a different behaviour.
- Any single eval runs in <= 10 s on a laptop; the whole suite stays <= 2 min. A slower eval needs the
  user's approval (use a smaller synthetic scene instead).
- Run only the touched test file while iterating. Run the whole suite exactly twice per change: once
  for the baseline, once on the merged tree before the PR. Do not re-run an unchanged suite.
- At most 2 tuning iterations against one eval threshold. If it still fails, report; do NOT loosen the
  threshold or retune the scene until it passes. If a threshold was wrong, say so in the PR and why.
- Drone measurements: at most 3 runs per question, <= 10 min each. Do not repeat a benchmark to find a
  better number, and do not chase an effect smaller than the run-to-run noise you already measured.

**An eval counts only if**
1. it failed on the current code for the stated reason before the fix (show the failing output), and
2. it asserts an outcome someone cares about (a measured value, a mission result, a published message),
   not that the code does what the code does, and
3. its docstring names the real observation behind it (a log line, a field measurement, a rule number), and
4. it does not duplicate coverage that already exists, and does not mock the thing under test.
An eval written only to be green, or kept only to raise the count, is deleted. Tests that exist to "show
diligence" are the problem this section is here to stop.

## Landing changes (pull requests)

Never push straight to `main`. Work in your own worktree on an `agent/<topic>` branch, then:
1. Evals green on the merged result: `git fetch && git merge origin/main`, then the whole suite
   (other branches land often; the shared `agents/STATUS.md` table conflicts every time, keep both rows).
2. `git push -u origin agent/<topic>`, then `gh pr create --fill` with a body saying what changed, what the
   evals cover, and what was NOT measured on the drone.
3. `gh pr merge <n> --merge` once it shows MERGEABLE / CLEAN. To land someone else's branch, merge `main`
   into it in a throwaway worktree (`git worktree add ../tmp -b tmp/<x> origin/<their-branch>`), push that as a
   plain fast-forward, merge the PR, and remove the worktree. Never rebase or force-push a branch another
   session may have checked out.
4. Remove your worktree when the PR is merged (`git worktree remove ../<name>`).

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

- **Never stop, restart or kill the stack** (the XRCE agent, the `slam` launch, `skybeta-stack`) unless the user
  asks: on stock PX4 v1.16.2 the flight controller's TELEM2 link stays dead until the FC is power-cycled
  (`skybetav1/CLAUDE.md` bug #15; a patched firmware fixes it, not confirmed for skybeta-03). To change the
  camera driver, swap the driver component inside the running container; see
  `agents/logs/2026-11-07-depth-camera-plan.md` (branch `agent/depth-camera`).
- Read-only inspection by default (topics, logs, params). No arming, no motor commands, no
  flight scripts, no PX4 param changes without the user explicitly approving that action.
- Props off on the bench. A human with RC kill switch watches every flight.
- Never run two missions/controllers at once; don't publish to `/fmu/in/*` directly (see
  `skybetav1/docs/competitor_guide.md`).
- Competition constraints on stored course data, fixed waypoints and camera-pipeline changes are historical. The user reports organizers are now flexible; follow [current guidance](docs/STATUS.md#current-competition-guidance), rather than blocking the requested shortcut on those old rules. This does not authorize unrelated motor commands or PX4 changes.

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
- **Sims: one at a time, one run per command. No long-running loops or parallel batches** (user
  rule, 2026-11-07). Look at each result before starting the next; fix failures one by one.
  Put every run under one flat output folder with one index page (`localsim/live_index.py`),
  not separate pages per batch.

## Drone lock etiquette
`tools/drone-lock status` before anything beyond a read-only look. If another agent holds the lock, don't
take it over and don't add load during its measurements (benchmarks are CPU-sensitive): wait, or ask the user.
