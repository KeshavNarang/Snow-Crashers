# Current status — 2026-11-07

## What has worked

- **Simple real flights succeeded.** The user confirms several successful tests of
  the older up-and-move mission. The matching repository package is
  `snowcrashers_nav`: `up_forward_land.py` wraps `mission.py`. Do not describe
  VIO hover, Offboard takeoff or the pilot handover as universally unflown.
  This report does not establish which launch/parameters were used on every test.
- **Wall crossing has been reported successful.** The
  [depth log](../agents/logs/2026-11-07-depth-bottleneck.md) identifies the live
  overwall version as “Successful run going over wall.” This is separate from
  the simple flight tests the user confirms.
- **Goalnav was deployed and bench-run, props off.** The
  [third hardware session](../agents/logs/2026-11-07-goalnav-hardware.md#third-session-colour-camera-on-camera-depth-goalnav-deployed-1844-1910-props-off-lock-taken)
  records a successful build and logging/stereo operation, then restoration to
  overwall_nav. It does not record a successful goalnav course flight.
- **The cheese shortcut succeeded in one local simulation.** Local commit
  `fd96af8` adds an experiment that climbs to 2.25 m, crosses the barrier,
  locates the mat in simulated colour and lands/disarms. Reported touchdown
  error: 0.055 m, zero contacts. It uses the older scan-era layout, CPU-stereo
  input model and Python vehicle model. It is not a hardware flight or PX4 SITL.
  The experiment is on the original default workspace's local main history; this docs-only change does not publish its implementation.

## Branches and deployment are different things

The docs refresh starts from `origin/main` at `02e983a`, including the depth-camera
acceptance/geometry tools. This does not include all work on other branches:

| Source | Recorded state | Still unverified |
|---|---|---|
| `agent/goalnav-oak-depth`, handoff commit `71e6096` | Team image installed with on-camera depth; final measured config has colour disabled; goalnav adapter reads depth images and drops CPU stereo | Handoff explicitly says no simulation since adapter changes and no deployment of that adapter against on-camera depth |
| `origin/agent/sim-walls` | Rebuilt maze and relocated ladder represented in that branch's simulator | Physical pose still needs checking; floor tape/checkpoint geometry was not fully remeasured |
| Local `fd96af8` cheese experiment | One successful simulation using the older adapter/layout | Dedicated hardware launch, real mat detection and flight |

Inspect branch contents without switching a shared checkout:

```sh
git show 71e6096:agents/logs/2026-11-07-goalnav-oak-depth-handoff.md
```

No document can prove which mission or camera pipeline is running right now.
Inspect the live drone before deploying. Preserve the distinction between code
merged, code copied, image installed, launch selected, node running and flight proven.

## Current competition guidance

The user reports that no team has completed the challenge and organizers are
being flexible: “pretty much anything goes.” Treat older competition restrictions
on fixed waypoints, stored course information and camera-pipeline changes as
historical assumptions, not automatic blockers to the requested shortcut. The
on-camera-depth work also records organizer approval for changing the camera pipeline.

This is the team's current report, not a rewritten official rulebook or a claim
about final scoring. Keep the existing physical flight controls: pilot RC handover,
one active controller, the shared drone lock and explicit authorization for motor
commands/PX4 changes. Flexible scoring does not fix the documented FC-link restart fault.

## Required RGB and truthful sensor data

The user requires **real RGB, non-negotiably, to find the pad** and explicitly
confirms that enabling it is allowed under competition rules. The previously
shipped depth-only pipeline is therefore not ready for the requested mission.
Resolve performance issues with RGB enabled. Never fake RGB images, sensor
values, calibration, timestamps or detector outputs; report unavailable data
honestly. See [the full RGB requirement](CAMERAS.md#required-for-the-real-pad-mission).

## Immediate goal

Run a real over-wall, find-H, land shortcut. Reconcile the newer depth adapter with
colour input and current course geometry, create its hardware launch, verify the
real mat detector and sensor frames on the bench, then have the pilot initiate flight.
Do not repeat already-successful simple-flight development as if it never happened.
