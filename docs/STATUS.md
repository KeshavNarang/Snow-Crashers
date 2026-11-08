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
- **Full goalnav course in the simulator (2026-11-08, PR #24).** The cheese shortcut is dropped
  (team decision: the full course only). Sim only (Python vehicle model; the pilot's Altitude, arm,
  Position start is modelled, not PX4 SITL). Inputs: rebuilt walls; depth and colour at 2.8 Hz,
  0.4 s old (Jetson depth measured 3.15 Hz, 90-400 ms); colour camera 27.0 deg (TF); near ladder
  within 1.5 ft of the barrier; rocks at least 1.5 ft from the mat's edge; safe profile 0.4 m/s;
  0.5 s on the pad from our own touchdown; return spot 0.725 m off the barrier and halfway between
  the west net and M1. Full course, near and far ladder: done and returned, 0 contacts (seed 405:
  0.85 s on the pad floor, rotor tips 0.21 m from the west net). Details: agents/logs/2026-11-07-cheese-sim.md.

## Live drone, read 2026-11-08 00:26 (USB, read-only)

`RGBSTEREO` team image, `overwall_nav` selected. PX4 link up after a flight-controller
power-cycle; **PX4 fuses VIO** (cs_ev_pos, cs_ev_yaw true, xy_valid true), vision 62-105 ms
old; rangefinder height in use (cs_rng_hgt). Colour 9.1 Hz, mono pair 29 Hz, cuVSLAM 31-32 Hz.
CPU stereo (overwall `stereo_depth`, the same StereoRig as goalnav's stereo_node) 3.15 Hz,
age 88-400 ms, with colour on, 9 min after boot. With `rgbd` (23:49) vision was 665 ms old
and PX4 did not fuse. goalnav (`drone_ref`) is not built in the drone's current workspace.

## Live drone, read 2026-11-07 21:55 (USB, 5 min after boot, read-only)

skybeta-03 ran `overwall_nav` with CPU `stereo_depth` (~3.1 Hz), not goalnav. Colour was
on (`/oak/rgb/image_raw`, 640x400 bgr8, ~8.6 Hz, fx 489.6); no on-camera depth topic;
`px4_link: false`. The team image is `RGBSTEREO` (colour, no on-camera depth): with `rgbd`
or `depth` PX4 stopped fusing VIO (PR #18, overwall-pad log). goalnav now makes its depth
with stereo_node on the Jetson (camera:=rgbstereo).

## Branches and deployment are different things

The docs refresh starts from `origin/main` at `02e983a`, including the depth-camera
acceptance/geometry tools. This does not include all work on other branches:

| Source | Recorded state | Still unverified |
|---|---|---|
| `agent/goalnav-oak-depth`, handoff commit `71e6096` | Team image installed with on-camera depth; final measured config has colour disabled; goalnav adapter reads depth images and drops CPU stereo | Handoff explicitly says no simulation since adapter changes and no deployment of that adapter against on-camera depth |
| `origin/agent/sim-walls` | Rebuilt maze and relocated ladder represented in that branch's simulator | Physical pose still needs checking; floor tape/checkpoint geometry was not fully remeasured |

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
