# Simulator-only cheese experiment

Started 2026-11-07, approximately 19:49 PST. Branch: agent/cheese-run.
Scope: a special local-simulator run and replay, with onboard image and depth
feeds, using existing integrations. No hardware work or PX4 parameter changes.

The special route and command live in `goalnav/experiments/cheese/`.
Baseline: unmodified safe navigator, seed 1, severity 0.7, first 12 simulated
seconds; this is a smoke baseline, not a full course completion evaluation.
No tests added (user's repository instructions).

Result: DONE, disarmed on the pad. Touchdown error 0.055 m; aim error 0.027 m;
zero wall/net contacts, no crash, no bus errors. Four camera frames accepted
the pad. Crossing height 2.259 m; maximum altitude 2.276 m. All four course
checkpoints were skipped. Original camera frames: 107; trace frames: 277.
Simulator run time after cache initialization: 53.5 s.

The baseline reached NAVIGATE with zero touches and no crash in its bounded
12 s interval. Both runs emit the pre-existing all-NaN depth median warning.

Presentation: `cheese.html` embeds a compact sampled camera replay; the full
3D `viewer.html` retains the original camera frames. Inline html_render
succeeded; html_preview failed because headless Chrome exited with code 127.
No browser verification claimed. Artifacts copied to the default workspace's
ignored `goalnav/localsim/out/cheese/seed1/` before worktree cleanup.

## Validation on the rebuilt course (22:00-22:30, branch agent/cheese-validate)

Live skybeta-03 (read-only over USB, 5 min after boot): overwall_nav running, RGB on
(640x400 bgr8, ~8.6 Hz, fx 489.6), CPU stereo /overwall/front ~3.1 Hz, RGB TF pitch 27.0 deg,
px4_link false. Sim now runs --colour --camera-hz 3.0 --camera-latency 0.4.

- Old lookouts (south of the barrier) left the mat below the colour frame at 2.25 m.
- v2 lookouts north of the barrier: 13/20 over severity 0.1-1.0 x both ladder poses. A lookout
  at x -2.80 hit the west net; abort descents beside the barrier hit it.
- v3 lookouts [(-2.10,0.95), (-1.75,-0.25), (-2.45,0.95)]: fixed far-ladder sev 0.9; two prior
  successes re-checked OK. Still failing: near ladder ('original' pose) with the mat in the
  west of the landing area: the ladder hides it from the north, and from the south it is
  cut off at the bottom of the frame. Needs lower lookouts (code: lookouts fly at cruise height).
- Not done: the cheese goals/tunes are not wired into goalnav.launch.py / tools/fly on the drone.

## Drone session, 2026-11-07 23:15-23:55 (USB, lock taken; claude with James)

- Found: goalnav deployed at 23:08 by someone (no lock), RGBSTEREO team image.
- doctor.sh --no-uart: 11 PASS, 2 WARN, 0 FAIL. preflight.sh: PX4 link down until the FC was
  powered by battery (it was off); then link up, battery 15.9 V.
- rgbd image (user: depth on camera), built and rebooted: depth 25 Hz, colour 5-9 Hz, mono pair
  12-19 Hz, cuVSLAM 19-25 Hz. **PX4 did not fuse VIO:** estimator cs_ev_pos/yaw/hgt false, xy_valid
  false, dead reckoning; /fmu/in/vehicle_visual_odometry at 22 Hz but timestamp - timestamp_sample
  = 665 ms. Matches the team's earlier finding (overwall-pad log). Not flyable.
- Restored the RGBSTEREO config (parked copy oakd_vslam.yaml.rgbstereo-2337), rebuilt, reboot sent;
  **the restore was not verified after the reboot** (drone handed back). Check it first next session.

## Plan now (branch agent/cheese-validate)

- goalnav.launch.py: RGBSTEREO colour + stereo_node depth on the Jetson; safe profile 0.4 m/s,
  0.5 s on the pad (PX4 auto-disarms at 2 s landed); south_turn 1 ft further from the changed corner.
- Sim at conservative numbers (depth and colour 2.8 Hz, 0.4 s old, colour camera 27.0 deg; rocks
  >= 1.5 ft off the mat; near ladder <= 1.5 ft off the barrier): cheese near + far ladder landed;
  full course far ladder, near ladder, and near ladder after the corner/pad changes: all DONE,
  returned to the launch, 0 contacts (one 1.7 cm sim touch at t=0 on the launch mark).
- Next drone session (~15 min): verify RGBSTEREO + PX4 fusion after 5 min; stereo_node rate and
  age with colour on (needs >= 3 Hz); dist_bottom present; mat held in view; then fly cheese
  (tools/fly deploy NAME cheese), then the full course.
