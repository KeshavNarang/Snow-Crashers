# Sim RGBSTEREO depth and colour matched to skybeta-03 (2026-11-08)

Start 09:07 PST. Branch `agent/sim-depth-match`. Simulator only. No drone contact in this task.

## Source

Read-only capture on skybeta-03 (2026-11-08), done by an earlier session: point clouds from
`/goalnav/stereo/points`, RGB frames, receive-time logs, camera infos and TF. The raw data sat in
`/tmp/simcmp/` on the laptop and is not committed. It came from `drone_ref/stereo_node.py`, which runs
`overwall_nav.stereo.StereoRig` (scale 0.4, 96 disparities, step 2, cols (96, 410), rows 0-90 %).

| | real (2026-11-08) | sim before |
|---|---|---|
| depth FOV across | -22.8..+37.5 deg | +-38.75 |
| depth FOV down | -26.3..+20.6 deg | +-26.6 |
| disparity noise | ~0.3 px, fB 19.87 px m, sigma_z ~ 0.015 z^2 | OAK-D model, then speckle-filtered |
| outliers > 15 cm | 3.5 % | none left |
| dropout | follows texture, dark floor gave none, ~39 % of matchable grid | 50 % iid per point |
| points/frame | ~6700 (goalnav caps 6000) | <= 4000 grid, ~2000 |
| depth timing | 3.77 Hz; interval median 267 ms, p90 383, max 567; age median 0.111, mean 0.159, p90 0.28, max 0.52 | fixed 4.5 Hz, 0.13 s |
| colour | 640x400 bgr8 10 Hz, 46 ms old, own clock; principal point (315.1, 196.6); dark room mean 14/255 | with each depth frame, exact centre |

Deciles of the depth frame interval and age (47 frames) are in `localsim/jetson_stereo.py`.

## What changed

- `localsim/jetson_stereo.py` (new) models stereo_node. It casts rays only at the pixels StereoRig
  searches (157 x 115 at the real intrinsics), applies disparity noise `z' = fB/(fB/z + N(0, 0.3))` and 3 %
  gross mismatches (2-12 px), and fills gaps by surface. Holes are smooth blobs with a per-surface fill rate:
  printed tarp 0.8, hall 0.55, floor 0.38, plywood 0.35, wire mesh 0.3. Every return at floor height
  counts as floor, including the hall's floor beyond the net, which `MessyWorld` labels "beyond".
  `--dark-floor` sets the floor's fill to 0.
- `goalrun`: rgbstereo depth frames follow the measured interval and age deciles, scaled so
  `--camera-hz` and `--camera-latency` set the means. The defaults are now 3.77 Hz and 0.159 s.
  Colour runs on its own clock (`--colour-hz`, default 10) and is 46 ms old. The principal point is
  scaled from (315.1, 196.6). `--colour-exposure dark` gives about 14/255 (gain 0.11); the default
  exposure is unchanged. rgbd mode keeps its old fixed timing.
- `MessyWorld.observe` is split. `surfaces()` returns the true ray lengths and surface kinds, and the
  OAK-D model (rgbd) is applied after it as before.
- `drone_measured.yaml` scenario `drone` is now 3.77 Hz, 0.159 s.

## Evals

`goalnav/test/test_sim_stereo_drone.py` has 8 evals. All failed before the change, because the module
did not exist (ImportError) and the old path had no such behaviour. The FOV, noise, gaps and density,
dark floor, depth timing, colour clock, principal point and dark exposure evals now pass.
`test_sim_camera` and `test_batch` follow the new measured defaults.

The first dark-floor threshold was wrong. Points under 5 cm include wall feet 3-5 cm up, spread by the
depth noise; with the noise off, nothing is left under 3 cm. The eval now asks for fewer than 10 % of
the default's floor-level points.

## NOT measured

- Floor colour and texture of the course, and the plywood, tarp and mesh fill rates. They are
  estimates, tuned only so the overall valid share is about 0.4. The real capture was a bench scene
  (wall and dark floor), not the course.
- Edge holes at depth jumps.
- Whether intervals and ages are correlated (measured r = -0.25; the sim samples them independently).

## Sims

One run per command: severity 0.7, `--duration 420 --camera rgbstereo --ladder-pose original`, default
profile, no lapses. Outputs are in `/tmp/cheese/realdepth_seed80{1,2}` (index `/tmp/cheese/index.html`).

| seed | final | finish_s | total_s | referee checkpoints | touches | pad error (aim) | wall time |
|---|---|---|---|---|---|---|---|
| 801 | DONE, returned to the launch | 77.4 | 101.3 | 0 | 0 | 0.022 m (0.006) | 159 s |
| 802 | DONE, returned to the launch | 75.1 | 100.7 | 0 | 0 | 0.120 m (0.122) | 269 s |

The mission's own log passes every goal, from corridor_1 through the tunnel and the window to the pad.
The referee still scores 0 checkpoints on route gate_b, strict and centre alike. This was not
investigated, and I did not check whether it also happens on main before this change. goalnav was not
tuned.
