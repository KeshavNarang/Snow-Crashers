# Depth bottleneck on skybeta-03 (2026-11-07, read-only + 2 short ground experiments)

Live pipeline = `team_ws/src/overwall_nav` (commit 2a7d5ac "Successful run going over wall"; its README
still says "not flown"). `goalnav/` has never run on hardware and needs a dense depth image the drone
does not produce.

## Measured on the drone (stack running overwall.launch.py, props-free bench, no flight)
| Item | Value |
|---|---|
| /oak/left/image_mono | 20.5 Hz (design 30) |
| cuVSLAM odometry | 27.8-28.7 Hz |
| /overwall/front (our depth) | 2.8-3.2 Hz (target 5) |
| stereo_depth CPU | ~275 % of 6 cores |
| node proc_ms / age_ms | 75 ms / 413 ms |
| raw /oak/left/image_raw age | ~45 ms; the converted image_mono ~160 ms (max 370) |
| CPU SGBM alone | ~30-40 ms; points() ~65-80 ms; point generation ~14 ms |
Driver exposes only /oak/{left,right}/{image_raw,image_mono,camera_info} + imu. No depth, no RGB.
Container: OpenCV 4.5.4 with NO CUDA, no VPI, no isaac_ros stereo/ESS; only CPU stereo_image_proc.

## Conclusions
- Bottleneck is CPU: SGBM + Python on the same 6 cores as cuVSLAM, plus queueing -> ~0.4 s old, ~3 Hz depth.
- On-camera StereoDepth would remove the CPU cost but needs `oakd_vslam.yaml`, which lives in the
  organizers' ~/skybeta checkout (rules: leave it as it is). Not available to us unless organizers agree.
- Tried: reading /oak/*/image_raw instead of image_mono (parameters left_image/right_image): WORSE
  (proc 222-293 ms, age 274-524 ms; 3x data, run alongside the live node). Do not switch.
- Laptop-only parameter benches (nd/block/rows/scale) were too noisy on the loaded Jetson to rank; scale 0.4
  and fewer rows looked ~2x faster but need accuracy checks (test_stereo.py) before any change.

## Next (needs a decision)
1. Ask organizers whether the camera config may enable on-device StereoDepth (best fix).
2. Else reduce SGBM cost inside overwall_nav (scale/rows/disparities) and validate with test_stereo.py + a bench A/B.
No code on the drone or in the repo was changed.

## Correction after a second, steady-state measurement (17:21, drone up 11 min)
The first numbers (3 Hz, age 413 ms, proc 75 ms) were taken 1-2 min after boot, while the stack was still
warming up; they overstate the problem. Steady state, live node: **4.5 Hz, proc ~63 ms, age ~127 ms**;
cuVSLAM odometry **30.8 Hz** (design point), camera 19-20 Hz. Depth is slow but works at the mission's 0.2 m/s.

Candidate `scale: 0.4` (config only): passes all 38 tests (10 stereo + 28 mission/checks, laptop; the baseline
also passes), and cropping the top rows is rejected (fails 2 stereo tests). But on the drone, side by side
with the live node: proc 52 vs 63 ms (-17 %), rate 4.4 vs 4.5 Hz, points 12.6k vs 19.6k. Too small a gain for
less range resolution, so NOT applied. `config/overwall.yaml` is unchanged.

Real limits remain for a faster course run (needs a dense depth image at >= 10 Hz): on-camera depth
(organizers) or a GPU stereo path. For the current slow over-the-wall mission, no depth fix is needed.
