# 2026-11-08: goalnav on RGBSTEREO (the team camera config), drone and simulator

The team camera config (`team_ws/camera/oakd_vslam.yaml`) runs RGBSTEREO: colour + the mono pair,
**no on-camera depth** (on-camera depth cost PX4 its VIO fusion on skybeta-03; `team_ws/preflight.sh`).
goalnav on main read only the on-camera depth image (`/oak/stereo/image_raw`), so on this config it
would have had no wall map, and it found the pad in the mono image.

## Changes
- `goalnav/launch/goalnav.launch.py`: `camera:=rgbstereo` (default) starts `stereo_node` (depth from the
  mono pair, the existing node) and gives goalnav `points_topic=/goalnav/stereo/points`, no depth image,
  and the colour image for the pad. `camera:=rgbd` keeps the on-camera depth image. Table: `CAMERA_MODES`.
- `goalnav/localsim/goalrun.py`: `--camera rgbstereo` (default) publishes stereo_node's points at the
  Jetson's measured ~4.5 Hz / ~0.13 s (the model from before 9deefe8, restored) plus mono and colour
  images; `--camera rgbd` is the on-camera depth model as it was. Goalnav topics per mode in `SIM_CAMERAS`.
- `tools/fly deploy` builds the team camera image when the drone's stack doesn't run RGBSTEREO;
  `tools/fly check` runs the organizers' `host/doctor.sh`, then `preflight.sh`, then goalnav's status.
- `goalnav/localsim/batch.py`: reads the launch's current argument table for the default profile
  (it only understood the older format, so `test_batch.py` failed on main); records `camera`.

## Evals
- `test_launch.py`: the launch defaults to rgbstereo with those topics (the old test asserted *no*
  stereo_node, which encoded the rgbd design). `test_sim_camera.py`: the simulator hands goalnav the
  launch file's topics in each mode, at 4.5 Hz / 0.13 s by default. Both failed first.
- goalnav suite + layout tests: 59 passed (after the batch.py fix).

## Sim (seeds 1-20, severity 0.7, `--camera rgbstereo`, main as of 4d6842a)
18/20 land on the pad with no contact before landing, 47-70 s. Failing: seed 12 (window frame,
t 42 s), seed 19 (after the tunnel, (2.25, -0.72)). **Not yet run on the drone.**
