# Handoff: goalnav on the OAK-D's own depth (branch `agent/goalnav-oak-depth`)

Written 2026-11-07 ~20:10 by the goalnav-drone session. The organizers approved changing the camera pipeline.
The team uses on-camera depth and does **no depth estimation of its own**: the CPU `stereo_node` is out of the launch.

## State
- **On the drone now:** the team image `skybeta-team:0.3.3`, built from `~/team_ws/Dockerfile`.
  - It installs `~/team_ws/camera/oakd_vslam.yaml` (same as `team_ws/camera/` here): `depth` pipeline, the pair at
    1024x640@30 for cuVSLAM, depth 320x200 at ~28 Hz in the left optical frame.
  - It is used on every boot. Measurements are in `2026-11-07-goalnav-hardware.md` (fourth session).
  - `drone.env` still runs `overwall_nav`, which does its own CPU stereo (1.4 cores; not ours).
- **Code on this branch,** merged with `origin/agent/sim-walls` for the rebuilt maze/ladder geometry:
  - **`goalnav/drone_ref/goalnav_node.py`:**
    - Map path: `depth_topic` `/oak/stereo/image_raw` (+ camera_info) → `geometry.depth_points` (tile 8) → map.
    - Tape/pad image: `image_topic` defaults to `/oak/left/image_mono`, because the drone runs no colour camera.
    - A colour image is undistorted from its CameraInfo and also feeds the pad.
    - Camera mounts come from the TF (already on main).
  - **`goalnav/localsim/goalrun.py`:**
    - Publishes 16UC1 depth (10 Hz, 0.1 s old, tile dropout) and the mono image.
    - `--colour` publishes the colour camera instead, so the pad can be found.
    - `--assume-colour-pad` is gone.
  - **`goalnav/launch/goalnav.launch.py`:** no stereo_node; records `/oak/stereo/*` and `/oak/rgb/*`.

## Not done / not verified
1. **No sim run since these changes** (the check was cut for time). Do this first:
   - Run `python3 -m localsim.goalrun --seed 3 --no-viewer`, then again with `--colour`.
   - Needs numpy, scipy, pyyaml, opencv and trimesh (e.g. a nix-shell with `python3.withPackages`).
   - Expect: mono never finds the pad (gives up and descends); colour finishes.
   - The sim-walls merge also changed the course, so rerun a small batch as well.
2. **Not deployed:** goalnav with these changes has never run on the drone against `/oak/stereo/image_raw`. Check:
   - the "mount from TF" log line;
   - that points land on the floor plane (floor_z);
   - that walls show up where expected.
3. **Depth minimum range is ~0.32 m;** the bottom rows had no returns on the bench. Options (measure each once):
   - `stereo.i_extended_disp: true` (halves the minimum range, may cost rate);
   - `i_depth_preset: HIGH_DETAIL`, or no decimation (640x400).
   - Note: `FAST_DENSITY` crashes the v3 driver.
4. **The pad needs colour,** and colour costs cuVSLAM ~6 Hz (see the log). Options:
   - (a) accept `rgbd` for the run;
   - (b) find the mat in mono (in brightness alone it is within ~10 % of the floor: hard);
   - (c) find the pad by geometry (the three rocks around it in the depth map);
   - (d) feed cuVSLAM the stereo node's rectified mono pair instead of the bgr8 pair, then retry colour. This frees
     link bandwidth but changes cuVSLAM's input, so it needs user approval.
5. **Tape in mono is untested on the drone.** `/oak/left/image_mono` is 1024x640 at ~20 Hz into Python, which costs
   CPU. A lighter option: enable `stereo.i_left_rect_publish_topic` (mono8, smaller) and point `image_topic` at it.
6. **Use the depth-camera branch's tools.** Another session's branch `agent/depth-camera` has:
   - evals: VIO non-regression (A) and depth correctness (B);
   - a swap tool that changes the driver without restarting the stack.

   Judge this config with its `acceptance.judge()`.

## Rules learned
- Restarting the stack can wedge the flight controller's link (CLAUDE.md). It survived ~8 restarts today, but ask
  the user first.
- No long sleeps in commands: poll instead.
- Release the drone lock (`tools/drone-lock release`) when done.
- Undo the team image: `rm ~/team_ws/Dockerfile` (or `docker rmi skybeta-team:0.3.3`), then a restart or power cycle.
