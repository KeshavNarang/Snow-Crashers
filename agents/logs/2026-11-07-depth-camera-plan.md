# Depth from the OAK-D S2 itself: plan (2026-11-07)

Branch `agent/depth-camera`. Goal: on-device stereo depth (the Myriad X `StereoDepth` engine) published as a ROS
topic, **without degrading the visual odometry PX4 flies on**. Evals are written before any code; every
claim below is from the driver source at the installed version or from a measurement, not assumed.

## What the source says (depthai-ros `v3.3.0-humble` = installed `ros-humble-depthai-ros-driver-v3 3.3.0`)

- The stack runs `pipeline_gen.i_pipeline_type: stereo`: two camera nodes, **no `StereoDepth` node**, so no depth.
  The type string is matched case-insensitively (`pipeline_generator.cpp`). `DEPTH` and `RGBD` create a
  `dai_nodes::Stereo`, i.e. on-device depth.
- Luxonis' own config for Isaac cuVSLAM (`config/isaac_vslam.yaml`) uses `i_pipeline_type: depth`.
- In the `Depth` pipeline the stereo node builds its own left/right camera wrappers (node names `left`/`right`,
  so the existing `left.*`/`right.*` parameters still apply; their publish default is false, but the stack's
  YAML sets `i_publish_topic: true`). So `/oak/{left,right}/image_raw`, which cuVSLAM's chain consumes,
  **should keep publishing unchanged**. Must be verified, not trusted.
- Depth goes to `~/stereo/image_raw` (16-bit). Optional rectified left/right come out as **GRAY8 (mono8)** on
  `~/{left,right}/image_rect`: a possible later fix for the bgr8 problem.
- The raw camera outputs cross the camera link as **NV12** (`camera.cpp`: `Type::NV12`), 1.5 bytes/px, and the
  host converts every frame to bgr8. The "143 MB/s XLink ceiling" in the stack's notes was computed at 3 bytes/px,
  so the real link headroom is **unknown**: measure it.
- `img_pub.cpp`: every enabled output is transferred and converted for every frame; the lazy-publisher flag only
  skips the final ROS publish. Only disabling an output saves bandwidth or CPU.
- Stereo defaults that would silently give wrong geometry here (`stereo_param_handler.cpp`):
  `i_aligned: true` with `i_board_socket_id` = CAM_A (the RGB camera, not running in this pipeline), so the depth
  `camera_info`/frame would describe RGB; stereo size 640x400 while the rectified outputs' `camera_info` is sized
  from `left/right.i_width/i_height` (1024x640). Both must be set explicitly and checked by eval.
- Minimum range with 96 disparities: ~0.32 m at 640x400 (fx ~399 px, baseline 7.5 cm), ~0.50 m at 1024x640;
  `i_extended_disp` halves it. Maze walls are tens of centimetres away, so this matters.
- `driver.launch.py` hosts the driver as one component (`depthai_ros_driver::Driver`, node `oak`) in
  `oak_container`; neither it nor `oakd_vslam.launch.py` has an on-exit shutdown handler.

## Safety constraints that shape the method

- `host/stack_service.sh` restarts the whole stack if the `slam` pane dies. That restarts the XRCE agent, which on
  stock PX4 v1.16.2 wedges the FC's TELEM2 until a power cycle (skybeta CLAUDE.md bug #15). **So nothing may
  kill the `slam` launch.** Method: unload only the `Driver` component from `oak_container` and load the candidate
  into the same container (or reload the original), so no process exits.
- Leave `~/skybeta` (organizers' checkout) untouched. The candidate config = the live driver's own parameter dump
  plus a small overlay, so nothing VIO-critical is retyped by hand.
- Drone lock held for every live step; props off, disarmed, bench only. A swap interrupts VIO for some seconds and
  may reset cuVSLAM's origin: power-cycle before any flight.
- Another agent (goalnav-drone) is testing the colour camera, which also needs a pipeline change. `RGBD` can serve
  both; agree before either is made persistent.

## Evals (written before the code; pass/fail decided in advance)

A. VIO must not regress (candidate vs a baseline taken minutes before, same scene):
   1. left/right frames by header stamp (from `camera_info`, one per frame, cheap): rate >= 29.5 Hz, missing
      frames <= baseline + 0.5 percentage points, left/right stamps paired >= 99 %.
   2. cuVSLAM: gap warnings/min <= 1.5 x baseline + 2; odometry >= 29 Hz; `vo_state` = 1 on >= 99 % of messages;
      static drift over 60 s <= max(2 cm, 2 x baseline).
   3. left/right `camera_info` (size, K, D, R, P, frame) identical to baseline; TF base_link -> both optical frames
      within 1 mm / 0.1 deg.
   4. CPU (cpu_watch): no saturated core, total +<= 0.5 core.
B. Depth must be real and right:
   1. depth topic present, 16UC1 in mm, size equal to its `camera_info`, >= 25 Hz by header stamp, age p95 <= 100 ms.
   2. frame consistency: depth matches a host stereo match (SGBM) computed from the same instant's left/right images
      in the frame its `camera_info` claims (median relative error <= 3 %, best horizontal shift 0 px).
   3. floor check when the floor dominates: plane pitch within 2 deg of the mount pitch (~27 deg).
   4. coverage: valid fraction >= the host SGBM's on the same frames.
C. Restore: after reloading the original driver config, all of A returns to baseline tolerance.

Offline unit evals cover every analysis function (depth decoding, plane fit, depth comparison and shift search,
stamp-gap and pairing analysis, camera_info/TF diffs, the parameter overlay and its invariants, the swap tool's
plan/dry-run), with known-answer synthetic data and mutation checks.

## Stages

1. Baseline on the organizers' config (A metrics), read-only.
2. Candidate: `Depth` pipeline, raw left/right exactly as today, depth at 640x400 aligned explicitly; run A + B;
   restore (C). Then tune (size, preset, extended disparity, subpixel, filters) one change at a time, each through A + B.
3. Only if 2 cannot meet B without breaking A: feed cuVSLAM the on-device rectified mono8 pair (drops the bgr8
   conversion and frees bandwidth). That changes the VIO input, so it needs its own, stronger evals and your approval.
4. Persistence at boot: decide with the user/organizers (a supported override in the stack; a guarded one-shot swap
   from our launch with automatic revert; or manual per run). Nothing is made persistent without approval.

## Status 19:05
- Offline work pushed: plan, 89 evals (all mutation-checked), the guarded swap tool, health check, acceptance
  contract, run summary. **Nothing has run on the drone yet**: the lock was held by other agents all along.
- 19:04 the lock was taken by `claude/goalnav-drone (James)` for "on-camera depth: oak driver pipeline at runtime
  (no file edits)", i.e. the same goal. Not started here to avoid two agents swapping the same driver. Points that
  agent may not have, from the source reading above: a dead slam pane restarts the agent (FC wedge on stock
  v1.16.2); stereo.i_aligned defaults to the absent RGB camera; rect outputs take camera_info size from left/right;
  minimum range ~0.50 m at 1024x640 with 96 disparities. Its result can be judged with acceptance.judge().
