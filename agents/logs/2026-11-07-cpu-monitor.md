# CPU/frame-rate monitoring tools and a camera survey (2026-11-07)

Branch `agent/cpu-monitor`; tools in `tools/drone_tools/` (README there). Drone lock was taken for the session.

## Built (evals first)
- `cpu_watch.py` (host) and `topic_rates.py` (container), 33 unit evals (mutation-checked: breaking the
  saturation threshold, the stray rule or the stamp parse each fails a test) and `eval_live.sh`.
- Found by running on the real drone, then fixed with a failing eval first: sysfs thermal read raised
  `TypeError` (not OSError) and crashed the monitor; ROS topic discovery is asynchronous (first run
  said "topic not found" for everything); `dockerd` looked like a container; the monitor flagged itself.
- Live eval (pinned burner, core 5): 8/8 PASS on the drone, also re-run on the final code (hashes matched the
  commit) after the drone came back: 33/33 unit + 8/8 live, burner at 0.999 cores, nothing left running.

## Camera survey (skybeta-03, stack up ~10 min, props-free bench)
- OAK-D S2 on USB3 (5000 Mbps), 1024x640, no USB errors. Rectified (median row offset 0.00 px), baseline
  7.5 cm, sharp, exposure fine (3-4 % saturated). IMU 199 Hz. cuVSLAM odometry 29.0 Hz.
- Delivered image rate 24-26 Hz (design 30), worst gap 140-170 ms. `/oak/left/image_raw` (driver output, before
  the mono8 converter) is 23.9 Hz with 40 late gaps in 10 s, the same as image_mono, so the drops happen AT or BEFORE
  the driver's publish, not in our consumers. L/R stamp offset median 0 ms, max 66.7 ms (a pair occasionally
  mismatched by two frames after drops).
- `ros2 topic hz /oak/left/image_raw` reported 18 Hz; the low-overhead probe says 23.9: the CLI tool depresses
  what it measures on 2 MB messages. Use `topic_rates.py`.
- CPU when quiet (37 % of 6 cores): oak_container 0.45, stereo_depth 0.45, cuVSLAM 0.39. No core saturated, no
  single-thread bound, VERDICT: no bottleneck. Earlier ~100 % of a core was a hung command of mine (killed by the user).

## Grayscale (bgr8) issue
The depthai v3 driver publishes the mono sensors as 3-channel bgr8 and exposes no encoding parameter
(`ros2 param list /oak`: only i_low_bandwidth, i_publish_full_resolution, i_resize_mode, ...). The conversion
to mono8 happens in the stack (GPU converter). Our consumers already read `image_mono` (stereo_depth does);
`image_raw` is not faster (see 2026-11-07-depth-bottleneck.md). So it cannot be fixed from team_ws.

## Frame rate: what could raise it (all need organizer-owned config, NOT applied)
Edit `~/skybeta/src/oakd_vslam_bringup/config/oakd_vslam.yaml` (tracked; organizers update it): the file's own
measurements give 960x600@30 as the clean-with-margin point; test it with `topic_rates.py` before/after,
with `cpu_watch.py` alongside. Not tried: it needs a stack restart (shared with the PX4 work) and touches
a file we were told to leave alone.
