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
- **CORRECTION (see "Frame loss, corrected" below):** plain-topic Python subscribers see 24-26 Hz, but cuVSLAM (the nav-critical
  consumer) sees 29.4 Hz effective. The earlier conclusion here, "the drops happen at or before the driver", was wrong: a plain
  subscriber receiving fewer frames does not show the camera dropped them.
- L/R stamp offset median 0 ms, max 66.7 ms (a pair occasionally mismatched after drops).
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

## Frame loss, corrected (2026-11-07 18:2x, drone up ~20 min)
- cuVSLAM logs `Delta between current and previous frame [X ms] is above threshold [40 ms]`. Parsing 8.5 min of its log: 212 warnings,
  296 frames missing of 15299 expected = 1.9 %, effective 29.4 Hz at its input. Gap sizes in frame periods: 2:158, 3:36, 4:10, 5:3,
  6:2, 15:1 (one 0.5 s gap). Gaps are exact multiples of 33.3 ms, so whole frames are missing at the source. Odometry holds 29-30 Hz.
- Bursty: warnings per minute [19, 35, 8, 5, 35, 26, 0, 42, 42]. Also 3x `Unknown Tracker Error 2`.
- Rejected hypothesis: images falling back from shared memory to UDP loopback with a 208 KB kernel receive buffer. Measured: only
  0.4 MB/s on `lo`, 0 receive-buffer errors in 6 s (the 3806 counted since boot were a one-off), and no socket with drops.
  Raising `net.core.rmem_max` would have done nothing.
- No driver thread above 25 % CPU, so the driver is not CPU-bound. USB: SuperSpeed 5000 Mbps, power/control=on.
- The 24-26 Hz seen by plain Python subscribers is a consumer-side effect and irrelevant to the pipeline: cuVSLAM is the
  consumer that matters (29.4 Hz) and stereo_depth only uses 5 Hz.

## Do the frame-loss bursts line up with CPU load? No (slam_gaps.py --cpu-log, 5 min at 1 s resolution)
- Over 14.4 min of cuVSLAM log: 359 gap warnings, 472 of 25873 frames missing = 1.82 %, effective 29.5 Hz. Steady, not
  worsening (per minute: 19 35 8 5 35 26 0 42 42 19 29 21 10 60 8). Gap sizes: 2 periods 288, 3: 48, 4: 16, 5: 4, 6: 2, 15: 1;
  worst 500 ms.
- 83 seconds with a gap vs 198 without: CPU 51 % vs 55 %, run queue 5.1 vs 6.1, the same busiest process (stereo_depth) in both.
  Verdict: no relationship with CPU load at 1 s resolution. (Run queue is ~5-6 on 6 cores throughout, so the machine is
  busy but not more so during gaps; sub-second spikes would not show at this resolution.)
- So the remaining ~1.8 % loss is not explained by CPU utilisation, DDS transport (see above) or a saturated driver thread.
  Left: USB/XLink transfer hiccups, device-side timing, or scheduling latency (not utilisation). The stack's own launch
  file notes the driver stamps 30 fps at 32.2-33.6 ms; neither can be changed from team_ws.
- Not tested yet: auto-exposure behaviour (e.g. bright hall into the dark tunnel), which is a feed-quality question, not a rate one.
