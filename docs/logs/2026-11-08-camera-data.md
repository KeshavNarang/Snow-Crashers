# Better data out of the front camera (2026-11-08)

Start 12:46 PST. Branch `agent/camera-data`. skybeta-03 over USB (192.168.55.1), props off,
FC unpowered (PX4 link down throughout, so no fusion numbers). Drone lock held. Jetson reboots
were allowed by the user. Mission on the drone: `drone_ref / goalnav.launch.py` (RGBSTEREO).

## Baseline (goalnav stack running, 19-27 min after boot)

`tools/fly bench`-style numbers: Jetson depth 4.0 Hz, colour 8.0 Hz (5.3 Hz in a Python
subscriber), cuVSLAM 30 Hz, vision age at the PX4 bridge median 162 ms, p90 337 ms.

Per stage (`topic_rates.py`, age = receive - header stamp):

| topic | age median / p95 / max (ms) |
|---|---|
| /oak/left/image_raw (driver out, bgr8) | 35 / 56 / 165 |
| /oak/left/image_mono (GPU converter out) | 140 / 346 / 369 |
| /visual_slam/tracking/odometry | 257 / 362 / 433 |
| /goalnav/stereo/points | 296 / 425 / 492 |

The frame is 35 ms old leaving the driver and 140-350 ms old leaving the bgr8 -> mono8 converter.
The converters' input QoS is the stock RELIABLE depth 10 (`input_qos_depth` 0 = default): 35 ms +
10 x 33 ms = 365 ms, the observed ceiling. A once-a-second logger (`tools/camera_data/agemon.py`,
one long-lived process) showed odometry age swinging between ~60 ms and 200-380 ms every few
seconds; depth points carry the same lag (stereo_node reads the converters' output).

- SIGSTOP of stereo_node for 13 s: odometry 59 ms median (p95 99), and it stayed at 59 / 74 ms for
  a while after SIGCONT, with depth still 4 Hz (age 102 ms). A 10 s all-core CPU burn did not
  reproduce the lag; it came back by itself (and after bursts of `ros2` CLI calls).

## Finding: cron spins a core after boot (clock at 1970)

The Jetson has no time source offline: the clock starts at 1970-01-01. `cron` used 8 min 40 s of CPU
in its first ~16 min (top: 99 % of a core); after `systemctl start cron` it spun ~5 s, then idled.
Not yet shown to be the cause of the known "first 5 minutes are slow" warm-up.

## Do not

- `ros2 component unload` a NITROS converter from `vslam_container`: the container segfaulted
  (exit -11) and took cuVSLAM with it; the launch does not restart it. Needs a reboot.

## Fix 1: converter queue depth 1 (commit 7b7a13a, team image built and running on skybeta-03)

`team_ws/Dockerfile` adds `'input_qos_depth': 1` to both converters in the stock launch (sed + grep,
the build fails if the stock line changes). Jetson rebooted into it; goalnav stack running.

| goalnav stack, RGBSTEREO | before (19-27 min after boot) | after (2-5 min after boot) |
|---|---|---|
| vision age at the PX4 bridge, `tools/fly bench` 30 s | median 162 ms, p90 337 ms | **median 59 ms, p90 63 ms** (695 samples) |
| odometry age, per-second medians | swinging 60-380 ms | 58-71 ms over 100 s, worst single message 98 ms |
| /goalnav/stereo/points age | median 245-296 ms | 85-110 ms |

The ros2-CLI bursts of the bench script no longer bring the lag back. PX4 fusion was not measured
(FC unpowered).

**Cost: dropped frames instead of late ones.** cuVSLAM's own gap warnings (`/tmp/gapwin.sh`-style
count over 30 s windows): 6 % of frames missing with stereo_node at 4 Hz, 10 % at 10 Hz, 16 % with
an in-flight load stand-in added (the stock image on 2026-11-07, no goalnav: 1.8 %). Likely worse
because the left and right converters drop different frames. Not measured: drops before the fix
(frames arrived late, but their count was not taken).

## Depth rate now that the queue is gone (manual stereo_node restarts; launch still 4 Hz)

| stereo_node | delivered | points age median | odometry age | CPU idle | stereo_node CPU |
|---|---|---|---|---|---|
| 4 Hz (launch) | 4.0 Hz | ~100 ms | 59-62 ms | ~50 % | 40 % |
| 8 Hz | 8 Hz | ~95 ms | 59-61 ms | 59 % | 61 % |
| 10 Hz + in-flight load stand-in | 9.6 Hz | 81-96 ms | 59-63 ms | 53 % | 68 % |
| 15 Hz, 3 workers | 14-15 Hz | ~82 ms | 59-61 ms | 56 % | 91 % |

In-flight load stand-in (`flightload.py`, not committed): every cloud into a goalnav `LiveMap` plus
the colour subscription. goalnav's `LiveMap.insert` on real 6000-point clouds: median 30 ms (41 ms
under load), so 10 Hz takes ~30-40 % of goalnav's single thread. Launch rate NOT changed yet: decide
after the frame-drop question (above) is settled.

## Fix 2 (in progress, not committed): GRAY8 from the camera

The driver hardcodes NV12 for every camera (`camera.cpp:45`, depthai-ros 3.3.0); the host converts
it to bgr8 (`ImageConverter.cpp:169`), 118 MB/s of bgr8 crosses processes to the converters, which
turn it back into mono8. GRAY8 is copied straight to a mono8 message by the bridge. Work tree
`../SC-camera`: `team_ws/camera/depthai_gray8.patch` (adds `i_output_type`), `fetch_driver.sh`
(laptop side; the drone has no internet), Dockerfile build step, `oakd_vslam.yaml` left/right
`i_output_type: GRAY8`. Open: whether the NITROS converter accepts mono8 input (else remap cuVSLAM
to `/oak/{side}/image_raw` and drop the converters).

**13:20: the Jetson dropped off USB** while the patched driver compiled inside the running
container (`colcon build`, default make parallelism, stack running). Not reachable afterwards
(no DHCP lease on the laptop's USB interface). Cause unknown (out of memory, or power). The
Dockerfile step now uses `MAKEFLAGS=-j2`; better to build with the stack stopped.
