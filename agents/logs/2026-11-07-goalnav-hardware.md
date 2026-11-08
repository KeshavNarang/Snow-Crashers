# 2026-11-07 goalnav vs skybeta-03 (read-only, over USB-C, 17:10-17:35)

Connected over the Jetson's USB-C (`192.168.55.1`), `hostname` = `skybeta-03` checked first.
Nothing on the drone was changed; two probe scripts copied to /tmp were deleted after.
Stack skybeta 0.3.3, autostart enabled; `drone.env` ran `overwall_nav/overwall.launch.py`.
The Pixhawk was powered partway through (`PX4 link up: nav_state=4 (AUTO_LOITER)`: RC off).

## What holds
| Item | Measured |
|---|---|
| `/fmu/out/vehicle_local_position` | the topic name, **no `_v1`** (goalnav listened on `_v1` until 590eac3) |
| `/fmu/out/vehicle_status_v1`, `vehicle_attitude`, `vehicle_land_detected` | present |
| local position / attitude rate | ~95 Hz / ~75 Hz |
| `/oak/{left,right}/camera_info` (header stamps) | **30.0 Hz, no gap over 33 ms** |
| `/visual_slam/tracking/odometry` | 30.0 Hz, `vo_state` 1 throughout |
| EKF2 | `cs_ev_pos`, `cs_ev_yaw`, `cs_rng_hgt`, `cs_baro_hgt` true; eph 0.04 |
| left camera, TF base_link -> oak_left_camera_optical_frame | (0.105, 0.038, 0.030) m, 27.6 deg down; 1024x640, fx 638 |
| stack container Python | numpy 1.26.4, pyyaml, scipy 1.15.3 |
| `flight_interface` on the drone vs `skybetav1/` in the repo | only `flight_interface.py` differs: 0.3.3 addresses the Offboard request to PX4's own system ID (drone 3); repo had 0.3.2 |

## What goalnav could not have used
- **No depth, no colour.** `pipeline_gen.i_pipeline_type: stereo`: only `/oak/{left,right}/image_{raw,mono}`
  (raw is bgr8 of a mono sensor), camera_info and imu.
- **No `/fmu/out/distance_sensor`** (PX4 v1.16.2's default dds_topics.yaml has only `/fmu/in/distance_sensor`).
  The laser's reading is in `vehicle_local_position.dist_bottom` (0.095 m on the ground, not valid: under its minimum).
- **Height disagrees at rest:** odom_enu z 0.65 m vs PX4 local z 0.21 m (horizontal agrees to 1 mm).
- goalnav (`drone_ref`) not in `team_ws/src` (only overwall_nav, snowcrashers_nav).

## A measurement I got wrong first
A Python probe subscribing to both full `image_raw` streams (2 MB each) reported the camera at 15-20 Hz
with gaps to 400 ms. The same probe on `camera_info` alone: 30.0 Hz, no gaps. The camera is fine; a Python
subscriber to the full images loses frames (and adds load). Hence goalnav's single `stereo_node`.

## Done since (branch agent/goalnav-drone)
stereo_node (overwall_nav's StereoRig -> points + rectified left image), goalnav fed by it, tape found in
mono, rangefinder from `dist_bottom`, camera mount from the TF, the logger in goalnav.launch.py
(`~/team_ws/logs/<date-time>/`), the simulator matched to all of the above, and touching down on the launch mark itself (the corner by it is crowded with stand feet).
Simulator, drone-matched inputs, assuming a colour pad detector: 15 of 16 on the batch 9/10 course sets
(one window not found at 4.5 Hz depth); without colour (as on the drone) the pad is never seen and the
mission gives up (descends where it is: team decision, the fly-home variant was reverted). Still open: the pad (needs the
colour camera or real mono images of the mat), deployment, and a props-off bench run (camera and cuVSLAM at
30 Hz with stereo_node + recorder running).

## Second read-only session (18:20, drone up 7 min, Pixhawk powered, overwall.launch.py running)
Nothing written or started on the drone (scripts piped over stdin). teammates' `topic_rates.py` and `cpu_watch.py`:

| Topic | Rate | Notes |
|---|---|---|
| `/oak/left/camera_info` | 30.5 Hz | one per camera frame: **the camera itself runs at 30** |
| `/oak/{left,right}/image_raw` | 21 Hz | worst gap 253 ms: the driver drops ~1/3 of the images at publish |
| `/oak/left/image_mono` | 19.8 Hz | age median 144 ms, max 374 ms |
| `/visual_slam/tracking/odometry` | 27 Hz | age median 232 ms |
| `/overwall/front` (CPU stereo) | 2.7 Hz | |
| `/fmu/out/vehicle_local_position` | 99.7 Hz | dist_bottom 0.103 m on the ground, not valid (under its minimum) |

**Correction to the first session:** "camera at 30 Hz, no gaps" was `camera_info`; the images are ~21 Hz.
`cpu_watch`: **CPU OVER-COMMITTED**, 59 % of 6 cores, run queue 7.9; overwall_nav `stereo_depth` 1.67 cores
(peak 2.08), camera driver 0.45, cuVSLAM 0.42. goalnav's `stereo_node` uses the same matcher: same load.

- **The colour camera is there:** the driver's calibration TF (from the device's EEPROM) has
  `oak_rgb_camera_frame` / `oak_rgb_camera_optical_frame`, with the left and right cameras placed from it.
  The `stereo` pipeline never starts it; capturing from it needs the pipeline changed (stack restart).
- PX4 `system_id` 3 (flight_interface 0.3.3 needed, as installed); `pre_flight_checks_pass` false (RC off).
- odom vs PX4 height at rest: -0.09 vs 0.14 m (0.23 m apart this boot).
- goalnav's needs in the container: camera frames `oak_{left,right}_camera_optical_frame`, distortion
  rational_polynomial all zeros, TF left->right 0.075 m; cv2 4.5.4, cv_bridge, message_filters,
  sensor_msgs_py; `overwall_nav.stereo` importable with team_ws sourced; rosbag2_storage_mcap and
  skybeta_flight_interface installed. overwall.launch.py runs flight_interface with max_altitude_m 2.5.

Read with a0fa7cb (depth-bottleneck log, same evening): once warmed up (~5 min) the machine was 45 % idle and
the cameras ~28 Hz; overwall's stereo_depth is bound by its single Python thread (receive + sync of ~28
pairs/s + match), topping out at 3.4-4 Hz. goalnav's stereo_node has the same structure, so expect <= ~4 Hz
depth on the drone (the simulator assumes 4.5). A threaded or C++ node, or on-camera depth, would lift both.

## Third session: colour camera, on-camera depth, goalnav deployed (18:44-19:10, props off, lock taken)
Stack stopped for the camera tests (camera free), then restarted. Scripts and results in
`agents/logs/colourtest-20261107-184424/` (also in ~/team_ws/logs/ and ~/team_ws/tools/ on the drone).

- **Colour camera:** depthai reports CAM_A = IMX378 COLOR, CAM_B/C = OV9282. Factory calibration: colour 3.78 cm
  from the left camera, 3.76 cm from the right (centred). Frames saved (`colour_*.png`): sharp, a bit dark.
- **It does not slow the pair:** pair alone 29.8/29.8 Hz; + colour 640x400@30: 29.5/29.5 + 29.5 Hz;
  + colour 1280x720@15: 29.5/29.5 + 14.9 Hz.
- **On-camera depth** (StereoDepth, DEFAULT preset) + colour 640x400@15: depth 640x400@30 -> 29.4 Hz (21 % valid on
  the bench scene); 1024x640 -> 21.5 Hz; 1280x800 -> 15.9 Hz. CPU stereo on the Jetson: 2.8-4.0 Hz.
- Both need the stack's camera pipeline changed: `oakd_vslam.yaml` is in the organizers' image and ~/skybeta
  checkout (README: leave tracked files as they are), and the launch takes no config override. Organizer decision.

goalnav deployed to ~/team_ws/src/drone_ref, drone.env set to drone_ref/goalnav.launch.py, stack restarted (built
fine, PX4 link came back on its own), then **restored to overwall_nav**:
- logger: `~/team_ws/logs/<UTC date-time>/` with bag/ (MCAP, split at 2 GB), ros/, run.json; ~35 MB/s (12 GB
  in ~6 min). Files owned by root (container).
- goalnav stereo_node single-threaded: 2.8 Hz, age 281 ms; with receive and match apart (2 worker threads):
  **4.0 Hz, age 234 ms**, CPU 43 % of 6 cores, cuVSLAM 28.7 Hz. The input image_mono reaches Python at ~21 Hz and
  120-370 ms old already: the remaining limit.
- After the last restart (back to overwall_nav) the FC link did NOT come back (px4_link false, agent no
  datawriters): needs a Pixhawk power cycle. cuVSLAM odom read (-11.6, -1.4) on a still drone.
