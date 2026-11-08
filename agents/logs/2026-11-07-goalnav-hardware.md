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
(`~/team_ws/logs/<date-time>/`), the simulator matched to all of the above, giving up by flying home (blind descents found rocks and
stand feet), and touching down on the launch mark itself (the corner by it is crowded with stand feet).
Simulator, drone-matched inputs, assuming a colour pad detector: 15 of 16 on the batch 9/10 course sets
(one window not found at 4.5 Hz depth); without colour (as on the drone) the pad is never seen and the
mission flies home. Still open: the pad (needs the
colour camera or real mono images of the mat), deployment, and a props-off bench run (camera and cuVSLAM at
30 Hz with stereo_node + recorder running).
