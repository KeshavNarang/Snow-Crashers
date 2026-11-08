# goalnav camera calibration check, and two sims with drone-like latency (2026-11-07)

Branch `agent/goalnav-calib`. Read-only look at skybeta-03 (`/tf_static`, `/oak/*/camera_info`,
the driver's `i_undistorted` parameter) while the stack ran overwall_nav; no lock needed.

## Calibration: the stereo matcher is fine

- `camera_info` (1024x640): per-camera factory intrinsics (left fx 637.9, right 640.3, the EEPROM
  values x0.8), distortion all zeros, R identity. The zeros are correct: `left.i_undistorted` and
  `right.i_undistorted` are True, so the driver undistorts on the camera. (Had the images been raw,
  zero distortion would have cost ~2.3 px of disparity at the 90th percentile at 1024x640.)
- TF right -> left: 75.35 mm, with the factory rotation (~0.8 deg), not a nominal 0.075 m.
  `stereo_node` builds its rig from these, so its rectification uses the real calibration.
- HFOV 77.5 deg and baseline 7.5 cm in the simulator match the drone (77.5, 7.54).

## Calibration: the mount was not (fixed)

TF chain base_link -> oak_left_camera_optical_frame: offset (0.106, 0.038, 0.031) m, axis
**27.6 deg down, 0.7 deg yaw left, 0.5 deg roll**. `goalnav_node` assumed 27.0 deg, no yaw or roll:
0.9 deg off, ~6 cm at 4 m, in every wall, window and tape cell it maps. It also placed a colour pad
camera at the left camera's mount (the colour camera is 3.8 cm to its right).

Fix (`fix(goalnav): place each camera from the TF`): each image uses the TF from `base_frame` to its own `frame_id`; the parameters are the
fallback when there is no TF (the simulator). The fallback pitch is now 27.6: the simulator renders at
27.6 but goalnav ran with the 27.0 default, a mismatch every earlier sim run carried.

## Two sims with drone-like latency (item 6, after the fix)

`localsim.goalrun --severity 1.0 --camera-hz 4.5 --camera-latency 0.4 --assume-colour-pad` (latency
0.4 s as overwall's depth measures, vs the 0.13 s default). New seeds, so not tuned on.

| seed | extra | result | finish | pad error | return | touches |
|---|---|---|---|---|---|---|
| 211 | - | DONE, window route, 5 CPs | 76.8 s | 2.9 cm | 5.9 cm | 0 |
| 212 | odometry drift x2 (0.016 m/sqrt s, 0.003 rad/sqrt s) | DONE, window route, 5 CPs | 72.9 s | 13 cm | 3.1 cm | 0 |

0.4 s old depth did not break either run; doubled drift costs pad accuracy (13 cm from the centre).
Two runs are a smoke test, not a pass rate. Not modelled: cuVSLAM losing track, glare holes beyond
oakd_model's per-surface rates, and the colour camera itself (the run assumes it is enabled).
