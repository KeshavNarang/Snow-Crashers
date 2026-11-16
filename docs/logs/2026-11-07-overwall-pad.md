# overwall_nav: see the pad past the wall and land on it (2026-11-07, 20:10-20:30, laptop only)

Branch `agent/overwall-pad`. No drone used; nothing deployed.

## Why it never landed on the pad
`mission.py` landed at a fixed `land_beyond_wall_m` (1.88 m) past the wall and had no pad input at all.
The stack streams only the mono pair, where the cyan mat is within ~10 % of the orange tarp's
brightness, so colour is needed (docs/CAMERAS.md: RGB approved and required for the pad).

## Geometry (why it drops down before looking)
Colour camera (IMX378, factory K: fx 1468.8 at 1920 wide) at 640x400: vertical field +-22 deg, on
the 27 deg holder -> floor seen from ~0.87 x camera height ahead. Crossing the 1.6 m test wall the
camera is ~2.0 m up: the floor nearer than ~1.75 m is below the image, and while still before the
wall its top hides the floor out to ~2.5 m past it. With the pad 1.88 m past the wall the whole mat
is never in view from the crossing height. From 0.8 m camera height, 0.55 m past the wall face, it is.

## What changed
- `pad.py`: goalnav's colour pad detector (chromaticity histogram of the real mat, floor projection,
  size check), height-free: candidates are per metre of camera height; `pick_pad` scales them with
  the mission's camera height and refuses blobs cut off by the frame (their centre is ~0.1-0.2 m off).
- `pad_detect` node: `/oak/rgb/image_raw` + camera_info (undistorted) -> `/overwall/pad`, 5 Hz,
  publishes every frame (empty list too) so the mission knows it runs. 8 ms/frame on a laptop.
- `mission.py`: with `/overwall/pad` fresh at the crossing, OVER ends 0.55 m past the wall, LOOK
  descends to camera 0.8 m, SWEEP turns +-35 deg once, TO_PAD flies over the mat (two sightings
  within 0.25 m, past the wall), then the usual ARRIVE/DESCEND/TOUCHDOWN on its centre. Not seen:
  land 1.88 m past the wall as before. **No pad feed: exactly the old behaviour.**

## Evals (6 new; suite 78 tests, 32 s)
- test_pad.py (rendered views of the reference mat on the tarp colour, real colour K): finds the
  mat from 0.8 m within 8 cm; refuses the cut-off view from 2.0 m; bare tarp -> nothing.
- test_mission.py (World gained a colour-camera pad model and a heading that follows hold_yaw):
  pad 0.35 m long and 0.35 m left of the set point -> lands 0.03 m from its centre (old code: 0.50 m);
  pad 1.1 m left -> found by the sweep, lands on it; feed but no pad -> sweep, lands at 1.88 m.

## NOT done / to do on the drone
1. Colour is not on in the stack. Needs the team camera image (`agent/goalnav-oak-depth`:
   team_ws/Dockerfile + camera/oakd_vslam.yaml) with `i_pipeline_type: rgbd`, measured there at
   cuVSLAM ~22 Hz. Organizer-image change, rebuild and power-cycle: user approval first.
2. Bench, props off: `ros2 run overwall_nav pad_detect` with the mat 1-2 m in front of the drone;
   confirm the topic name, encoding, the "biggest ... sides ~0.85/height" log line, and the
   rendered-tarp threshold on the real tarp/carpet under the hall's light.
3. Pitch: the node uses camera_pitch_deg 27 (same holder); docs/CAMERAS.md says to use the colour
   image's own TF. Check the colour optical frame's TF on the drone.
4. The VIO-tilt shift (README: ~0.45 m along the heading per metre of height) would move the
   touchdown when descending 0.66 m onto the pad, if it is still present after CAMERA_PITCH was set.

## On the drone, 2026-11-07 evening (claude, with the team at the net)
- Deployed main's overwall_nav to ~/team_ws/src. **Gotcha:** a backup in ~/team_ws/parked/ broke the
  power-up build ("Duplicate package names"): colcon scans all of team_ws. ~/team_ws/parked has a COLCON_IGNORE now.
- Colour via James's team image (oakd_vslam.yaml with `depth`, switched to `rgbd`): colour came up
  (8.9 Hz, K fx 489.6 as the tests assumed), but **PX4 never fused VIO** (estimator cs_ev_pos false,
  xy_valid false, dead reckoning), with `rgbd` AND with James's plain `depth`. cuVSLAM logged frame
  deltas of 66 ms; one vision message reached PX4 290 ms old.
- Stock organizers' image (Dockerfile parked): PX4 fused VIO (cs_ev_pos, cs_ev_yaw), xy_valid true.
- Ours: stock oakd_vslam.yaml + `i_pipeline_type: RGBSTEREO` + an rgb block 640x400@10 (team_ws/camera/,
  team_ws/Dockerfile). PX4 fused VIO with colour on, cuVSLAM 28 Hz, no frame-delta warnings.
  Stock notes: the OAK-D USB link saturates ~143 MB/s; the pair is 118 MB/s; on-camera depth is what tipped it.
- Arming refused once: battery_status 7.98 V on a 4-cell setting (warning 3, 0 %): wrong pack on the main lead.
  With the flight pack: 15.86 V, 81 %.
- Flight 1 (ours, colour on, ~2 min after boot): climbed, saw the wall 1.15 m ahead, crept the 0.25 m
  backstop and landed in front (drone was ~1.2 m from the wall, settings assume 27 in).
- Flight 2: reported hitting the wall in front. **No log**: powered off, the container's screen log is gone.
  Same approach/climb code as Luke's (mission changes only start after the climb; depth changes since
  were output-identical speed-ups and the floor-fit prior). Differences from Luke's flight: colour + pad_detect
  on the CPU, flown ~2 min after boot, start spot unknown. Depth measured 0.2-2.5 Hz in that warm-up
  vs 4.3 Hz on the stock image. Fix applied: runs are now logged to ~/team_ws/logs/overwall/<date>/.
