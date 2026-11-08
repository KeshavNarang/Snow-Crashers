# goalnav entered like overwall_nav, deployed and run on skybeta-03 (2026-11-07 ~20:20, branch agent/goalnav-fly)

Goal: James's goalnav (`drone_ref`) runs the way Luke's `overwall_nav` does. No tuning, no sim work.

## Change
- `goalnav/launch/goalnav.launch.py` now mirrors `overwall_nav/launch/overwall.launch.py`: the same `INTERFACE`
  table and the same `flight_interface` Node (2.5 m ceiling, 0.3 m floor, 1.5 m/s cap, 10 Hz, Offboard on
  arm then Position, cuVSLAM odometry), always started. Then `goalnav_node`, plus the logger (bag, ros logs,
  run.json in `~/team_ws/logs/<date-time>/`). The CPU `stereo_node` is not started (the camera computes depth).
- Eval first: `goalnav/test/test_launch.py` (static, no ROS). It failed on the old launch (flight_interface
  only got `auto_offboard_on_arm`), passes now.

## On the drone (lock held, stack not restarted, drone.env not changed)
- `~/team_ws/src/drone_ref` replaced by this branch's `goalnav/`; the old copy is in `~/drone_ref.prev-20261107`.
  **Not under ~/team_ws:** colcon scans all of team_ws, and a second `drone_ref` there fails the (boot) build
  with "Duplicate package names".
- `team_build.sh --packages-select drone_ref`: OK. `colcon list`: drone_ref, overwall_nav, snowcrashers_nav.
- `ros2 launch drone_ref goalnav.launch.py record:=false` on an isolated domain (77): flight_interface up with
  "altitude 0.30-2.50 m, speed <= 1.50 m/s", "Offboard on the switch to Position after arming: ON"; goalnav_node
  up with profile `safe`.
- `goalnav_node` alone on the live domain, twist and status remapped to `/goalnav_test/*` (no second
  controller): twist 48 Hz (max gap 0.33 s), status IDLE. **No depth reached the map** because the Pixhawk link
  was down (`/skybeta/state` px4_link false, no `/fmu/out/*`): goalnav needs PX4 pose to place depth.
- Real depth frame through goalnav's own `depth_image_m` + `depth_points`: 16UC1 320x200, 19 % valid,
  median 0.47 m, 250 points (tile 8). Camera depth and camera_info at ~25 Hz.

## To fly it
1. Pixhawk power cycle (link was down), wait for `PX4 link up`.
2. `~/team_ws/run_mission.sh drone_ref goalnav.launch.py` (sudo), battery out 10 s, in, wait ~3 min.
   Back to Luke's: `~/team_ws/run_mission.sh overwall_nav overwall.launch.py`.
3. Bench, props off: `ros2 topic echo /goalnav/status` shows `floor_z` and the "mount from TF" line in the log.
4. Staged first flight: set `tuning` (launch arg) e.g. `{stop_after: corridor_1}`. Pilot: Altitude, arm, Position.

## Open
- Sim (seed 3, mono, rebuilt maze) aborted at `first_corridor` (0 checkpoints); not investigated (out of scope).
- Pad needs colour (camera config has none); mission gives up and descends at the end.
- Ctrl-C prints an rclpy "already shutdown" traceback from goalnav_node: cosmetic.
