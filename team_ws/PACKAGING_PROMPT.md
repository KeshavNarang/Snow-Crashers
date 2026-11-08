# Prompt: package the course code for the real drone (skybeta-03)

Paste everything below the line into your coding agent, from the Snow-Crashers repo.

---

Package my course-running code (developed against a simulator) as a ROS 2 package in
`team_ws/src/<name>/` of this repo, so it runs on our real drone **skybeta-03**. Do not fly
anything or touch the drone: build a package that can be deployed and tested by the team.

**Read first, and copy their patterns:** `team_ws/src/overwall_nav/` (README, `mission.py`,
`mission_node.py`, `launch/overwall.launch.py`, `config/overwall.yaml`, `test/`) and
`team_ws/DRONE_SETUP.md`. `overwall_nav` has flown on this drone (take off, approach a wall
with stereo depth, climb over, land). The organizers' drone software is backed up in
`jetson-backups/skybeta-03-*/skybeta/`: read `README.md`, `docs/competitor_guide.md`,
`src/skybeta_flight_interface/` and `src/competitor_example/`.

## How code runs on the drone (verified on skybeta-03, skybeta 0.3.3)

- **Runtime:** a Jetson running ROS 2 **Humble** inside the organizers' "stack" container, with
  a Pixhawk running PX4 1.16.2. Team code lives in `~/team_ws/src` on the drone (this repo's
  `team_ws/src`).
- **Start-up:** at power-up the stack's `competitor` window runs `team_build.sh`, which
  colcon-builds the WHOLE `team_ws`, then `ros2 launch $COMPETITOR_PKG $COMPETITOR_LAUNCH`
  from `/etc/skybeta/drone.env`. **If any package in `team_ws/src` fails to build, nothing
  launches.**
- **The launch file:** must start `skybeta_flight_interface`'s `flight_interface` (exactly
  ONE copy) with `auto_offboard_on_arm: true`, plus your node(s). Copy
  `overwall_nav/launch/overwall.launch.py`, which also sets `max_altitude_m: 2.5`: the net top
  is 9.5 ft = 2.9 m.
- **The contract:** your node ONLY publishes to `flight_interface`, which alone talks to PX4:
  - **Publish** `/skybeta/setpoint/pose` (geometry_msgs/PoseStamped): a position to hold.
  - **Publish** `/skybeta/setpoint/twist` (geometry_msgs/TwistStamped): a velocity, in world
    axes. It expires 1 s after the last message and is capped at 1.5 m/s.
  - **Read** `/skybeta/state` (std_msgs/String JSON): `px4_link`, `armed`, `landed`,
    `offboard_active`, `local_enu`, `origin_enu`.
  - **Read** `/visual_slam/tracking/odometry` (nav_msgs/Odometry), from cuVSLAM.
  - Reading `/fmu/out/*` is fine.
  - **NEVER** publish to `/fmu/in/*`, never arm, never change modes, never run a second
    `flight_interface`, never change PX4 parameters (the organizers' rule: modified-parameter
    drones don't fly), never edit `~/skybeta`.
  - In `drone.env`, only `COMPETITOR_PKG` / `COMPETITOR_LAUNCH` are ours. `CAMERA_PITCH=0.4712`
    is already set: the camera is in a 27 deg down holder.
- **Frame:** ENU metres in cuVSLAM's `odom` frame. The origin is where the drone sat when the
  stack started; +x is where its nose pointed then, +y left, +z up. **No simulator truth or
  course map exists on the drone.** Course coordinates must be expressed relative to the
  take-off spot and heading, so state the assumed start pose: where to place the drone and
  which way it faces. For arrival checks use `local_enu` from `/skybeta/state`: PX4's own
  estimate, which is what PX4 steers to.
- **Takeover:** the pilot arms with the mode switch on Altitude, then switches to Position on
  the ground within ~3 s. That triggers Offboard ONCE per arming, and only if:
  - your node published a setpoint in the last second (publish a pose at the current position
    at 10 Hz while waiting),
  - PX4 reports landed,
  - PX4 has a valid position.

  When `offboard_active` goes true, start the mission. PX4 disarms if the drone hasn't taken
  off 10 s after arming, so climb immediately. If `offboard_active` goes false mid-flight, the
  pilot has taken over: stop the mission, publish nothing more, and wait for disarm.
- **Clamps:** every pose is clamped to 0.3-2.5 m above the vision origin, so even "hold here"
  lifts the drone to 0.3 m. On this drone PX4's height read 0.3-0.45 m below the vision
  height on the ground, so compare targets after that clamp (`overwall_nav` `clamp_z`).
- **Heading:** ALWAYS command a yaw. With an all-zero quaternion, PX4 1.16 holds no heading
  (PositionControl.cpp sets the yaw target to the current yaw every loop), and our drone
  yawed ~30 deg on takeoff. Hold PX4's heading at Offboard start:
  `yaw_enu = pi/2 - heading_ned`, from `/fmu/out/vehicle_local_position`, topic resolved with
  `skybeta_flight_interface.topics.resolve_endpoints()`. Put it in the pose quaternion (see
  `overwall_nav/mission_node.py`).
- **Landing:** there is no land command and poses can't go below 0.3 m. Descend with poses to
  0.3 m, then publish twists straight down at >= 0.3 m/s (we use 0.4) until PX4 disarms.
  PX4's land detector needs a commanded descent of >= 0.275 m/s, and it auto-disarms 2 s after
  detecting the landing. Move targets gradually (`MPC_XY_VEL_MAX` is 12 m/s: a pose 2 m away
  flies at ~2 m/s).
- **Sensors:** the drone publishes NO depth topic. For obstacles, reuse
  `overwall_nav`'s `stereo_depth` node (`/overwall/front`) and `perception.py`; don't write a
  second image pipeline. The camera is pitched 27 deg down, so it sees nothing above its own
  height. Jetson CPU is tight (depth ~100 ms/frame, cuVSLAM must stay ~25-30 Hz): no heavy
  Python per-frame work.
- **Reliability:**
  - Restarting the stack can freeze the PX4 link (PX4 1.16.2 bug), so apply changes with a
    battery power cycle.
  - The stack's logs vanish on reboot, so log clear one-line events and a final `RESULT` line.

## What to build

1. **The package** `team_ws/src/<name>/` (ament_python): `package.xml`, `setup.py`,
   `setup.cfg`, `resource/<name>`, `launch/<name>.launch.py`, `config/<name>.yaml` (every
   tunable: speeds, waypoints, heights, a forward/area backstop) installed via
   `setup.py data_files`, and `test/`.
2. **Structure:**
   - Pure-Python mission logic with no ROS imports, like `overwall_nav/mission.py`:
     `step(now, state, odom, ...) -> ('pose', xyz) | ('twist', xyz) | None`.
   - A thin ROS node around it.
   - Strip anything simulator-specific: ground truth, sim services, arming, direct PX4
     commands, mode requests.
3. **Safety in the logic:**
   - Height cap below the net (2.2 m, net 9.5 ft with 0.4 m clearance).
   - Horizontal backstop.
   - Stale-data handling: no position, or no depth, means stop advancing, then land.
   - One go per arming; never fly an Offboard you didn't see start.
4. **Tests:** reuse the model in `overwall_nav/test/test_mission.py`. It imports the drone's
   real `AutoOffboard` and `SafetyEnvelope` from `jetson-backups`, and models PX4, the pilot
   sequence, the PX4/vision height offset and VIO drift. Cover at least:
   - the full course,
   - pilot takeover,
   - no setpoints before arming,
   - lost position / lost depth,
   - landing detection.
5. **Check it builds exactly as the drone will.** The whole workspace must build. If you can
   reach the drone (team approval needed), use a sealed container that can't touch the flight
   hardware:

   ```bash
   docker run --rm --network none -v ~/team_ws:/root/team_ws --entrypoint /bin/bash skybeta:0.3.3 \
     -c "team_build.sh && source skybeta_env.sh && ros2 launch <name> <name>.launch.py --show-args"
   ```

   Then run the tests inside it.
6. **A README:** what it does, the assumed start pose, the settings, how to deploy, and its
   limits.

Deploy, for the team to do:
1. `scp -r team_ws/src/<name> dronehacks@skybeta-03.local:team_ws/src/`
2. On the drone: `~/team_ws/run_mission.sh <name> <name>.launch.py`
3. Battery out 10 s and back in. Wait ~3 min, drone still.
4. Run the sensor check:
   `docker exec skybeta-stack bash -c "source skybeta_env.sh; ros2 run overwall_nav sensor_check"`
5. Rehearse with the props off.
6. First flight low and short.

Report back:
- the package name and launch file,
- the assumed start pose,
- what you changed from the sim code and why,
- the test results,
- anything you could not verify.
