# snowcrashers_nav: up 2 m, forward 2 m, land (real skybeta drone)

A team package for the skybeta drones (Jetson Orin Nano + Pixhawk 6C, PX4 1.16), written
against what is actually on **skybeta-03** (skybeta 0.3.3, read 2026-11-07), not against
the simulator. It lives on the drone in `~/team_ws/src/snowcrashers_nav`.

## How the drone really runs team code

```
OAK-D -> cuVSLAM -> px4_odom_bridge -> uXRCE agent -> PX4 EKF2      (the "stack": tmux windows
                                                                     agent, slam, bridge)
our node --/skybeta/setpoint/{pose,twist}--> flight_interface --/fmu/in/*--> PX4
         <--/skybeta/state, /visual_slam/tracking/odometry--     (tmux window "competitor")
```

- **The stack is a docker container** (`skybeta-stack`) started by `host/run.sh stack` or, if
  installed, by `skybeta-stack.service` at boot. Its `competitor` window builds
  `~/team_ws` and runs `ros2 launch $COMPETITOR_PKG $COMPETITOR_LAUNCH` from
  `/etc/skybeta/drone.env`.
- **Our launch file starts `flight_interface` (unchanged) plus our node.** The interface is
  the only thing allowed to talk to PX4 (rules: never publish `/fmu/in/*`). It streams the
  Offboard heartbeat at 10 Hz, converts ENU to NED, floors every pose at 0.3 m and caps twists
  at 1.5 m/s.
- **Nothing on the Jetson can arm.** The safety pilot arms on RC **with the mode switch on
  Altitude**, then **switches to Position, on the ground**. That switch is the go signal: if
  our node has published a setpoint in the last second, PX4 has a valid position and it is
  landed, the interface asks PX4 for Offboard, once per arming (since 0.3.3 addressed to the
  FC's own system ID; 0.3.2 sent it to system 1 and only drone 1 ever took over).
- **PX4 disarms by itself** if the drone has not left the ground 10 s after arming
  (`COM_DISARM_PRFLT`), and 2 s after it detects a landing (`COM_DISARM_LAND`).
- **The pilot can take over at any time** (throttle up / sticks, mode switch, kill switch on
  channel 5). Offboard does not come back until disarm.

## What our node does

WAIT (pose at the current position, 10 Hz) -> Offboard starts -> CLIMB to 2 m at 0.5 m/s ->
SETTLE 1.5 s -> FORWARD 2 m along the nose direction (0.4 m/s, gentle start and stop) ->
ARRIVE (within 0.2 m for 1.5 s) -> DESCEND with poses to the 0.3 m floor -> TOUCHDOWN with a
0.4 m/s downward twist (PX4 1.16 only detects a landing under a commanded descent of at least
1.1 x `LNDMC_Z_VEL_MAX` = 0.275 m/s) -> PX4 detects the landing and disarms -> DONE.

All in `snowcrashers_nav/mission.py` (no ROS, tested against the drone's own takeover rules
in `test/test_mission.py`); `up_forward_land.py` is the ROS wrapper. Parameters:
`height_m`, `forward_m`, `climb_speed_mps`, `forward_speed_mps`, `descent_slow_mps` (launch
arguments).

**Step-by-step setup and flying: [`../../DRONE_SETUP.md`](../../DRONE_SETUP.md)** (on the drone:
`~/team_ws/DRONE_SETUP.md`). Two launch files: `low_test.launch.py` (0.5 m up, 0.5 m forward,
for the first flights) and `competitor.launch.py` (2 m / 2 m). No PX4 parameter changes are
needed or wanted: the organisers ask everyone to keep the fleet defaults.

## Setup (once), in short

1. Code: `~/team_ws/src/snowcrashers_nav` (this folder). The stack builds it.
2. `sudo nano /etc/skybeta/drone.env` and set only these two lines:
   `COMPETITOR_PKG=snowcrashers_nav` and `COMPETITOR_LAUNCH=competitor.launch.py`.
3. The stack must be running. **On skybeta-03 it is not set to start at boot**
   (`host/doctor.sh`: `stack-autostart not installed`). Either start it by hand each power-up
   (`cd ~/skybeta && host/run.sh stack`, needs an interactive terminal) or install the boot
   service: `cd ~/skybeta && sudo host/autostart.sh enable` (what the organisers' test needs:
   "your code starts by itself, with nobody logged in").
4. `host/doctor.sh` must show `uart-traffic PASS`. FAIL = no bytes from the flight controller:
   it is unpowered, the cable is not seated, or (on stock v1.16.2 firmware) the link wedged
   after an earlier stack stop and only an FC power cycle clears it.

## Each flight

1. Power up, **keep the drone still for the first minute** (VIO origin and heading).
2. Check: `docker exec -it skybeta-stack tmux attach -t skybeta`, window 3 (`competitor`)
   shows `up_forward_land ready` and `flight_interface` shows `PX4 link up`. Ctrl-b d to leave.
3. Pilot: mode switch on **Altitude**, throttle low, **arm**, then within a few seconds switch to
   **Position**. The drone climbs at once.
4. Watch the `competitor` window: `[CLIMB] Offboard active`, `[SETTLE]`, `[FORWARD]`,
   `[DESCEND]`, `[TOUCHDOWN]`, `PX4 reports landed`, `[DONE] Disarmed. RESULT {...}`.
5. After landing, flip the mode switch back to Altitude. The next arm + Position flies again.

If the drone stays in Position instead, the `flight_interface` log says why:
`auto offboard: staying out of Offboard: <reason>`.

**Props off first.** Rehearse the whole sequence props-off: arm in Altitude, switch to
Position, and check `[CLIMB] Offboard active` appears (then disarm). Then low and tethered
(`height_m:=0.5 forward_m:=0.5`), then the full mission. A VIO hover has not been flown on
any skybeta airframe yet (organisers' README), so your first flights are the test.
