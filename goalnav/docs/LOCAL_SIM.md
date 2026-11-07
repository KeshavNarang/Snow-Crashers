# Local simulator

A portable simulator of the real drone, so changes can be tested in about a
minute without lab credit. It runs our **real** `mission_node` and
`vision_node` code with `config/sim.yaml` plus the `config/race.yaml` overlay
(the planned minimum-time race, [RACE.md](RACE.md)) against:

- a fake ROS 2 layer (`localsim/fakeros.py`) using rclpy's real value types
  (bytes diagnostic levels, NumPy CameraInfo, `array('B')` image data);
- the Sky Beta under PX4 1.16 (`localsim/vehicle.py`): PX4's offboard position
  and velocity control law at default gains (`MPC_XY_P` 0.95,
  `MPC_XY_VEL_P/I/D_ACC` 1.8/0.4/0.2 with D on the measured acceleration,
  `MPC_Z_VEL_P/I_ACC` 4/2, `MC_YAW_P` 2.8),
  with acceleration and yaw-rate feedforward and PX4's velocity limits; the
  thrust-vector attitude loop (`MC_ROLL_P`/`MC_PITCH_P` 6.5, 220 deg/s body-rate
  limit, collective thrust sized for the *desired* tilt, so a lagging tilt
  lifts the aircraft), the 45 degree tilt limit, `MPC_THR_MIN`, thrust about
  4.5x weight (1.30 kg, X2807 1500KV, 7 inch props), drag and rough
  aerodynamics (`localsim/aero.py`); PX4's takeoff handling
  (`localsim/takeoff.py`: `COM_SPOOLUP_TIME`, the `MPC_TKO_RAMP_T` ramp and
  `MPC_TILTMAX_LND`, as tuned in `config/px4_race.params`); Offboard
  handshake, arming, force disarm with free fall, the drop speed and a slide to
  rest, offboard-loss failsafe, AUTO_LAND with position hold, auto-disarm; an
  EV-fused local position with drift, noise and the eph/epv the lab reports;
  the lab's `_v1` topic names;
- the published CAD course (`localsim/world.py`): 640x400 16UC1 depth from the
  OAK-D at 60 Hz with 30 ms latency (`--world hardware`, default) or the lab's
  15 Hz preset (`--world lab`); wall contacts; an optional dark tunnel;
- an independent referee (`localsim/referee.py`) using true pose, the drawn
  checkpoint lines (whole-footprint and centre judges), touches, crashes
  (including a hard drop after a disarm in the air), CP5 where the aircraft
  rests disarmed on the floor, with the pad touchdown and disarm times, the
  return (land and disarm within 2 ft of the launch mark inside 3 minutes)
  and the rulebook score on the end-to-end time, as the judge's stopwatch
  takes it (leaving the launch mark to the return's disarm). Its clock starts
  when the body truly leaves the pad, not at PX4's land detector, which
  clears earlier.

It does not model rendering, cuVSLAM or DDS, and aerodynamics only roughly.
`--px4` swaps the Python control model for real PX4 SITL on a rigid body
(below). The thrust
figure comes from vendor claims, and the intrinsics (80 degree HFOV) and camera
mount (0.10 m ahead of `base_link`) are estimates until measured.
`--vehicle FIELD=VALUE` changes any `VehicleConfig` value for sensitivity runs.

## Run it

Python 3.10+ with NumPy and PyYAML, plus SciPy for the wall aerodynamics
(`pip install -r localsim/requirements.txt`).
From `reference_stack/`:

```bash
python3 -m localsim.run                     # one nominal run with the return, about 25 s
python3 -m localsim.suite                   # all scenarios in parallel
python3 -m localsim.run --vehicle attitude_p=5 --set drone_ref_mission.race_attitude_lead=0.12 --set drone_ref_mission.race_turn_lead=0.14
python3 -m localsim.run --set drone_ref_mission.race_plan_file=race_plan_safe.json   # another rung
python3 -m localsim.run --params config/sim.yaml --params config/pursuit.yaml   # depth-guarded controller
```

The first run voxelizes the course (about 15 s) into `localsim/.cache/`.
Each run writes `localsim/out/<name>/report.json`, `trace.json`, `run.svg`
(top-down map: plan dashed, flown path coloured by speed, touches circled) and
`replay.html`, an animated replay to open in any browser
(`python3 -m localsim.replay <dir>` rebuilds it; `--compact` makes it smaller).
`run` exits 0 only when the referee sees a finished run.
`python3 -m localsim.bound <run dir> [--plan <plan>]` prints lower bounds on the run time.

## Current result

`python3 -m localsim.suite` (`sim.yaml` + `race.yaml`, window route, default
plan `race_plan.json` at 5 m/s^2, the PX4-checked model with the rough
aerodynamics on): nominal and seeds 1-2 touch down on the pad 7.62-7.70 s
after liftoff, disarm on the floor 10-260 ms later, re-arm, and are back on
the launch mark (0.24-0.30 m from it) 12.39-12.69 s after liftoff, end to
end, with no touches: 75 run points if fastest. The suite prints the pad
touchdown (`pad s`), the liftoff for the return (`off s`) and the end-to-end
time (`total s`). It holds with the lab's 15 Hz camera, a dark tunnel, 3x
thrust, 2x body drag, 3x estimator noise, pad-heading errors and every
launch-alignment placement. It fails, as before, with a PX4 attitude gain
25 % off until the leads are re-fitted (`localsim.tune_lead`), with the
rough aerodynamics off or doubled until `race_drag_linear` matches the rotor
drag, and with PX4's stock takeoff. The ladder (seeds 0-2, end to end): slow
14.96-15.21 s, safe 12.57-12.84 s, fastest 12.10-12.36 s, max 11.54-11.70 s, all clean. In real PX4 SITL (below)
the slow, safe and default plans flew 4 of 4 seeds clean. Details, bounds
and the hardware checklist: [RACE.md](RACE.md).

`python3 -m localsim.bound <run dir>` prints the run against its bounds
(default plan): 2.70 s (straight lines at the tilt limit, ignoring walls),
5.05 s (the race line to the touchdown at the tilt limit), 7.08 s (at the
plan's 5 m/s^2), 7.33 s (plus the jerk limit and vertical motion), 7.67 s
(plus lifting the skids, the settle and the sink onto the pad), the planned
return (3.50 s), and the flown times: pad touchdown 7.62 s (PX4 SITL
7.79 s), end to end 12.41 s (PX4 SITL 12.91 s).

The depth-guarded pursuit controller (`config/pursuit.yaml`) took 29.2 s on
the earlier airframe model, whose attitude responded in 0.06 s. With PX4's
default attitude loop it overshoots the CP1 corner stop, the camera comes
within its minimum depth of the net and the run aborts; it completes
(29.9 s) only with that faster attitude (`--vehicle attitude_p=16 --vehicle
max_tilt_rate=10`). It has not been re-run on the corrected model; re-tune it
before relying on it. The slow race plan (`race_plan_slow.json`, 9.9 s) is
the current fallback.

## Rough aerodynamics

`localsim/aero.py`, shared by both vehicle models; each effect has its own
zero-able `VehicleConfig` gain and `aero_gain` scales them all (suite rows
`aero_off`, `aero_x2`). Sources and assumptions are in the module docstring.

| Effect | Default | Model |
|---|---|---|
| Ground effect | `ground_effect` 1, cap 1.2 | Cheeseman-Bennett 1/(1-(R/4z)^2), R 0.089 m, rotor plane 3 cm above the body origin: +5 % on the pad |
| Rotor drag | `rotor_drag_per_s` 0.25 | horizontal drag across the rotor discs, linear in speed and proportional to thrust |
| Wall pull | `wall_suction` 0.03, `wall_range_m` 0.18 | up to 3 % of thrust toward a wall within a rotor diameter of the prop tips (0.24 m reach); distances from a distance transform cached in `localsim/.cache/` (nets ignored, needs SciPy) |
| Tunnel | `tunnel_thrust_change` -0.03, `tunnel_turbulence` 2 | thrust change and stronger turbulence inside the tube |
| Turbulence | `turbulence_accel` 0.15 m/s^2, 0.3 s | seeded Gauss-Markov gusts, stronger near walls |
| Disarmed drop | `ground_friction` 0.5 | keeps its horizontal speed and slides to rest (v^2/(2 mu g): 2.5 cm from 0.5 m/s); the race no longer disarms in the air |

Cost: about 40 us per flying step, under 1 % of a run (depth rendering
dominates). Two effects matter for the race. Rotor drag: the follower now
feeds forward linear rotor drag (`race_drag_linear`, 0.25 1/s at hover
thrust, as here) besides the quadratic `race_drag`; without it the aircraft
ran about 15 cm wide into the CP1 corner, and a 50 % mismatch either way
still fails the default plan, so both terms must be fitted from flight logs.
Turbulence near the walls: in PX4 SITL it pushed the aircraft 0.14 m wide in
the maze hairpin (clean with `turbulence_accel=0`) until the follower got its
cross-track feedback (`race_cross_track_gain`).

## Real PX4 SITL (`--px4`)

Real PX4 1.16.2 (EKF2 on external vision only, the position, attitude and rate
controllers, takeoff handling, land detector, Offboard, force disarm) in
lockstep with a six-DOF quad (`localsim/rigidbody.py`: 1.30 kg, motors
0.15 m out, 30 ms motor lag, 4.5x weight, estimated inertia, the rough
aerodynamics), with our real nodes on the fake ROS layer. `localsim/px4/`
holds the airframe (quad X at PX4's default gains; EV-only EKF2, no GPS or
compass, no RC, simulator only), the launcher, the MAVLink links and the
bridge, which plays the uXRCE-DDS client: TrajectorySetpoint becomes
SET_POSITION_TARGET_LOCAL_NED, VehicleCommand becomes COMMAND_LONG, and the
telemetry comes back as the `_v1` topics. The cuVSLAM stand-in sends ODOMETRY
from truth at 30 Hz with 40 ms latency, noise and drift. `dist_bottom` is
synthesized as in the Python model.

Build once (about 5 minutes; the clone and build live in
`~/.cache/snowcrashers/PX4-Autopilot`, override with `PX4_DIR`), then run from
`reference_stack/`:

```bash
nix-shell localsim/px4/shell.nix --run localsim/px4/build.sh
nix-shell localsim/px4/shell.nix --run 'python3 -m localsim.run --px4 --px4-params config/px4_race.params'
nix-shell localsim/px4/shell.nix --run 'python3 -m localsim.px4.probe --px4-params config/px4_race.params'
nix-shell localsim/px4/shell.nix --run 'python3 -m localsim.tune_lead --px4 --px4-params config/px4_race.params'
```

A run takes 12-30 s, as in the Python model, and also writes `px4.ulg`
(control chain at full rate) and `px4_console.log`. `--px4-instance N`
moves the ports (4560+N, 14540+N); `tune_lead --px4` gives each lead its own
instance. `localsim.px4.probe` flies the same scripted takeoff and velocity
steps through PX4 and the Python model and prints both.
`localsim.px4.probe` flies the same scripted takeoff and velocity steps
through PX4 and the Python model and prints both. The validation below ran
four PX4 instances at a time on a 16-core, 14 GB laptop.

**Takeoff (measured, times from arming).** Stock: spool-up to +1.0 s; the
ramp's upward velocity limit starts at -g/`MPC_Z_VEL_P_ACC` (zero thrust),
so liftoff comes at +2.85-3.0 s. The tilt limit stays at `MPC_TILTMAX_LND`
(12 degrees) until the ramp ends (+4.0 s), then slews to 45 degrees at a
hard-coded 0.2 rad/s, reaching it at +6.9 s. With `config/px4_race.params`
(`MPC_TKO_RAMP_T` 0.1, `MPC_TILTMAX_LND` 45): liftoff at +1.17 s, 0.3 m up at
+1.6 s, and full tilt from the start. `MPC_TKO_RAMP_T` 0 lifts off just as
fast but toggles zero thrust once while the land detector catches up. The
Python model reproduces both cases to within about 0.1 s (PX4 later, from
motor lag), and the planner now plans exactly this (RACE.md).

**Against the Python model.** Thrust-vector lag on velocity steps: PX4
tau 0.130 s plus 0.045 s delay; the model 0.130 s plus 0.035 s. The model was
corrected to PX4's code (D term on the measured acceleration, integrator and
anti-windup, `MPC_ACC_DECOUPLE`, takeoff) and its rate loop fitted to SITL
(40 ms, torque-authority limit from motor headroom). Results with
`px4_race.params` (seeds 0-3, the max plan 0-7), end to end from liftoff to
the return's disarm on the launch mark:

| Plan | Python model, aero on (seeds 0-2) | PX4 SITL | PX4 SITL pad touchdown |
|---|---|---|---|
| slow, 3 m/s^2 | 3/3, 14.96-15.21 s | 4/4, 15.44-15.51 s | 10.24-10.27 s |
| safe, 4.5 m/s^2 | 3/3, 12.57-12.84 s | 4/4, 13.16-13.22 s | 7.97-8.06 s |
| default, 5 m/s^2 | 3/3, 12.39-12.69 s | 4/4, 12.91-13.04 s | 7.79-7.82 s |
| fastest, 5.25 m/s^2 | 3/3, 12.10-12.36 s | 3/4, 12.68-12.93 s (seed 3: CP1 hairpin wall) | 7.55-7.60 s |
| max, 7 m/s^2 | 3/3, 11.54-11.70 s | 1/8, 12.29 s (else a maze hairpin wall) | 7.03 s |

Every PX4 run that reached the pad also completed the return, 0.22-0.29 m
from the mark, 1.47-1.56 s after the pad disarm. Before this revision (in-air
disarm, the IMU artefacts below) the max rung flew 3/8 and fastest 4/5, and a
run took about 22 s end to end.

`tune_lead --px4` (default plan): clean from 0.07 to 0.11 s, against
0.03-0.09 s in the Python model. PX4 still lags more than the model in turns
and saturates motors (at zero in up to 15 % of samples) in the hairpin tilt
reversals; 6 m/s^2 and up fails there in most seeds. What got the plans
through, in order: holding the heading (the old plan yawed at up to 3 rad/s
and the yaw torque used up the motor headroom), feeding forward the rotor
drag and PX4's velocity D term (it acts on the measured acceleration and
took a fifth off every planned one), a separate lead for the turning
acceleration (`race_turn_lead`) and the cross-track feedback.

**On the floor (fixed IMU artefacts).** The rigid floor stops the body
within one 2 ms substep, and the rigid body used to send PX4 the specific
force and body rate of the last substep only, not their means over the 4 ms
IMU sample. An impact in the first substep never reached EKF2: its height
ran away below the floor (about 1 m), the aircraft bounced back up while
still armed (PX4 pushed up against the runaway estimate), and after the old
in-air disarm PX4 refused to arm for 5-7 s ("vertical velocity unstable",
"height estimate not stable"). The levelling on the legs at contact also
turned the body without any gyro signal, leaving EKF2 a degree or two off
and its velocity drifting 0.2 m/s on the pad, which held the return for
about 2 s. `localsim/rigidbody.py` now integrates both over the IMU sample,
as a real IMU's delta velocity and delta angle do. A real airframe's landing
gear softens the impact further; how a real IMU and EKF2 take the 0.5 m/s
touchdown is untested. MAVLink ODOMETRY carries only the sum of EKF2's
reset counters, so the bridge infers which state reset from jumps in the
estimate (`ResetCounters`).

**Limits.** The airframe's inertia, motor lag and thrust curve are estimates,
and the organizers' tune is unknown, so PX4 SITL tests PX4's code and default
gains, not the real aircraft. PX4 SITL is not deterministic run to run: the
same seed can finish once and fail the next time.

## Problems it found (and the fixes)

1. **Depth metadata rejected under real rclpy.** `DiagnosticStatus.level` is
   `bytes`; the parser required `int`, so no depth would ever authorize motion.
2. **Takeoff aborts.** On the pad the lab reports epv 0.093 m and an invalid
   rangefinder, and the camera sees floor nearer than minimum depth, which the
   guard treated as an obstacle inside the body. Fixed with
   `max_vertical_error_landed`, `require_range_consistency`, a floor band and
   a bottom-of-image coverage exemption.
3. **Vertical speed leaked into the horizontal guard** through tilt, reading
   as backward motion outside the camera view (fixed: horizontal velocity only).
4. **Velocity noise at rest** (a few mm/s, random direction) failed the camera
   cone check: `measured_speed_deadband`.
5. **1.0 m corridors read as blocked.** Body radius plus sampling support
   exceeded half the corridor, so side walls were "ahead". The guard now
   ignores depth on published course surfaces (`drone_ref/map_filter.py`).
6. **Command steps tilted the drone past 15 degrees**; the guard's camera cone
   shrinks with rotation. The pursuit controller bounds the gap between command
   and measured velocity, keeps the nose within `yaw_lead` of travel and turns
   in place below `free_turn_speed`.
7. **Gate B is only a few centimetres wide after inflation**; the window route
   is wider and worth 5 points.
8. **PX4's default velocity gain (1.8/s) turns the velocity vector slowly**, so
   nose-steered pursuit drifted wide in turns. `position_tracking` sends PX4
   position, velocity and acceleration setpoints along the path, with a
   cross-track correction and tangent yaw.
9. **PX4's velocity integrator winds up during hard braking** and pushes the
   drone sideways afterwards. Turns in place now hold the stop point actively,
   slow motion below `measured_speed_deadband` is not direction-gated, and a
   brief out-of-view motion brakes first (`measured_fov_grace`).
10. **Boulders below the path counted as walls** in the camera-band map. The
    contact aborts (`abort_margin`) use a separate map of the body's own
    height band, against real surfaces rather than the inflated margin.

11. **A lagging tilt lifts the aircraft.** PX4 sizes thrust for the desired
    tilt; in hard turns the body tilts less, so it climbs (to 1.06 m from
    0.9 m). The race follower adds stiffer height feedback.
12. **Inconsistent references track badly.** Position, velocity and
    acceleration computed separately disagreed by up to 0.9 m/s^2; the race
    reference now differentiates one smoothed position signal.
13. **The return needs a settled height reference.** On the pad after the
    run the estimate had drifted 2 cm up, so the return follower saw a
    liftoff during spool-up; it now uses the resting estimate on the pad.


14. **The return never flew after the drop (PX4 SITL).** EKF2's velocity
    and height resets after the impact counted as a frame change and aborted
    the mission, and a rejected re-arm aborted it too. The return now waits
    for PX4's arming checks and a resting estimate, retries ARM, and accepts
    velocity and height resets on the pad (see above).
15. **Race decisions waited for depth.** With `race_require_depth: false` the
    dispatch check still required a fresh depth pairing; a late frame
    aborted the race (`vo_noise_x3`).
16. **The suite flew `stock_takeoff` with the tuned ramp**: the suite-wide
    takeoff default overrode the scenario's own values.
17. **The IMU aliased floor impacts (PX4 SITL).** See "On the floor" above:
    the specific force and rate are now means over each IMU sample.
18. **Disarming in the air was the wrong objective.** The judge times the run
    end to end, including the return, so the race now lands and disarms on
    the pad and the return is planned for minimum time (RACE.md).

## Known sensitivities

- **Leads.** The race plans need `race_attitude_lead` and `race_turn_lead`
  within about 0.02 s of the airframe's thrust-vector lag. Fit them from a
  ULog (`tools/fit_attitude_lag.py`) and `localsim.tune_lead`.
- **Drag.** `race_drag_linear` within about 50 % of the real rotor drag.
- **Pad heading.** Map yaw is taken from the declared pad heading, corrected
  by the launch alignment ([LAUNCH_ALIGN.md](LAUNCH_ALIGN.md); `place_*` and
  `raw_*` suite scenarios, `--launch-yaw-deg`, `--launch-offset`). Without it
  3 degrees of placement error already fails the race plan; the pursuit
  controller stopped at 2.
- **Aggressiveness.** In PX4 SITL 5 m/s^2 is the fastest budget clean in
  every seed tried; 7 m/s^2 finished 3 of 8.
- Everything here is tuned against these models. On hardware, work up the
  plan ladder from `race_plan_slow.json`.
