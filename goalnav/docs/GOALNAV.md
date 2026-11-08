# Goal navigation on a live map, and the photo-matched course simulator

> Design/history reference. For current flight evidence, enabled cameras, adapter branches and competition guidance, read [the central docs](../../docs/README.md). Sensor descriptions and old rule references below are not a live inventory.

The race stack ([RACE.md](RACE.md)) replays a plan computed from the CAD. The
course as built is not the CAD: walls are feet off, freestanding and can move
between runs, the net is nearly invisible and everything is tarped. This
stack does not trust the CAD for positions at all. It uses it for the order of
the course and the rough offsets between its parts, and finds everything else
with its own sensors during the run. Nothing is stored between runs (rule 6.4).

## How it flies

```
depth (OAK-D) ──► live map ──► goal estimates ──► A* path ──► speed limit ──► follower ──► twist ──► flight_interface
PX4 local pos ──►   (odom frame, 10 cm height bands)                 ▲
VL53L1X ──────► floor height (gated) ──────────────────────────────┘ height hold
```

- **One frame: odometry.** PX4's local frame starts where the drone was
  placed. There is no course origin or registration step, and the launch
  heading is not trusted either: the course's heading is learned from the
  walls seen in the maze (`goals.dominant_heading`), then frozen at Gate A.
- **Live map** (`drone_ref/goalnav/livemap.py`). Per XY cell (5 cm), a log-odds
  value per 10 cm height band. Depth returns add evidence of occupancy, and the
  rays before them add evidence of free space. Hits outweigh misses, so moving
  tarps stay solid and single outliers fade. Returns within 6 cm of the floor
  are the floor. The invisible net gets two fences:
  - Returns from beyond the expected net (the room) add no obstacle, but their
    rays still clear the space they crossed.
  - The last 10 cm inside the net is never cleared, so the few returns from the
    net strands build up when the drone is close.
- **The route as a goal chain** (`config/goals.yaml`, `goals.py`). About 20
  pass points (corridors, wall ends, gates, tunnel, window, pad). Each is first
  expected at the previous goal's estimate plus its CAD offset, then looked
  for in the map: the point of most clearance in a small window across the
  direction of travel. That finds a corridor's middle, a gate between wall
  ends, the inside of the tunnel, or a hole in a wall. When one goal is found
  off, all the later goals shift with it.
  - An `opening` (the window) also has its wall line fitted from the solid band
    above it. That squares the approach, sets the pass height, and allows a
    tight zone (body radius + 1 cm) on the line through its centre.
  - A `fallback` branch (Gate B) takes over if the window is not reached in
    10 s, but only while the drone is still on the approach side.
- **Planning** (`planner.py`). A* at 10 cm cells:
  - walls are impassable;
  - cells within the body radius + 3 cm are escape-only;
  - cells within 0.6 m cost more, so paths keep to the middle;
  - unknown space is allowed but costs extra, i.e. planning is optimistic.
  Each leg between goals is shortcut on its own, so every goal stays on the path.
- **Speed** is the minimum of:
  - the observed free distance ahead (`braking.speed_for_clearance`), so the
    drone never outruns what it has seen;
  - braking ahead of each corner, not in it;
  - a per-kind cap: tunnel 0.6, window 0.35, gaps 0.65 m/s, held flat within
    0.8 m of an opening;
  - clearance here.
- **Following** (`control.py`): pure pursuit, cross-track feedback (2/s), a
  limit on how fast the commanded velocity can change, and yaw toward the path
  so the 80° camera looks into turns. A repulsion reflex under the planner
  pushes off anything mapped within 0.4 m. On takeoff the drone eases forward
  away from the walls beside the launch mark, which the camera can't see.
- **Height** (`altitude.py`). Cruise is 0.75 m. Height is PX4's inertial and
  visual estimate. A rangefinder reading corrects the floor estimate only when:
  - it agrees within 4 cm;
  - the map shows nothing under the drone;
  - the correction stays within ±10 cm of the takeoff floor.
  A reading that is too short is a boulder or tarp under the drone: it is
  marked in the map and never treated as a height change.
- **Finding the pad.** The mat moves between visits: it can be anywhere in the
  landing area's south-east corner. From the window the drone heads for the
  pad's first `lookout` (goals.yaml) with the camera on that corner; most
  mats are seen on the way. The detector needs the whole mat in the frame,
  and the camera sees the floor only from ~0.85 m out, so the lookouts stand
  well back, north of the corner, either side of A1. Together they see all of
  it. At each, if the mat is still unseen, it sweeps the heading ±35° once
  (3.5 s), then tries the next lookout. It aborts if no lookout shows the mat,
  rather than hovering.
- **Finish.** Over the pad (from a pad detector, below) it descends and
  touches down armed for CP5, and lifts off again 1 s later: PX4 disarms a
  landed vehicle after `COM_DISARM_LAND` (2 s), and Offboard does not come
  back until the pilot arms again. It climbs to 2.1 m, flies straight over
  the barrier toward its takeoff point, and descends until PX4's land
  detector and `COM_DISARM_LAND` disarm it there, for the return bonus. By then the
  odometry has drifted up to ~0.2 m, and the launch corridor leaves ~0.25 m a
  side to the nearly invisible west net. So the touchdown spot is set from
  M1's corridor face and the barrier's north face, measured from depth seen on
  the way back (`return_spot.py`, goals.yaml `return_walls`): on the mark
  itself, 0.50 m from M1 and 0.42 m from the barrier, at most 0.3 m from the
  remembered spot. The spot it was placed on is clear; around it, where M1
  meets the barrier, stand feet and pavers reach ~0.6 m into the corridor at
  floor level. Until both
  are measured it stops 0.4 m short (for up to 3 s), and it keeps the spot
  once below 0.6 m, where tarp skirts would fool the fit.

`drone_ref/goalnav_node.py` is the ROS 2 adapter. It talks to PX4 only
through the organizers' `skybeta_flight_interface`
(`skybetav1/docs/competitor_guide.md` §1-4); nothing is published on
`/fmu/in/*`:
- **Subscribes:** PX4 local position and attitude (read only; the
  downward laser's reading is the position's `dist_bottom`: PX4 v1.16 does
  not publish `distance_sensor` over DDS); `/skybeta/state` (armed,
  Offboard, landed); `drone_ref/stereo_node.py`'s points and rectified left
  image; a colour image for the pad only if `pad_image_topic` names one.
- **Sees with** `stereo_node`: skybeta-03's camera publishes only the raw
  mono pair (1024x640, 30 Hz) and the IMU, for cuVSLAM: no on-camera depth,
  no colour in that original enabled pipeline (the hardware does have a colour sensor). The node matches the pair on the Jetson with overwall_nav's
  `StereoRig` (scale 0.4, SGBM; ~4.5 Hz, ~130 ms old with cuVSLAM running)
  and publishes the points (left camera frame) and the rectified left image.
  It is the only subscriber to the full-size images: a second Python one
  drops frames. The floor tape is found in that image by brightness (under
  0.3 of the floor's). The pad is not: in brightness alone the mat is within
  ~10 % of the floor (see goalnav/README.md).
- **Publishes:** `/skybeta/setpoint/twist` at the control rate (50 Hz), and
  `/goalnav/status`.
- **Start:** no command. The node streams a zero twist from start-up (the
  interface needs a setpoint less than 1 s old). The safety pilot arms on
  Altitude, then switches to Position; `flight_interface`
  (`auto_offboard_on_arm:=true`, started by `launch/goalnav.launch.py`)
  enters Offboard; the mission starts and takes off as soon as
  `/skybeta/state` shows `offboard_active`, well inside the 10 s PX4 allows
  after arming. If Offboard ends, the mission stops (`HANDED_OVER`).
- **Control** (`goalnav/skybeta.py`). The interface sends PX4 a position or a
  velocity setpoint at 10 Hz, never both and no acceleration feedforward; a
  pose is floored at 0.3 m and persists, a twist is capped at 1.5 m/s and
  expires after 1 s. So the mission's setpoints (velocity, plus a position to
  hold and an acceleration feedforward) go out as twists only: the velocity,
  plus 0.95 x the position error (MPC_XY_P) and the feedforward / 1.8
  (MPC_XY_VEL_P_ACC), the heading as a yaw rate (2.8 x its error). Never a
  pose: if goalnav stalls, the twist expires and PX4 holds where it is (no
  stale pose to fly back to), and the landings can go below the pose floor.
  There is no arm, disarm, mode or land command: takeoff is an upward twist,
  every landing a downward one held until PX4 disarms, and ABORTED descends
  where it is.

## The simulator: the course as built

`localsim/messy_course.py` (+ `messy_stand.py`, `messy_mesh.py`) builds a new
course per seed from the CAD's topology and photos of the event net:

- **Walls:**
  - **Panels:** plywood on wooden stands, in two lengths (long 1.61 m, short
    1.03 m), each the same everywhere. A1 and C2b, either side of the pad, are
    short. Every panel has the same height and an 18–32 cm gap at its base.
  - **Tarps:** printed vinyl (grey sci-fi panels or red rock) on one side only.
    The bare side has an upright post and a diagonal strut at each end (a
    triangle reaching ~0.13 m out at flight height). Batten feet run on the
    floor with pavers on them.
  - **Placement** is what differs from the CAD: walls pivot where they meet the
    barrier or the net, slide ±16 cm and turn ±5° (every placement offset is
    1.3× the first estimate: the course has been rebuilt differently between visits).
  - **Barrier:** two long panels and one cut to the tunnel.
- **As-built corrections from the team:**
  - **C1** leans the opposite way from the CAD, less steeply, narrowing the
    tunnel exit.
  - **The window** is ~0.70 × 0.70 m (CAD 0.91) and 0.2 m further along its
    wall toward the course's middle.
  - **The tunnel** is ~0.3 m short and ~10 cm lower. It is a wire-mesh hoop
    tube with a rock-print tarp draped over it and hanging off its mouth, and
    tall rocks at its four corners.
  - **The target** is a smaller blue H mat, at any rotation. Its centre is
    anywhere in the landing area's south-east corner (`PAD_AREA`), at least
    0.8 m from the window wall's centreline and wholly inside the net. Three
    large rocks stand around it, each one's near edge 0.40–1.0 m (1.3 ft at
    the nearest) from the H.
- **Floor:** an orange wrinkled tarp, and the black tape line through the maze
  (the CAD guideline, carried with the dividers it runs between).
- **Room:** an almost invisible net, with people walking, tables and the hall
  behind it.
- **Placement and checks:** the drone is placed ±0.2 m and ±12° off the launch
  mark. Layouts that close a passage (5 cm slack a side at flight height, 2 cm
  for the window), open a way around the tunnel, or put a tarp or paver under
  the drone on the launch mark are resampled. The maze must stay open with
  the north net 10 cm further in: the gates at its far end (M1's end, gate A)
  are against that net, which the drone cannot see. `--severity 0` keeps the CAD
  placement with everything else as built.

Sensor models (`localsim/messy_world.py`):
- **Stereo depth:**
  - error grows with distance squared (0.25 px of disparity on a 7.5 cm baseline);
  - 6% of returns drop out and 1.5% are outliers;
  - tarps ripple ±5 cm (moving);
  - glossy vinyl drops a further 12%;
  - the wire tunnel returns from only ~35% of its mesh each frame;
  - net strands return with a probability falling with distance;
  - everything else passes through to the room.
- **Rangefinder:** the downward VL53L1X reports the nearest return in its 27°
  cone.
- **VIO drift:** 0.008 m/√s and 0.0015 rad/√s.

The referee's checkpoint lines move with their walls. Sustained (1 s) or deep
wall contact counts as a crash, a graze as a touch.

## Running it

From `reference_stack/`, with NumPy, SciPy and PyYAML:

```bash
python3 -m localsim.goalrun --seed 3                 # one course; writes localsim/out/goalnav/seed3/
python3 -m localsim.goalrun --seed 3 --severity 0    # CAD placement, as-built construction
python3 -m localsim.viewer3d localsim/out/goalnav/seed3   # rebuild the 3D replay
```

`viewer.html` is a self-contained 3D replay (three.js inlined), viewable
offline. Layers:
- **truth:** the course, tarps, net and room;
- **the drone's belief:** the live map growing, raw depth returns, each goal's
  estimate (violet: CAD prior, orange: found, grey: passed), the planned path
  and the flown path.

It also has a HUD, the checkpoint times, the event log, and orbit, chase and
top cameras. `#t=20&cam=chase&hide=map,cloud` in the URL jumps to a time,
camera and layer set.

## Results (local simulator, 2026-11-06, photo-matched course)

| Courses | Reach the pad (CP5) | All 5 checkpoints credited | Via window | Via Gate B fallback |
|---|---|---|---|---|
| Severity 0, seeds 1–8 | 7 | 6 | 2 | 4 |
| Severity 1, seeds 1–8 | 2 | 2 | 0 | 2 |

Severity 0 times:
- through the window: pad at 39–41 s, back on the launch mark at 56–58 s;
- via Gate B: pad at 49–68 s.

Earlier, on the softer pre-photo course (0.91 m window, two-sided tarps, no
struts), the same navigator finished 7 of 10 at severity 0 and 5 of 11 at
severity 1.

Where it fails:
- **The window**, with 7 cm to spare a side. It is usually seen and centred,
  but the drone often can't get the path through and takes the Gate B fallback.
- **Grazing tarps in the maze hairpins**, where tracking error of 10–15 cm
  meets about as much slack.
- **CP1 not credited** when the drone turns east before its centre crosses
  the line.

## Limits

- The pad detector is the real one (`pad_detect.py`) on rendered colour: always with
  `--camera rgbstereo` (the default, as the drone's team camera config), or `--camera rgbd --colour`.
- There is no stereo image and no cuVSLAM: VIO is the vehicle model's
  drifting estimate.
- The PX4 model is the Python one, not SITL. Between goalnav and it runs
  the organizers' `flight_interface` itself, unmodified (`localsim/goalrun.py`,
  on `fakeros`; 0.3.3, as on skybeta-03): its clamps, conversions, twist
  expiry and 10 Hz output. PX4 is system 3 and ignores commands for any other.
  The safety pilot arms on Altitude at 3 s and switches to Position 1.5 s later.
- The simulator gives goalnav what skybeta-03 gives it: stereo points from
  the left camera's mount as measured (27.6 deg down, (0.105, 0.038, 0.030) m;
  goalnav assumes 27), half of them matched, 0.2-6 m, at 4.5 Hz and 130 ms
  old; a mono image (205x128, half the stereo node's); the laser's reading
  in `dist_bottom` (a 1 deg beam). Stereo is not rendered: the depth's holes
  are random, not where real matching fails (glare, blank surfaces).
  `--assume-colour-pad` adds a colour image for the pad; without it the pad is
  never found, as on the drone.
- Panel dimensions, strut geometry, window size and tunnel height are
  estimates from photos and team notes, not measurements.
- Unit tests for `goalnav` are not written.
