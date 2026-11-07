# Goal navigation on a live map, and the photo-matched course simulator

The race stack ([RACE.md](RACE.md)) replays a plan computed from the CAD. The
course as built is not the CAD: walls are feet off, freestanding and can move
between runs, the net is nearly invisible and everything is tarped. This
stack does not trust the CAD for positions at all. It uses it for the order of
the course and the rough offsets between its parts, and finds everything else
with its own sensors during the run. Nothing is stored between runs (rule 6.4).

## How it flies

```
depth (OAK-D) ──► live map ──► goal estimates ──► A* path ──► speed limit ──► follower ──► PX4 velocity
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
- **Finish.** Over the pad (from a pad detector, below) it descends and disarms
  for CP5. It then re-arms, climbs to 2.1 m, flies straight over the barrier
  to its takeoff point and lands, for the return bonus.

`drone_ref/goalnav_node.py` is the ROS 2 adapter:
- **Subscribes:** PX4 local position, attitude, status and land detector;
  depth and CameraInfo; `/fmu/out/distance_sensor`; pad detections.
- **Publishes:** Offboard velocity setpoints, vehicle commands, and
  `/goalnav/status`.
- **Start:** `/goalnav/start` is the single start command.

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
  the drone on the launch mark are resampled. `--severity 0` keeps the CAD
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

- The pad detector in the simulator is an oracle: the pad's true centre, with
  noise, whenever it is in the camera's view. A real RGB detector for the blue
  H mat is not written.
- Nothing uses the floor line yet; the simulator only records where it is.
- There is no stereo image and no cuVSLAM: VIO is the vehicle model's
  drifting estimate.
- The PX4 model is the Python one, not SITL.
- Panel dimensions, strut geometry, window size and tunnel height are
  estimates from photos and team notes, not measurements.
- Unit tests for `goalnav` are not written.
