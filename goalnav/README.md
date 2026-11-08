# goalnav: the Sky Beta's course navigator and its simulator

> Start at the [central docs index](../docs/README.md), especially [status](../docs/STATUS.md), [cameras](../docs/CAMERAS.md) and [simulation](../docs/SIMULATION.md). This package README describes the CPU-stereo implementation in this checkout; the newer on-camera-depth adapter is tracked separately.

Moved here from the old prep repo (`SnowCrashers Prep/reference_stack`); only
the goal navigator, its simulator and what they need came along (the old
CAD-replay race stack did not). ROS 2 package name: `drone_ref`.

## Layout

| Path | What |
|---|---|
| `drone_ref/goalnav/` | The navigator: live map, goal chain, A* planner, window manoeuvre, pad detection, return |
| `drone_ref/goalnav_node.py` | ROS 2 adapter: stereo points + mono image, PX4 position (and its rangefinder reading); velocity setpoints to skybeta's `flight_interface` (never `/fmu/in/*`) |
| `drone_ref/stereo_node.py` | Depth and the rectified left image from the OAK-D's mono pair (overwall_nav's `StereoRig`), drone only |
| `launch/goalnav.launch.py` | flight_interface, stereo_node, goalnav_node and the logger (`~/team_ws/logs`) |
| `config/goals.yaml` | The course as a chain of goals, plus course facts (net anchors, return mark and corner, takeoff offset) |
| `config/goalnav_profiles.yaml` | Risk profile: `safe` only (faster/race profiles removed: we play it safe) |
| `localsim/` | The as-built course simulator (`goalrun.py`), 3D viewer, run index |
| `assets/course-scan.npz` | The 2026-11-06 phone scan, aligned to the nets, cropped (made by `tools/course_scan.py`) |
| `tools/course_map.py` | The labelled course map (scan, goal chain, one run, window elevation, drone side view) |
| `docs/GOALNAV.md` | Design notes (written before the scan; see "Since the notes" below) |
| `docs/organizer/` | The organizers' rules, hardware notes and AI context |

## Simulate

```
python3 -m localsim.goalrun --seed 1 --severity 0.7 --profile safe --duration 200 --out localsim/out/goalnav/s1
python3 -m localsim.live_index localsim/out/goalnav
python3 -m http.server 8765 --bind 127.0.0.1 --directory localsim/out/goalnav   # then http://127.0.0.1:8765/
PYTHONPATH=. python3 tools/course_map.py . localsim/out/goalnav/s1 localsim/out/goalnav/course_map.html
```

Needs `numpy`, `scipy` (scan tool only) and `pyyaml` (`localsim/requirements.txt`).
`--tune key=value` overrides any profile setting for one run.

## Fly

On skybeta-03 (see `team_ws/DRONE_SETUP.md`, the root `CLAUDE.md` and
`tools/drone`; take `tools/drone-lock` first). The package needs `overwall_nav`
(its stereo matcher) in the same workspace:

```
tools/drone run 'rm -rf ~/team_ws/src/drone_ref'   # scp nests into an existing folder
tools/drone push goalnav team_ws/src/drone_ref      # then, on the drone:
cd ~/team_ws && ./run_mission.sh drone_ref goalnav.launch.py   # or set COMPETITOR_PKG/LAUNCH in drone.env
```

`goalnav.launch.py` starts four things: skybeta's `flight_interface`
(`auto_offboard_on_arm:=true`), `stereo_node` (depth and the rectified left
image from the mono pair, ~5 Hz), `goalnav_node`, and the logger. There is no
start command: the pilot arms on Altitude, then switches to Position, the
interface enters Offboard and the navigator takes off (docs/GOALNAV.md, "the
ROS 2 adapter"). Arguments: `profile`, `tuning`, `flight_interface`,
`log_dir`, `record`, `record_video`.

**Logs.** Every launch writes `~/team_ws/logs/<date-time>/` on the drone (the
only folder that survives a stack restart or power cycle): `bag/` (MCAP, split
every 2 GB) with both mono camera streams, IMU, the stereo points and image,
cuVSLAM, PX4's state, `/skybeta/state`, every setpoint, `/goalnav/status` and
`/rosout`; `ros/` with every node's log and screen output; `run.json`. The
video is ~40 MB/s, ~7 GB for a 3-minute run (164 GB free on skybeta-03).
Pull a run with `tools/drone pull team_ws/logs/<date-time> .`; replay it with
`ros2 bag play`.

**Real RGB is required for pad finding, and enabling it is competition-permitted (user confirmation). Never fake RGB-sensor values.**

**Pad input needs wiring and verification.** The drone has a tested IMX378 colour camera. Whether its pipeline publishes colour and this adapter subscribes to it is configuration-dependent (`pad_image_topic` is empty by default here). Without usable colour the existing colour detector cannot find the mat, and the mission eventually aborts/descends. Camera-pipeline changes have organizer approval; see [camera status](../docs/CAMERAS.md).

`stop_after: <goal id>` ends a staged test there: straight down until PX4
disarms it (`COM_DISARM_LAND`). Goal
ids are in `config/goals.yaml` (`corridor_1`, `first_corridor` (CP1), `gate_a`
(CP2), `tunnel_exit` (CP3), `window_back`, `pad`). Suggested order: props off on
the bench first (check the camera panels, the camera's top edge level, the mat
detected, time per frame), then `corridor_1`, then each checkpoint in turn.

Check before flying: the PX4 origin's height over the feet (`landed_base_height`,
0.07; the CAD's IMU is ~65 mm up), the left camera's offset and pitch (from
skybeta-03's TF: (0.105, 0.038, 0.030) m, 27 deg; `camera_offset`,
`camera_pitch_deg`), that PX4's `COM_DISARM_LAND` stays above the 1 s rest on
the pad, and with props off that the camera (`/oak/*/camera_info`) and
cuVSLAM stay at 30 Hz while the stereo node and the recorder run.

## Since the notes

- **Course from the scan:** one long window wall (C2a only), the window centred,
  0.90 m wide, sill 0.87 m; M2 and M3 0.29 and 0.70 m east of the CAD; the mat
  toward the landing area's south-east corner with one rock in front; the next
  course 3 ft north (blue floor), glass windows 8 ft west.
- **Drone shape from the CAD:** feet 7 cm under the origin, battery top 5 cm
  over, arms/motors 0.20 m, rotor tips 0.29 m.
- **Pad:** touches down armed and lifts off again after 1 s, under PX4's
  2 s `COM_DISARM_LAND`: nothing may disarm or re-arm it (the interface has no
  arm path), and Offboard comes once per arming. The published rules say
  "land and disarm" on the pad: confirm with the organizers.
- **Return:** one move along a height profile from the map, facing where M1
  meets the barrier.
- **Holds:** takeoff, descents and any stop are twists carrying a position
  correction (the interface takes a pose or a twist, never both).
- **Deployment evidence:** the older CPU-stereo goalnav stack built and ran props-off on the drone, including logging and stereo. This does not establish a goalnav flight or validation of the newer depth-image adapter. The simple up/forward/land mission has flown successfully; see [status](../docs/STATUS.md).
