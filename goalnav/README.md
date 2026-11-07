# goalnav: the Sky Beta's course navigator and its simulator

Moved here from the old prep repo (`SnowCrashers Prep/reference_stack`); only
the goal navigator, its simulator and what they need came along (the old
CAD-replay race stack did not). ROS 2 package name: `drone_ref`.

## Layout

| Path | What |
|---|---|
| `drone_ref/goalnav/` | The navigator: live map, goal chain, A* planner, window manoeuvre, pad detection, return |
| `drone_ref/goalnav_node.py` | ROS 2 adapter: OAK-D depth + RGB, VL53L1X, PX4 offboard (full trajectory setpoints) |
| `config/goals.yaml` | The course as a chain of goals, plus course facts (net anchors, return mark and corner, takeoff offset) |
| `config/goalnav_profiles.yaml` | Risk profiles: `safe`, `normal`, `fast` |
| `localsim/` | The as-built course simulator (`goalrun.py`), 3D viewer, run index |
| `assets/course-scan.npz` | The 2026-11-06 phone scan, aligned to the nets, cropped (made by `tools/course_scan.py`) |
| `tools/course_map.py` | The labelled course map (scan, goal chain, one run, window elevation, drone side view) |
| `docs/GOALNAV.md`, `docs/LOCAL_SIM.md` | Design notes (written before the scan; see "Since the notes" below) |
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

Build with colcon (`goalnav_node` executable), then

```
ros2 launch drone_ref goalnav.launch.py profile:=safe tuning:='{stop_after: corridor_1}'
ros2 service call /goalnav/start std_srvs/srv/Trigger
```

`stop_after: <goal id>` ends a staged test there: straight down, disarm. Goal
ids are in `config/goals.yaml` (`corridor_1`, `first_corridor` (CP1), `gate_a`
(CP2), `tunnel_exit` (CP3), `window_back`, `pad`). Suggested order: props off on
the bench first (check the camera panels, the camera's top edge level, the mat
detected, time per frame), then `corridor_1`, then each checkpoint in turn.

Check before flying: the PX4 origin's height over the feet (`landed_base_height`,
0.07; the CAD's IMU is ~65 mm up), the camera offset and pitch (0.10 m, 26.7 deg),
and that PX4's `COM_DISARM_LAND` stays above the 1 s rest on the pad.

## Since the notes

- **Course from the scan:** one long window wall (C2a only), the window centred,
  0.90 m wide, sill 0.87 m; M2 and M3 0.29 and 0.70 m east of the CAD; the mat
  toward the landing area's south-east corner with one rock in front; the next
  course 3 ft north (blue floor), glass windows 8 ft west.
- **Drone shape from the CAD:** feet 7 cm under the origin, battery top 5 cm
  over, arms/motors 0.20 m, rotor tips 0.29 m.
- **Pad:** touches down armed (`pad_disarm: false`) and disarms back on the
  launch mark. The published rules say "land and disarm" on the pad: confirm
  with the organizers.
- **Return:** one move along a height profile from the map, facing where M1
  meets the barrier.
- **Holds:** takeoff, descents and any stop use PX4 position setpoints.
- **Not yet run on the drone:** the node's ROS wiring, PX4 taking full
  trajectory setpoints, the Orin's compute (7 ms per depth frame on a laptop).
