# overwall_nav: approach a wall with stereo depth, fly over it, land

A test mission for one wall in a test net, to check the drone's sensors end to end:
take off, creep forward until the wall is 0.5 m ahead, climb over it, land behind it.
**Not flown yet.** Same skybeta contract as `snowcrashers_nav`: setpoints only through
`flight_interface`; the pilot arms in Altitude, then switches to Position.

## Nodes

| Executable | Does | Commands the drone? |
|---|---|---|
| `stereo_depth` | Depth from the OAK-D-S2's two mono cameras (the stack has no depth topic), as 3D points in a level frame binned into 5 cm cells, plus a floor-plane fit -> `/overwall/front` | no |
| `sensor_check` | Listens ~12 s to every sensor the mission uses and prints PASS/WARN/FAIL | no |
| `overwall_mission` | The mission (`mission.py`, perception in `perception.py`) | via `/skybeta/setpoint/*` only |

## Launch files

| File | Runs | Use |
|---|---|---|
| `sensor_check.launch.py` | stereo_depth + sensor_check, then exits | before every flight, drone flat on the floor |
| `depth_test.launch.py` | stereo_depth only, keeps logging | bench: walk a board towards the camera |
| `overwall.launch.py` | flight_interface + stereo_depth + mission | the flight: `~/team_ws/run_mission.sh overwall_nav overwall.launch.py`, then power-cycle |

```bash
# on the drone, stack running (nothing here flies):
docker exec skybeta-stack bash -c "source skybeta_env.sh; ros2 launch overwall_nav sensor_check.launch.py"
docker exec skybeta-stack bash -c "source skybeta_env.sh; ros2 run overwall_nav stereo_depth"   # Ctrl-C to stop
```

## How it sees the wall

- **Camera mount: `camera_pitch_deg`, default 27 deg down.** The stack's TF says the camera is
  level; `sensor_check` measures the real angle two ways (floor-plane fit, IMU gravity).
  A camera pitched down sees nothing above its own height, mostly floor.
- **Wall distance:** the nearest surface at the drone's own height (0.35 m below the camera to
  0.25 m above), floor rejected. It must be dense (40+ points per 5 cm cell) **and vertical**
  (3+ connected cells tall): false stereo matches come from the top rows and sit in one row.
- **Untextured wall:** invisible to stereo, but it hides the floor behind it: the approach also
  stops when the floor ends within ~0.85 m. Then the top can't be measured: it climbs to
  `max_height_m` and goes over. **Put tape or posters on a plain test wall.** A see-through
  net may be invisible either way: set `max_forward_m` short of it as a backstop.
- **Wall top:** once the camera is ~0.15 m above it, the wall face's top edge appears below the
  camera; the top is measured (reads up to 5 cm high, the safe way) and it crosses with the
  camera `over_margin_m` (0.4 m) above it.
- **Holding the gap while climbing:** by depth, not by position. If the VIO frame is tilted
  (the TF/mount mismatch), PX4 shifts the drone ~0.45 m along its heading per metre climbed;
  the depth loop cancels that at the wall, and landing 2 m past the stop point leaves room for
  it on the way down.

## Tests (31, also run on the drone)

```bash
cd team_ws/src/overwall_nav && python -m unittest discover -s test     # ~2.5 min
```

- `test_stereo.py`: exact ray-traced stereo pairs with skybeta-03's real calibration and the
  27 deg mount (`test/synthetic.py`): floor fit, horizontal wall distance, wall top, blank wall,
  no phantom walls, wrong-pitch failure mode, rectification.
- `test_mission.py`: the mission against the drone's real takeover rules, a PX4 model, and a
  camera model on the rig's pixel grid: normal wall, known height, tilted VIO both ways, blank
  wall, too tall, no wall, false matches, no/lost depth, takeover, height offset, close wall.
- `test_checks.py`: the sensor-check verdicts.

## Before each test flight (stack running overwall.launch.py)

1. `config/overwall.yaml`: `wall_height_m: 0.0` measures the wall top with the camera (the point
   of this test); `max_forward_m: 4.57` (15 ft) is the backstop if depth never sees a wall.
   After an edit: battery out 10 s and in. Drone on the floor, nose at the wall, still 1 minute.
2. Sensor check (the stack already runs stereo_depth, so only the checker):
   `docker exec skybeta-stack bash -c "source skybeta_env.sh; ros2 run overwall_nav sensor_check"`
   Needs: PX4 link PASS, camera tilt (floor) PASS ~27 deg, stack camera TF PASS (~27 now that
   CAMERA_PITCH=0.4712), nothing in front at drone height.
3. Pilot: Altitude, arm, Position within 3 s. Throttle to the middle takes over at any time.
4. Watch: `docker exec -it skybeta-stack tmux attach -t skybeta`, Ctrl-b 3 (leave: Ctrl-b d).

## Live results on skybeta-03 (2026-11-07)

- Textured carpet, drone flat: **floor fit 27.8 deg down, camera 0.11 m above the floor**, 65 %
  matched, nothing in front, floor seen to 2.6 m: the 27 deg holder confirmed by stereo.
- Glossy table: SGBM piled false matches at its largest disparity (a fake surface ~0.29 m away,
  square-on): now rejected (saturated disparities and low-texture pixels are dropped).
- The driver's IMU -> optical TF is identity and wrong; in the BNO086's own axes gravity read
  [8.5, -0.24, -4.06] = 25.5 deg (`checks.oak_imu_pitch_deg`).
- The stack TF said 0.6 deg: `CAMERA_PITCH=0.4712` (27 deg down; radians, positive = nose down)
  set in `/etc/skybeta/drone.env`, applies at the next power-up.
- Depth runs ~100 ms per frame on the Jetson: limited to 5 Hz (`rate_hz`), stop triggers 0.2 m
  early to match. cuVSLAM odometry stayed 21-27 Hz with it running.
