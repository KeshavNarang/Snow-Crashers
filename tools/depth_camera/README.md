# depth_camera: on-device depth from the OAK-D S2, without breaking VIO

Plan, facts from the driver source and the pass/fail contract: `agents/logs/2026-11-07-depth-camera-plan.md`.

| File | What |
|---|---|
| `driver_params.py` | Builds the depth configuration = the running driver's own `ros2 param dump` + a checked overlay. Refuses any change outside `pipeline_gen.i_pipeline_type` / `stereo.*`, implicit geometry settings, depth aligned to the absent RGB camera, aspect or rect-size mismatches. |
| `driver_swap.py` | Inside the container: `health`, `candidate [--dry-run]`, `restore`. Unloads only the driver component from the stack's `/oak_container` and loads the new configuration into it (`load_driver.launch.py`), so no process exits. Saves the organizers' original once; reverts by itself if cameras or cuVSLAM are unhealthy afterwards. |
| `live_health.py` | Health decision from `camera_info` stamps (left/right/depth) and `/visual_slam/status`. |
| `acceptance.py` | Candidate vs baseline verdict: A1-A4 (VIO must not regress), B1-B4 (depth real and right). Missing data fails. |
| `depth_analysis.py` | Depth decoding, floor fit, depth comparison and frame-shift search, frame-loss and pairing stats, camera_info/TF diffs. |
| `mutation_check.py` | Proves the evals can fail: applies one deliberate bug at a time to a copy. |

```bash
python3 -m unittest            # all evals, no drone/ROS needed (71 tests)
# on the drone, inside skybeta-stack, lock held, props off, disarmed:
python3 driver_swap.py health                     # read-only
python3 driver_swap.py candidate --dry-run        # changes nothing
python3 driver_swap.py candidate --size 640x400 --align left
python3 driver_swap.py restore
```

A swap interrupts VIO for some seconds and cuVSLAM may restart its origin: power-cycle before any flight.
Nothing here is applied at boot; persistence is a separate decision.
