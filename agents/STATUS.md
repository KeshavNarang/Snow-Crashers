# Agent coordination board

> Task board, not a live deployment inventory. Older rows below are historical handoffs.
> See [current project status](../docs/STATUS.md) for successful simple flights, goalnav bench deployment, on-camera depth and updated competition guidance.

| Workstream | Owner / branch | State | Files | Next step |
|---|---|---|---|---|
| Sensor selection and declaration | unassigned | todo | agents/sensor/ | Pick sensor (<$100, <50 g), confirm price link and mass |
| Sensor driver and ROS topic on the Jetson | unassigned | todo | | Wire to Jetson/Pixhawk, publish topic |
| Sensor mount (CAD) | unassigned | todo | cad/ | Print bracket; passes rule 8.4 inspection |
| Fuse sensor into goalnav | unassigned | todo | goalnav/ | After driver works |
| Drone bring-up on skybeta-03 (read-only first) | claude | done (depth look) | agents/logs/2026-11-07-depth.md | Fix floor-fit orientation gate in overwall_nav |
| Depth latency/rate (single-thread node, ~4 Hz) | unassigned | todo | team_ws/src/overwall_nav, agents/logs/2026-11-07-depth-bottleneck.md | scale 0.4 tried and rolled back; next: threaded or C++ node, or on-camera depth |
| CPU-bottleneck monitor + frame-rate survey | claude / agent/cpu-monitor | done (monitor) | tools/drone_tools/, agents/logs/2026-11-07-cpu-monitor.md | Open: cuVSLAM sees 29.5 Hz (1.8 % frames missing, not CPU-related); later organizer approval permits camera-pipeline changes; see docs/CAMERAS.md |
| goalnav hardware integration | claude / agent/goalnav-drone; newer agent/goalnav-oak-depth | older adapter bench-run; newer adapter handoff | goalnav/, docs/STATUS.md, docs/CAMERAS.md | Real RGB is required and permitted; enable it and verify mat detection. Reconcile newer on-camera-depth adapter before flight; see central status for branch/deployment evidence |
| goalnav camera calibration check + latency sims | claude / agent/goalnav-calib | done | goalnav/drone_ref/goalnav_node.py, agents/logs/2026-11-07-goalnav-calibration.md | Mount now from the TF; confirm the "mount from TF" log line on the next drone run |
| Camera exposure / dark-tunnel behaviour | claude / agent/exposure-probe | blocked on drone lock | tools/drone_tools/exposure_*.py | Tools built and tested offline (87 evals); need the drone for: param list, steady-state brightness, cover-lens test with a person |
| Floor-fit orientation gate (evals first) | claude / agent/sensor-floorfit | done on branch, not deployed | team_ws/src/overwall_nav/{overwall_nav/stereo.py,test/test_floorfit.py}, agents/logs/2026-11-08-floorfit.md | Merge after review; deploy needs a power-cycle |
| GPU stereo matcher (VPI CUDA/OFA) + benchmark | claude / agent/gpu-stereo | active | team_ws/src/overwall_nav/overwall_nav/{stereo,vpi_stereo,stereo_depth_node}.py, tools/gpu_bench/, agents/logs/2026-11-07-gpu-stereo.md | Live A/B (cuVSLAM rate, CPU, GPU) once the drone lock is free; not deployed |

| overwall_nav lands on the pad (colour) | claude / agent/overwall-pad | done on branch, not deployed | team_ws/src/overwall_nav/{overwall_nav/pad.py,pad_detect_node.py,mission.py}, agents/logs/2026-11-07-overwall-pad.md | Colour on (team image, rgbd), props-off bench check of pad_detect with the real mat |
| Central documentation refresh | Codex / agent/docs-current-state | done (docs only) | docs/, package guides | Current index, successful-flight report, camera/branch distinctions and rules update; no drone changes |
