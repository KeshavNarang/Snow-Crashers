# Agent coordination board

| Workstream | Owner / branch | State | Files | Next step |
|---|---|---|---|---|
| Sensor selection and declaration | unassigned | todo | agents/sensor/ | Pick sensor (<$100, <50 g), confirm price link and mass |
| Sensor driver and ROS topic on the Jetson | unassigned | todo | | Wire to Jetson/Pixhawk, publish topic |
| Sensor mount (CAD) | unassigned | todo | cad/ | Print bracket; passes rule 8.4 inspection |
| Fuse sensor into goalnav | unassigned | todo | goalnav/ | After driver works |
| Drone bring-up on skybeta-03 (read-only first) | claude | done (depth look) | agents/logs/2026-11-07-depth.md | Fix floor-fit orientation gate in overwall_nav |
| Depth latency/rate (single-thread node, ~4 Hz) | unassigned | todo | team_ws/src/overwall_nav, agents/logs/2026-11-07-depth-bottleneck.md | scale 0.4 tried and rolled back; next: threaded or C++ node, or on-camera depth |
| CPU-bottleneck monitor + frame-rate survey | claude / agent/cpu-monitor | done (monitor) | tools/drone_tools/, agents/logs/2026-11-07-cpu-monitor.md | Open: cuVSLAM sees 29.5 Hz (1.8 % frames missing, not CPU-related); bgr8 and any rate change need organizer-owned config (oakd_vslam.yaml) |
| goalnav on the drone: mono-pair depth, mono pad/tape, rangefinder from local position, flight logger | claude / agent/goalnav-drone | blocked | goalnav/, agents/logs/2026-11-07-goalnav-hardware.md | Pad needs the colour camera (organizers) or real mono images of the mat; then deploy + props-off bench run (30 Hz with stereo_node + recorder) |
| goalnav camera calibration check + latency sims | claude / agent/goalnav-calib | done | goalnav/drone_ref/goalnav_node.py, agents/logs/2026-11-07-goalnav-calibration.md | Mount now from the TF; confirm the "mount from TF" log line on the next drone run |
| Camera exposure / dark-tunnel behaviour | claude / agent/exposure-probe | blocked on drone lock | tools/drone_tools/exposure_*.py | Tools built and tested offline (87 evals); need the drone for: param list, steady-state brightness, cover-lens test with a person |
| Floor-fit orientation gate (evals first) | claude / agent/sensor-floorfit | done on branch, not deployed | team_ws/src/overwall_nav/{overwall_nav/stereo.py,test/test_floorfit.py}, agents/logs/2026-11-08-floorfit.md | Merge after review; deploy needs a power-cycle |
| GPU stereo matcher (VPI CUDA/OFA) + benchmark | claude / agent/gpu-stereo | active | team_ws/src/overwall_nav/overwall_nav/{stereo,vpi_stereo,stereo_depth_node}.py, tools/gpu_bench/, agents/logs/2026-11-07-gpu-stereo.md | Live A/B (cuVSLAM rate, CPU, GPU) once the drone lock is free; not deployed |
| Sim: maze as rebuilt 2026-11-07 (M2 gone, M1 30-60 deg, M3 short and swung, new NW panel); `LAYOUT`/`goalrun --layout scan` switches back | claude / agent/sim-walls | done (sim only) | goalnav/localsim/messy_course.py, agents/logs/2026-11-07-walls-rebuilt.md | Planner (config/goals.yaml) still plans the old maze; CP1 vs the NW panel needs the organizers; tape not re-measured |
| Ladder in the simulator (2026-11-07; moved to the SW corner later) | claude / agent/sim-ladder | committed, not pushed | goalnav/localsim/messy_ladder.py (POSES, DEFAULT_POSE='moved'), test_messy_ladder.py, agents/logs/2026-11-07-ladder.md | Switch back: DEFAULT_POSE='original' or goalrun --ladder-pose original. Confirm the pose on site |
