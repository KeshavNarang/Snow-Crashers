# Agent coordination board

| Workstream | Owner / branch | State | Files | Next step |
|---|---|---|---|---|
| Sensor selection and declaration | unassigned | todo | agents/sensor/ | Pick sensor (<$100, <50 g), confirm price link and mass |
| Sensor driver and ROS topic on the Jetson | unassigned | todo | | Wire to Jetson/Pixhawk, publish topic |
| Sensor mount (CAD) | unassigned | todo | cad/ | Print bracket; passes rule 8.4 inspection |
| Fuse sensor into goalnav | unassigned | todo | goalnav/ | After driver works |
| Drone bring-up on skybeta-03 (read-only first) | claude | done (depth look) | agents/logs/2026-11-07-depth.md | Fix floor-fit orientation gate in overwall_nav |
| Depth latency/rate (single-thread node, ~4 Hz) | unassigned | todo | team_ws/src/overwall_nav, agents/logs/2026-11-07-depth-bottleneck.md | scale 0.4 tried and rolled back; next: threaded or C++ node, or on-camera depth |
| CPU-bottleneck monitor + frame-rate survey | claude / agent/cpu-monitor | done (monitor) | tools/drone_tools/, agents/logs/2026-11-07-cpu-monitor.md | Open: camera 24-26 Hz vs 30 and bgr8 need organizer-owned config (oakd_vslam.yaml); needs approval + PX4 work finished |
| goalnav on the drone: mono-pair depth, mono pad/tape, rangefinder from local position, flight logger | claude / agent/goalnav-drone | blocked | goalnav/, agents/logs/2026-11-07-goalnav-hardware.md | Pad needs the colour camera (organizers) or real mono images of the mat; then deploy + props-off bench run (30 Hz with stereo_node + recorder) |
| Floor-fit orientation gate (evals first) | claude / agent/sensor-floorfit | done on branch, not deployed | team_ws/src/overwall_nav/{overwall_nav/stereo.py,test/test_floorfit.py}, agents/logs/2026-11-08-floorfit.md | Merge after review; deploy needs a power-cycle |
| Ladder in the simulator (2026-11-07; moved to the SW corner later) | claude / agent/sim-ladder | committed, not pushed | goalnav/localsim/messy_ladder.py (POSES, DEFAULT_POSE='moved'), test_messy_ladder.py, agents/logs/2026-11-07-ladder.md | Switch back: DEFAULT_POSE='original' or goalrun --ladder-pose original. Confirm the pose on site |
