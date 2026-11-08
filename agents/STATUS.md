# Agent coordination board

| Workstream | Owner / branch | State | Files | Next step |
|---|---|---|---|---|
| Sensor selection and declaration | unassigned | todo | agents/sensor/ | Pick sensor (<$100, <50 g), confirm price link and mass |
| Sensor driver and ROS topic on the Jetson | unassigned | todo | | Wire to Jetson/Pixhawk, publish topic |
| Sensor mount (CAD) | unassigned | todo | cad/ | Print bracket; passes rule 8.4 inspection |
| Fuse sensor into goalnav | unassigned | todo | goalnav/ | After driver works |
| Drone bring-up on skybeta-03 (read-only first) | claude | done (depth look) | agents/logs/2026-11-07-depth.md | Fix floor-fit orientation gate in overwall_nav |
| Depth latency/rate (CPU SGBM, ~3 Hz, ~0.4 s old) | unassigned | todo | team_ws/src/overwall_nav, agents/logs/2026-11-07-depth-bottleneck.md | Organizers: on-camera depth? else tune SGBM |
