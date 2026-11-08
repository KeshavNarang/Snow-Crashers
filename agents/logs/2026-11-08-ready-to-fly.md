# Ready to fly: unattended session (claude, for James)

Started 2026-11-08 01:43. Ends 07:24. Branch agent/ready-to-fly.
Goal: goalnav safe and minimal enough to fly the full course on skybeta-03 (3 min allowed: slow is fine).
Drone not connected at start: lock (in /tmp) clears on reboot; the mission is still drone_ref /
goalnav.launch.py, so teammates need `tools/fly done` (or run_mission.sh overwall_nav ...) to get overwall_nav back.

## Log
- 01:55 CPU cut (no mono into goalnav, lazy colour, stereo 4 Hz, lighter bag) + 30 s bench, committed.
- 02:00 referee counts return touches; return floor over walls + spot keep-off (seed 602 clean now).
- 02:10 VIO lapse model + vision-hold guard: seed 704 sev0.9 lapse6 done/returned (13 lapses, 113 s).
- 02:55 OPEN: seed 707 (sev 0.7, no lapses) brushes the tunnel mouth's east side at (2.40, 1.60, 0.82),
  0.44 m/s, yaw swinging -1.6 -> -2.2 rad while entering. tunnel_inside WAS refined correctly from
  outside (-0.22 m x, matching the course's shift), entry estimate follows it, yet the body tracked
  x ~ 2.38 (~0.4 m east of the entry). Path following into the mouth, not estimation. A "look into the
  tunnel" stop did nothing (reverted). At the mouth the body went over the speed limit (0.39 vs 0.25
  m/s) with 2.9 m free ahead: pushed east and forward (sim aero near the mouth?). Next: aero vs path.
- 02:40 tight passages wait for 3 s of steady vision; no mapping while dead-reckoned; takeoff climb 0.25 m ahead (panel 1 stand). Seeds 704 lapse6 done (1 window brush), 706/702 clean, 705 nolapse stuck at window_lookout (no path; open).
