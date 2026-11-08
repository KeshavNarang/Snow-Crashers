# Snow Crashers documentation

Start here for current project guidance. Updated 2026-11-07 from repository code,
dated session logs, branch handoffs, and the team's reports in chat. This is not a
live inspection of the drone.

| Read | Purpose |
|---|---|
| [Current status](STATUS.md) | What has flown, what was bench-tested, branch differences, current competition guidance |
| [Running on the drone](FLIGHT.md) | Deployment, startup, arming and takeover; what must already be running |
| [Cameras and depth](CAMERAS.md) | Physical sensors versus enabled pipelines, colour detection, measured rates |
| [Simulation](SIMULATION.md) | Applicable entry point, assumptions and limits of simulation results |

## Package references and historical sources

- [Simple up/forward/land package](../team_ws/src/snowcrashers_nav/README.md)
- [Wall-crossing package](../team_ws/src/overwall_nav/README.md)
- [Goal navigator](../goalnav/README.md) and [design notes](../goalnav/docs/GOALNAV.md)
- [Detailed drone setup](../team_ws/DRONE_SETUP.md): includes historical troubleshooting;
  use the current flight guide first.
- [Organizer stack](../skybetav1/README.md) and [flight-interface contract](../skybetav1/docs/competitor_guide.md)
- [Dated session logs](../agents/logs/): historical measurements and decisions;
  later sections can supersede earlier sections in the same file.
- [Agent coordination board](../agents/STATUS.md): task handoffs, not proof of live deployment.
- [Original organizer lab context](../goalnav/docs/organizer/AI_CONTEXT.md): a different
  simulator environment, not the hardware's current ROS/topic contract.
- Root `README.md` and `THOUGHTS.md` are preserved human notes. Older scoring and
  restrictions there should be read with the team's current guidance in STATUS.md.

Keep package-specific reference material beside its code, dated evidence in the
logs, and current cross-project instructions here. Update these guides when a new
flight, deployment, camera configuration or organizer decision supersedes them.
Distinguish user reports, recorded bench measurements, simulation and live verification.

## Mandatory RGB and evidence

Real RGB is required for pad finding and enabling it is competition-permitted,
confirmed by the user. Never fake RGB-sensor values or present rendered/replayed
images as live sensor evidence. See [the camera requirements](CAMERAS.md#required-for-the-real-pad-mission).
