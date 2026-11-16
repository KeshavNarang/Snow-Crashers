# Snow Crashers documentation

Start here for current project guidance. Updated 2026-11-15 from repository code,
dated session logs, branch handoffs, and the team's reports in chat. This is not a
live inspection of the drone.

## Guides

| Read | Purpose |
|---|---|
| [Current status](STATUS.md) | What has flown, what was bench-tested, branch differences, current competition guidance |
| [Flying goalnav](FLYING.md) | The four `tools/fly` commands, safety rules, and how start-up and the pilot handover work |
| [Flying by hand](MANUAL_FLIGHT.md) ([one-page version](MANUAL_FLIGHT_QUICK.md)) | Manual flight with everything recorded |
| [Cameras and depth](CAMERAS.md) | Physical sensors versus enabled pipelines, colour detection, measured rates |
| [Simulation](SIMULATION.md) | The 2D race replay, the 3D goal-navigation sim, how to run it and what a result proves |
| [Agent guide](AGENTS.md) | Rules and context for coding agents working in this repo |

## Notes, evidence and sources

- [Human notes](HUMANS.md) and [early thoughts](THOUGHTS.md): the team's own notes, human-owned.
  Older scoring and restrictions there should be read with the current guidance in STATUS.md.
- [Dated session logs](logs/): measurements and decisions, with their scripts and pictures;
  later sections can supersede earlier sections in the same file.
- [Agent coordination board](logs/BOARD.md): task handoffs, not proof of live deployment.
- [Sensor selection](sensor/README.md) and its [criteria](sensor/CRITERIA.md).
- [Organizer material](organizer/): the challenge rules, hardware notes, their
  [autonomy pipeline guide](organizer/AUTONOMY.md) (skybeta 0.3.3+), and their
  [lab simulator context](organizer/AI_CONTEXT.md) (a different simulator, not the hardware contract).
- [Images](images/) used in the README and the simulation guide.

## Package references

- [Simple up/forward/land package](../team_ws/src/snowcrashers_nav/README.md)
- [Wall-crossing package](../team_ws/src/overwall_nav/README.md)
- [Goal navigator](../goalnav/README.md) and [design notes](../goalnav/docs/GOALNAV.md)
- [Detailed drone setup](../team_ws/DRONE_SETUP.md): includes historical troubleshooting;
  use the flight guide first.
- [Organizer stack](../skybetav1/README.md) and [flight-interface contract](../skybetav1/docs/competitor_guide.md)

Keep package-specific reference material beside its code, dated evidence in the
logs, and current cross-project instructions here. Update these guides when a new
flight, deployment, camera configuration or organizer decision supersedes them.
Distinguish user reports, recorded bench measurements, simulation and live verification.

## Mandatory RGB and evidence

Real RGB is required for pad finding and enabling it is competition-permitted,
confirmed by the user. Never fake RGB-sensor values or present rendered/replayed
images as live sensor evidence. See [the camera requirements](CAMERAS.md#required-for-the-real-pad-mission).
