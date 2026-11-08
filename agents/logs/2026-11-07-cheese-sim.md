# Simulator-only cheese experiment

Started 2026-11-07, approximately 19:49 PST. Branch: agent/cheese-run.
Scope: a special local-simulator run and replay, with onboard image and depth
feeds, using existing integrations. No hardware work or PX4 parameter changes.

The special route and command live in `goalnav/experiments/cheese/`.
Baseline: unmodified safe navigator, seed 1, severity 0.7, first 12 simulated
seconds; this is a smoke baseline, not a full course completion evaluation.
No tests added (user's repository instructions).

Result: DONE, disarmed on the pad. Touchdown error 0.055 m; aim error 0.027 m;
zero wall/net contacts, no crash, no bus errors. Four camera frames accepted
the pad. Crossing height 2.259 m; maximum altitude 2.276 m. All four course
checkpoints were skipped. Original camera frames: 107; trace frames: 277.
Simulator run time after cache initialization: 53.5 s.

The baseline reached NAVIGATE with zero touches and no crash in its bounded
12 s interval. Both runs emit the pre-existing all-NaN depth median warning.

Presentation: `cheese.html` embeds a compact sampled camera replay; the full
3D `viewer.html` retains the original camera frames. Inline html_render
succeeded; html_preview failed because headless Chrome exited with code 127.
No browser verification claimed. Artifacts copied to the default workspace's
ignored `goalnav/localsim/out/cheese/seed1/` before worktree cleanup.

## Validation on the rebuilt course (22:00-22:30, branch agent/cheese-validate)

Live skybeta-03 (read-only over USB, 5 min after boot): overwall_nav running, RGB on
(640x400 bgr8, ~8.6 Hz, fx 489.6), CPU stereo /overwall/front ~3.1 Hz, RGB TF pitch 27.0 deg,
px4_link false. Sim now runs --colour --camera-hz 3.0 --camera-latency 0.4.

- Old lookouts (south of the barrier) left the mat below the colour frame at 2.25 m.
- v2 lookouts north of the barrier: 13/20 over severity 0.1-1.0 x both ladder poses. A lookout
  at x -2.80 hit the west net; abort descents beside the barrier hit it.
- v3 lookouts [(-2.10,0.95), (-1.75,-0.25), (-2.45,0.95)]: fixed far-ladder sev 0.9; two prior
  successes re-checked OK. Still failing: near ladder ('original' pose) with the mat in the
  west of the landing area: the ladder hides it from the north, and from the south it is
  cut off at the bottom of the frame. Needs lower lookouts (code: lookouts fly at cruise height).
- Not done: the cheese goals/tunes are not wired into goalnav.launch.py / tools/fly on the drone.
