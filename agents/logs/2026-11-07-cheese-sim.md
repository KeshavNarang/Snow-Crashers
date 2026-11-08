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
