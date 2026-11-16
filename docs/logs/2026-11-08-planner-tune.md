# 2026-11-08: planner tuning on main (RGBSTEREO, ladder original), 25 seeds

Success = rests on the pad within 120 s, no contact before ON_PAD (the return flight after is
not judged). `localsim.goalrun --severity 0.7 --profile safe`, seeds 1-25, main at 3e56c92.

| Version | Success | Failing |
|---|---|---|
| main (baseline) | 23/25 | 5 (contact at take-off), 22 (window jamb) |
| + launch floor clearance (kept, this branch) | **24/25** | 22 |
| + window approach 0.75 -> 0.9 m (reverted) | 23/25 | 10 (stalled, landed off the pad), 16 (window jamb) |

## Seed 5: a sim layout flaw, fixed
The swung M1's stand frame lay 0.086 m from the launch mark, below the 3 cm floor band
`validate()` checked, so the drone touched it while spinning up on the mark. Nobody would launch
from there. `validate()` now also keeps -0.5..3 cm clear within `LAUNCH_FLOOR_CLEAR` (0.22 m, the
arms' reach). Eval: the flown layout (seed 5, attempt 8) has the batten and is now rejected.

## Seed 22: window, open
- The window estimate was accurate (3 cm off), but the drone held 9-12 cm off it through the
  settle and drifted to ~14 cm at the plane: the jamb by 1 cm (0.91 m opening).
- The settle phase waits for 8 steady jamb measurements (`WindowManeuver.observe`, which re-centres
  on the jambs seen now, no drift) and gave up after its 6 s timeout: the estimate never changed,
  so it flew the drifted line. Measurements only count while >= 0.62 m short of the plane
  (`view_until`), and the depth now comes at ~4.5 Hz (RGBSTEREO).
- Tried: approach back to 0.9 m (both jambs further inside the view). Seed 22 passed, but 10 and 16
  failed: 23/25, within the noise of 25 runs, so reverted (2 tuning tries, CLAUDE.md).
- Next: record the jamb measurement count (`len(window.centres)`) in goalrun's trace to see why
  the settle gets none, before touching the manoeuvre. Seeds 22 (0.75) and 10, 16 (0.9) to replay.
