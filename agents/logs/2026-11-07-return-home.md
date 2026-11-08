# 2026-11-07: the return home (pad -> over the barrier -> launch mark), as a module

Branch `agent/return-home`, on `agent/planner-rebuilt` 69e1efe (rebuilt maze + ladder) with
`agent/sim-clearance` merged in (batch tools). 30 min timebox.

## What was built
- `goalnav/drone_ref/goalnav/homeward.py`: the return's thresholds in one `HomewardConfig`, no mission
  state, so the final planner can take it whole:
  - `reach` = rotor radius 0.29 + target 0.184 = 0.474 m sideways (mission's `return_reach` 0.45 left
    0.16 m); `vertical_margin` 0.19 m feet over the tallest thing near the line (was 0.15); ceiling 2.2 m.
  - `face_offset`: the touchdown spot >= `reach` from each measured wall (barrier's 0.42 left 0.13 m).
  - `spot_may_move`: the spot follows ReturnSpot's wall measurements only above the wall tops (1.85 m).
  - `DescentGate`: beside the walls, hold when > 0.08 m off the spot or sliding > 0.15 m/s, go on at
    < 0.04 m / < 0.08 m/s; holds total <= 4 s, then slow on down; never holds with the feet nearly down.
- `mission.py`: one switch, `return_guard` (default **False**), wiring the four into `_return_profile`,
  `_observe_return`, the ReturnSpot walls and `_return_descend`. Off, a run is identical to before
  (seed 3, drone scenario: same 807 trace frames).
- `goalnav/test/test_homeward.py`: 12 evals (margins vs the target, the old margins shown under it,
  freeze height, gate hysteresis, hold cap, ground).

## Result (drone scenario, severity 0.7, seeds 1-16, --assume-colour-pad, rebuilt course)
| | Finish | Home mean / max s | Return scrapes (clearance <= 0 on the way home) |
|---|---|---|---|
| return_guard off (as flies) | 11/16 | 77.9 / 86.9 | 3 (s3, s8, s15) |
| return_guard on | 11/16 | 79.5 / 88.6 | 6 (s3, s4, s8, s11, s13, s15) |

**The guard made the return worse; it stays off.** The 5 failures are before the pad (maze / window
approach, 0 checkpoints credited on the rebuilt course): the planner's work, the same in both.
Not yet known which part hurts (the higher, wider profile, the 5 cm spot shift, or holds letting drift
build beside the walls). The return scrapes are not seen by the referee (it stops counting touches at
the pad; see 2026-11-07-sim-clearance.md).

## Next
1. Turn the parts on one at a time (add a flag per part) and batch each, 24+ seeds, to find the harmful one.
2. Log where each return scrape happens (state, height, which wall) in `localsim.batch`.
3. The launch mark has only 0.05-0.12 m of best-case clearance at 1.15 m (sim-clearance log): the
   descent itself, not the cruise, is where margins can't be met.
