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

## Follow-up: which part hurt, and the real cause (same 16 seeds and flags)

Run in this worktree only: outputs in its git-ignored localsim/out, at most 6 runs at a time (another
session had 20 sims running), no change to goals.yaml (the planner terminal's file).

| Setting | Finish | Home mean / max s | Return scrapes |
|---|---|---|---|
| as flies (all off) | 11/16 | 77.9 / 86.9 | 3 (s3, s8, s15) |
| gate only | 11/16 | 80.1 / 89.3 | 3, the same |
| **freeze only** | 11/16 | 77.9 / 87.0 | **8** |
| spot only | 11/16 | 77.4 / 86.6 | 3, the same |
| margins only | 11/16 | 78.0 / 87.0 | 3, the same |
| all on (return_guard) | 11/16 | 79.5 / 88.6 | 6 |
| **goals copy: return_walls barrier only** | 11/16 | 78.4 / 87.9 | **2 (s8 -0.027, s15 -0.002 m)** |

- **Freeze is the harmful part.** Freezing the touchdown spot below the wall tops cuts off
  ReturnSpot's drift correction during the descent, the correction that keeps the drone off the walls.
  Drop it. Gate, spot and margins change nothing measurable here; gate costs ~2 s.
- **Every return scrape is in RETURN_DESCEND**, never over the barrier, 0.14-0.38 m east of the
  mark (x -2.14..-2.38), toward where M1 meets the barrier.
- **Cause: `goals.yaml` `return_walls` still has M1's old face** ([[-1.98, 0.65], [-1.88, 1.65]],
  near north-south). As rebuilt, M1 runs diagonally from the barrier junction ((-2.02, 0.48) to
  (-0.72, 1.38), and its swing varies 30-60 deg), so ReturnSpot fits points of the new M1 to the old
  line and shifts the spot up to 0.3 m toward the corner.
- **Fix tested (not applied):** return_walls with only the barrier entry. Scrapes 3 -> 2 and far
  shallower (worst -0.146 -> -0.027 m), mean return clearance +0.065 m, landing on average 0.07 m from
  the mark instead of 0.18 m, home time +0.5 s. Not deployable until `config/goals.yaml` changes (the
  drone reads it): **for the planner terminal**, which owns that file:
  `return_walls: [{wall: [[-2.90, 0.324], [-2.15, 0.402]], offset: 0.42}]`  (delete the M1 line).
- The two scrapes left are on the west side (x -2.6..-2.77), not the M1 corner: not yet looked at.
- 16 seeds: a direction, not a rate. Before adopting, re-run on 24+ seeds with the planner's final
  goals.yaml.

## 21:03-21:13: return success on 40 seeds (barrier-only return_walls goals copy, guard off)
Return success = of runs that reached the pad, home within tolerance with no wall contact on the way.
| Seeds | Reached pad | Returned clean | Rate | 95 % lower bound |
|---|---|---|---|---|
| 1-16, as flies (M1 in return_walls) | 11 | 8 | 72.7 % | 43 % |
| 1-16, barrier only | 11 | 9 | 81.8 % | 52 % |
| 17-40, barrier only (new seeds) | 16 | 15 | 93.8 % | 72 % |
| **1-40, barrier only** | **27** | **24** | **88.9 %** | **72 %** |
Every return reached home (no return failed); the 3 misses are shallow scrapes in the descent
(s8 -0.027, s15 -0.002, s22 -0.023 m). 95 % not reached; proving it needs >= 73 clean returns in a row.
Whole mission on 17-40: 16/24 (pre-pad failures, the planner's), home 76.5 s on average.
