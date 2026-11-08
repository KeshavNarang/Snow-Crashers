# 2026-11-07: the planner on the rebuilt course (agent/planner-rebuilt)

Goal (team): round the first wall, through the tunnel, through the window, land on
the pad without contact within 120 s; tape and checkpoints do not matter. RGB camera
assumed (goalrun --assume-colour-pad).

## Change
`config/goals.yaml`: the old maze goals (CP1 reach, corridors 2-3, M2's end, gate A)
replaced by corridor_1_top -> m1_end (gap, south of the new NW panel) -> north_run
(north of M3) -> the unchanged tunnel / window / pad chain. heading_walls and
net_anchors keep only the barrier (M1 and M3 moved, M2 gone).

## Sim results (seeds 1-20, severity 0.7, safe profile)
Success = referee finish_s (whole footprint resting in the pad ring) <= 120 s with no
touch, crash or ground contact before it.

| Pad camera | Success |
|---|---|
| mono (as main is now) | 0/20: every run clears maze + tunnel; pad not detected |
| RGB (--assume-colour-pad) | 14/20, landed in 48-62 s |

Every run cleared the maze and the tunnel (seed 5 and 9 have a touch at t=0 on the
launch, the known attempt-1 launch spot by M1's stand).

## The 6 failures (RGB)
- Window, seeds 9, 11, 12, 16, 19: the drone waits 0.7-0.95 m east of the window wall and
  0.4-0.5 m north of the planned window_front, speed limit 0, until "no progress toward
  window_front" (or touches the wall). advance_if_passed will not pass window_front before
  the window is detected. Not explained by the wall's turn (66-74 deg in passing and failing
  seeds alike) or by the rock near window_front.
- Tried: window `across` 0.40 -> 0.60 (wider detection search). Identical 14/20, same
  failures: the search width is not it. Reverted.
- Pad, seed 5: "pad not seen from any lookout"; the ladder now stands in the SW corner by
  the pad lookouts (not checked).

## Next
Log the window goal's estimate/detected state per frame in goalrun's trace (it is not
recorded now) for seed 12 to see why the window is never detected; check seed 5's lookouts
against the ladder. Then re-run seeds 1-20 (and 21-40: 20 runs bound 70% only to ~50-85%).
