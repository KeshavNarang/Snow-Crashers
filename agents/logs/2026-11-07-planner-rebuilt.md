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

## Window failures: diagnosed (second session)
goalrun's trace now records the current goal and the window goals' estimate, detection
and axis (`wgoals`, course frame). Seed 12 (fails) vs seed 2 (passes):
- The window IS detected in seed 12, its axis correct. window_front (0.9 m out on the
  axis) is at (0.59, -2.51).
- The mission's net fence (the `fence` corners in the trace) is turned ~7.5 deg CCW in seed
  12, so its south edge at x 0.59 is at y -2.48: window_front lies just outside the fence,
  the planner finds no path, speed 0 until "no progress". Seed 2's front is inside by 0.13 m.
- So the course heading the mission flies by is off by several degrees at the window. Not
  the rocks (below the planning band at cruise) and not the window search width (tried).
- Tried: fitting the heading/north net to the barrier only west of M3 (x -0.9..0.0, in case
  M3's plywood biased it): seed 12's fence identical to the decimal, 14/20 again. So the
  barrier fit is not correcting seed 12's heading at all; the error matches the sim's
  launch heading jitter (+-10 deg, placed by eye). In the old maze M1/M2/M3 (heading_walls)
  corrected it; on the rebuilt course nothing does before the window. Reverted.

## Next
1. Give the mission a heading reference on the rebuilt course: e.g. the C2a window wall
   itself, or the tunnel's straight sides (both unchanged by the rebuild and seen before the
   window), as heading_walls; check `fence` turns true in seed 12, then seeds 1-40.
2. Or let window_front sit inside the fence: place it along the measured axis only as far
   out as the fence allows (0.6-0.9 m), since the window's own axis is measured.
3. Seed 5: pad not seen (ladder by the lookouts?), not yet looked at.
