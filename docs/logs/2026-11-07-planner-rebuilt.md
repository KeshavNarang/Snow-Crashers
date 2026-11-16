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

## Step 1: heading references on the rebuilt course (third session)
heading_walls += the long panel across the first corridor's north end and the rebuilt M3,
at their measured poses (+-3-5 deg each); the barrier stays. Seeds 1-20, RGB:

| | Success | Heading error at the window (fence turn) |
|---|---|---|
| before | 14/20 | up to ~10 deg (seed 12: 7.5) |
| after | 15/20 | within +-3 deg in 16/20; seeds 11 5.8, 12 8.6, 14 -4.2 |

Window now passed in seeds 11, 16, 19 (and 9, which fails only on a t=0 launch contact).
Still failing: 12 (heading 8.6 deg: the added walls did not pull it in), 14 (crash at the
window, -4.2 deg), 6 (NEW: brushed the tunnel's north mouth entering ~0.2 m east of
centre at t~22 s; clean before, so plausibly the heading change moving the tunnel's
expected place), 5 and 9 (contact at t=0 on the launch mark: the launch spot within rotor
reach of M1's stand, a sim layout issue seen before the rebuild; 5 also touched at landing).
14 -> 15 of 20 is within the noise of 20 runs; the heading improvement is the measured gain.
Note: MissionParams.heading_until is 'gate_a', a goal the rebuilt chain no longer has, so
the heading now keeps learning all mission.

## Next
1. (done above; superseded) Give the mission a heading reference on the rebuilt course: e.g. the C2a window wall
   itself, or the tunnel's straight sides (both unchanged by the rebuild and seen before the
   window), as heading_walls; check `fence` turns true in seed 12, then seeds 1-40.
2. Let window_front sit inside the fence: place it along the measured axis only as far
   out as the fence allows (0.6-0.9 m), since the window's own axis is measured.
3. Seed 5: pad not seen (ladder by the lookouts?), not yet looked at.

## Final session (to the 21:30 close): 18/20
Judge corrected: contacts are counted only before the mission's ON_PAD (trace sim time);
the referee restarts its clock for the optional return, so return-flight contacts at the
launch had shown up as "t=0" touches (seeds 5, 9 earlier). Changes kept:
- window approach 0.9 -> 0.75 m (`WindowConfig.approach`, `window_front` moved to match): the
  line-up point 0.15 m further from the south net; jambs still in view (0.54 m half-width at
  0.75 m vs 0.40-0.48). Seeds 12 and 14 now pass the window; every run passes the window.
- tunnel_exit -0.80 -> -1.05: stay low until clear of the south mouth's hanging tarp.
- localsim validate: launch floor clearance BODY_RADIUS + 0.01 -> + 0.05 (seed 5's real contact
  at takeoff).

Seeds 1-20, severity 0.7, RGB pad: **18/20 (90%)**, landed 45.7-66.0 s, no contact before landing.
Still failing: seed 6 (brushes the north mouth's tarp entering 0.33 m east of centre: the
entry follows tunnel_inside's estimate) and seed 19 (contact at (1.89, -1.0) heading for
south_room after the tunnel). Tried and reverted: south_turn heading 200 -> -90 (17/20,
seed 19 unchanged). 20 seeds cannot tell 90% from 95%: run 40+.
