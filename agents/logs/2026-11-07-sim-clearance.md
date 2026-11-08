# 2026-11-07: sim success rate and collision clearance (target: tunnel radius / 4)

Branch `agent/sim-clearance`, based on `agent/sim-ladder` 4766235 (course with the
stepladder). Timebox 30 min. No navigator or profile change was made: the evidence was
too thin for one (see "Decision").

## The metric (new, evals first)

- `goalnav/localsim/clearance.py`: the gap from the drone's **surface** (the contact
  judge's 3 layers: core 0.13, arms 0.20, props 0.29 m) to the nearest contact voxel, and
  the props' rim to each net. It is negative exactly when `goalrun.drone_contacts` sees a
  touch (`goalnav/test/test_clearance.py`, 6 tests). Sampled at the trace's 10 Hz while the
  feet are > 0.15 m up.
- Target `TARGET_M` = `TUBE['hw']` / 4 = 0.7366 / 4 = **0.184 m** from the surface,
  i.e. 0.474 m from the centre. The safe profile *prefers* 0.42 from the centre
  (0.13 surface) and never plans under 0.33 (0.04).
- `goalnav/localsim/batch.py`: flies seeds in parallel, writes `scores.jsonl` and
  `summary.json` (success rate, mean/max finish, touch runs, clearance, window clearance
  reported apart). `--summarise DIR` re-scores existing runs.

## Results (severity 0.7, profile safe, seeds 1-16, 200 s)

**Without `--assume-colour-pad` (the default): 0/24 finish.** All pass 4 checkpoints,
find no mat (no colour camera: the known hardware blocker), abort and descend where they
are, often into a wall. Every number below uses `--assume-colour-pad`, as the ladder log did.

| Setting | Finish | Mean / max s | Runs with a touch | Touches | Median min clearance | Median window clearance |
|---|---|---|---|---|---|---|
| safe (baseline) | 16/16 | 65.5 / 74.5 | 6 | 8 | 0.038 m | 0.076 m |
| A: radius 0.48, soft 0.9, repel_reach 0.50 | 16/16 | 68.4 / 73.4 | 3 | 3 | 0.060 m | 0.082 m |
| B: A + soft_weight 20, cross_track_gain 2.0, turn_speed 0.2 | 16/16 | 73.9 / 86.2 | 3 | 4 | 0.062 m | 0.097 m |

No run in any setting met 0.184 m. Where each run's minimum falls: the descent onto the
launch mark on return (about half), the launch corridor at takeoff, m1_end (net), the
corridor_2/corridor_3/m2_end hairpins, gate_a, the tunnel once.

## The target is not reachable everywhere on this course

The best clearance over any point across each passage (z 1.15, tunnel 0.85), from the
course itself, not the flight (seeds 8, 9, 13):

| Passage | Best possible surface clearance |
|---|---|
| Launch mark (y 0.75) | 0.05-0.12 m |
| Launch corridor (y 1.05) | 0.26-0.29 |
| corridor_2 | 0.33-0.39 |
| m2_end gap | 0.17-0.19 |
| gate_a | 0.14-0.23 |
| Tunnel (y 1.1) | 0.23-0.43 |
| Window (0.80-0.95 m wide, 0.58 m rotor span) | 0.11-0.19 |

So the 0.184 m target is impossible at the launch mark, the window, usually m2_end, and
often gate_a, whatever the tuning. It is reachable in the corridors and the tunnel.

## Decision

Neither candidate was adopted. Both cut touches (8 -> 3/4) and raise the median minimum,
but 16 seeds can't show 99 % (16/16 is consistent with ~85 %), and A's worst run (s15,
-0.099 m on the return descent) is worse than the baseline's worst. Next:

1. Agree a reachable target: 0.184 m where the course allows it, and centred in the
   passages that are narrower (launch mark, window, m2_end, gate_a).
2. Run A on 100+ seeds at severities 0.7 and 1.0 (`python -m localsim.batch --seeds
   1-100 --jobs 30 --assume-colour-pad --tune radius=0.48 --tune soft=0.9 --tune
   repel_reach=0.50`, ~10 min on 32 cores).
3. The return's descent onto the launch mark touches in a third of runs: look at that leg
   (it is flown blind, the camera faces ahead).
