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

## Session 2 (19:15-19:55): drone-measured sims, batch fixes, a faster ray march, sim-to-real

Branch rebased onto `main` 607b851 (calibration fix); course = `main`'s (no ladder, old maze).

### What the sim now assumes (`goalnav/localsim/drone_measured.yaml`)
`--scenario drone` = depth 4.0 Hz, 234 ms old (goalnav stereo_node, 2 threads, measured
2026-11-07); `drone_slow` 2.8 Hz / 0.4 s; `drone_drift` x2 odometry drift (not measured);
`sim_default` = goalrun's own 4.5 Hz / 0.13 s, kinder than the drone.

### Drone scenario, 24 seeds, severity 0.7, --assume-colour-pad
| Setting | Finish | 95 % lower bound | Pad mean s | Home mean / max s | Runs with a touch | Return scrapes |
|---|---|---|---|---|---|---|
| safe (deployable) | 23/24 | 0.80 | 66.5 | 91.2 / 110.7 | 5 | s15 |
| candidate A (--tune) | 23/24 | 0.80 | 71.3 | 95.2 / 101.3 | 5 | - |

- Both fail **s18**: 1 s along the side of the **window** during the traverse. The same seed with
  the sim's default latency passes (1 touch): the old defaults overstated the pass rate.
- A under drone latency: no seed fixed, +4.8 s, minimum clearance lower on 14/23 seeds. Its gain at
  the default latency does not transfer. **Dropped.**
- 18 runs first died of MemoryError (48 at once on 32 GB, every new course voxelising); they were
  re-flown. The batch now keeps such runs apart (`sim_errors`), retries them, defaults to 16 jobs.

### Found in the referee
After the pad, `referee.update` only watches for a hit above `crash_speed`, and goalrun's crash check
is off: wall scrapes on the way home show in neither `touches` nor `crashed`. The batch reports them
from clearance (`return_scrape_runs`). Not changed in the referee (owned by the course work).

### Faster sim
`MessyWorld.cast` (50 % of a run) packs solid / mesh / near into one flat uint8 grid. Frozen-copy eval
`goalnav/test/test_cast_speed.py`: bit-identical hits, 1.3-1.5x on the march; end to end, identical
traces (seeds 1, 2) and best-of-3 wall time 51.8 -> 41.6 s (**1.25x**) for a 60 s flight. Measured
while another session ran sims, so noisy.

### Sim to real: the process, and its gaps
1. **Measure** on the drone (read-only: `tools/drone_tools/topic_rates.py` on `/goalnav/stereo/points`
   for rate and age) -> update `drone_measured.yaml` with the source log.
2. **Gate in the sim** with exactly what flies: `python -m localsim.batch --scenario drone --seeds 1-100`
   with no `--tune`. `summary.json` says `deployable` and records commit, profile hash and flags.
3. **Only profile changes fly.** The drone runs `ros2 launch drone_ref goalnav.launch.py` with no
   arguments at power-up (`skybetav1/container_scripts/run_competitor.sh`), so a `--tune` result counts
   only once it is written into `config/goalnav_profiles.yaml` and re-gated. Staged tests (`stop_after`)
   hit the same wall: there is no way to pass `tuning` at power-up.
   *Proposal (not done; drone code, needs the team):* the launch file reads its default
   profile/tuning from one committed `config/deploy.yaml`, and the batch reads the same file, so
   sim and drone cannot disagree and a staged test is a one-line change plus a power cycle.
4. **Fly, pull the log, compare** (`tools/drone pull team_ws/logs/<run>`). Missing: no tool reads the
   MCAP bag back (no `mcap` module here) to compare real depth age/rate and the flown path with the sim.

Open gaps: no colour camera on the drone stack (every result above assumes one; 0/24 without);
odometry drift not measured; return-leg contact not judged by the referee.

### Next
- Re-run the drone gate on the rebuilt course (`agent/sim-walls`, ladder merged in): it changes the
  maze (M2 gone), so the corridor numbers above are out of date.
- Window centring under 0.23 s latency (s18); 100+ seeds for a real bound (381 clean runs for 99 %).
