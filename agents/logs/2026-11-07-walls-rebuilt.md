# 2026-11-07: the maze as rebuilt, measured and put in the simulator

The organizers rebuilt the maze. Photos: `photos/Changed_Walls.jpeg` (from the west
net), `photos/New_Walls_Overhead1.png` and `New_Walls_Overhead2.png` (overhead, wide
angle; Overhead2 has the team's route drawn on it), `New_Walls_Side1.jpg` and
`New_Walls_Side2.jpg` (from outside the north net). The photos are not in git.

## What changed

| Wall | Before (scan, 2026-11-06) | Now |
|---|---|---|
| M1, first wall the drone goes around | on the barrier, ~6 deg off north-south | same base, swung **30-60 deg** east of north (team's description; the photos agree) |
| M2, second, hung from the north net | there | **gone** |
| M3, third, before the tunnel | long panel on the barrier at x ~1.0, north-south | **short panel, standing free ~0.3 m north of the barrier, centre (0.65, 1.13), 35.5 deg east of north**, stand feet and pavers on its north-west side |
| NW, new | - | **long panel across the north end of the first corridor**, centre (-2.00, 2.67), ~0.4 m in from the north net, turned -3.5 deg, stand feet toward the corridor (south) |

The route the team drew on Overhead2 is consistent with this: north from the launch,
east over M1's end (south of the NW panel), east past M3's north end, then south into
the tunnel's north mouth (the drawn hook maps to (2.73, 1.88)).

## How it was measured

Only floor points can be measured from the overhead photos: they are wide-angle shots from
about 2 m up, so the top of a 1.7 m wall lands far from its foot. For each overhead photo a
floor homography was fitted to the floor tape's corners (CAD `Guideline (zone 1)`;
residuals 3-8 px), then stand feet and pavers were clicked and mapped to the floor
(`2026-11-07-walls-rebuilt/measure.py`, which prints every number below). Check: the net's
north-west floor corner maps to (-2.98, 2.95), true (-3.05, 3.05).

| Point | Overhead1 | Overhead2 |
|---|---|---|
| M3 paver, south | (0.18, 0.98) | (0.18, 0.96) |
| M3 paver, north | (0.65, 1.62) | (0.63, 1.61) |
| NW post foot, west | (-2.61, 2.78) | (-2.62, 2.64) |
| NW post foot, east | (-1.36, 2.76) | (-1.42, 2.51) |
| End of the floor tape's last run | (1.39, 2.62) | (1.29, 2.64) |

- **M3:** the two photos agree to 2 cm. The pavers line up 36.3 / 34.7 deg east of north
  and are 0.79 m apart. A stand's pavers sit on its end feet, so 0.79 m fits a short panel
  (its end pavers ~0.9 m apart in the sim's stand); a long panel's would be ~1.5 m apart.
  The side photo (Side2) shows the same wall with its stand on the north-west side.
- **NW:** the post feet are 1.21-1.25 m apart, too far for a short panel; a long panel with
  its stands ~0.15 m in from its ends (M3's pavers are ~0.12 m in, against the sim's 6 cm)
  fits. The two photos disagree by ~0.15 m on how far south it stands (y 2.51-2.78).
- **Side photos:** a camera fit to Side2 (tape + net rail) put the post tops 600+ px off, so
  it was not used for numbers, only to confirm which wall is which and the stands' sides.
- **Uncertainty:** ~0.1 m and ~3 deg for M3; ~0.15 m and ~5 deg for NW; the panel lengths are
  inferred from stand spacing, not seen end to end. Worth a tape measure on site: M3's two ends
  to the barrier, NW's distance from the north net.

`2026-11-07-walls-rebuilt/overlay_O1.jpg` and `overlay_O2.jpg`: the sim's walls at severity 0
(lines), where their stands put pavers (crosses) and the measured pavers (green circles),
drawn onto the photos' floors.

## In the simulator

`goalnav/localsim/messy_course.py`, `LAYOUT = 'rebuilt'` (default) or `'scan'` (the course as
scanned, M2 and all); `generate(..., layout=...)`, `goalrun --layout scan|rebuilt`. The
rebuild is drawn after the walls as scanned (`Sampler.rebuild`), with the usual severity
jitter: M1 30-60 deg (45 at severity 0), M3 and NW +-0.1 m and +-4 deg on the measured pose.

**Not modelled (left for now, at the team's request):** the floor tape. The photos show its
last run now reaching x ~1.3, past the CAD's 0.33. In the sim the tape and the lines tied to it
(CP1, gate A) still follow the walls as scanned, so they are the same in both layouts.
Consequence: the NW panel stands ~4 cm north of the CP1 line (the tape's turn), so the whole drone
can no longer cross CP1 there as the rules define it. How the organizers will judge CP1 now is open.

Evals: `goalnav/localsim/test_messy_course_layout.py` (9 tests, including that M3's simulated
pavers land within 0.12 m of the measured ones and that every rebuilt course validates).

## Sim runs

`goalrun --seed 1 --severity 0.7 --profile safe --duration 200`, the planner unchanged:

| Layout | Result | Checkpoints | Touches |
|---|---|---|---|
| `scan` | ABORTED at the pad ("pad not seen from any lookout"), contact with a wall at 79 s | 4 (5.3, 24.2, 38.2, 52.5 s) | 1 |
| `rebuilt` | ABORTED at 38 s, "no progress toward first_corridor", landed and disarmed | 0 | 0 |

The `scan` run matches main's run of the same seed (same checkpoint times, same crash at the pad,
which is an older, separate problem), so switching back really gives the old course. On the
rebuilt course the planner (`config/goals.yaml`, still the old maze) flies up the first corridor
and stops: its CP1 goal is in or behind the NW panel. Next: CP1's definition (organizers), then the
planner's goal chain for the new maze (no `m2_end`/`corridor_2`, M1 and M3 swung, NW panel).
