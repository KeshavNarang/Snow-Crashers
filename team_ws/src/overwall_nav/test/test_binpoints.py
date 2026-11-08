"""bin_points speed-up: same cells, same order, much faster.

Profile (laptop, one depth frame after the matcher): bin_points 8.9 ms of 20.3 ms -- np.unique(axis=0) on
an N x 2 integer array is a slow structured sort. `reference_bin_points` is a FROZEN copy of the version
before the speed-up; the new bin_points must return exactly the same list (cells, counts and ORDER, which
consumers see in the published JSON), be clearly faster in the same process, and survive edge cases.
"""
from pathlib import Path
import sys
import time
import unittest

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))
from overwall_nav.perception import BIN, bin_points   # noqa: E402
from overwall_nav.stereo import StereoRig, extrinsics_from_tf, level   # noqa: E402
import synthetic                                                       # noqa: E402

# ---- frozen reference (do not edit: it is the behaviour being preserved) --------------------------
def reference_bin_points(flu, corridor=0.4, min_count=3):
    """N x 3 level-frame points (F, L, U) -> list of [fi, ui, count] cells (|L| < corridor)."""
    flu = np.asarray(flu, dtype=float).reshape(-1, 3)
    flu = flu[np.abs(flu[:, 1]) < corridor]
    if len(flu) == 0:
        return []
    idx = np.floor(flu[:, [0, 2]] / BIN).astype(np.int64)
    keys, counts = np.unique(idx, axis=0, return_counts=True)
    keep = counts >= min_count
    return [[int(f), int(u), int(n)] for (f, u), n in zip(keys[keep], counts[keep])]
# ---- end frozen reference ---------------------------------------------------------------------------


def clouds():
    rng = np.random.default_rng(7)
    out = {}
    out['uniform'] = np.column_stack([rng.uniform(0, 6, 20000), rng.uniform(-1, 1, 20000), rng.uniform(-1.5, 0.5, 20000)])
    out['wall+floor'] = np.concatenate([
        np.column_stack([np.full(3000, 1.5) + rng.normal(0, .01, 3000), rng.uniform(-.4, .4, 3000), rng.uniform(-.9, .2, 3000)]),
        np.column_stack([rng.uniform(.3, 5, 9000), rng.uniform(-.4, .4, 9000), -.85 + rng.normal(0, .01, 9000)])])
    out['negative coords'] = rng.uniform(-3, 3, (5000, 3))
    out['far outliers'] = np.concatenate([out['wall+floor'], [[900.0, 0.0, 0.0], [0.5, 0.1, -750.0], [-400.0, 0.0, 300.0]]])
    out['huge spread'] = np.concatenate([out['wall+floor'], [[1e6, 0.0, 0.0], [0.0, 0.0, -1e6]]])
    out['single point'] = np.array([[1.0, 0.0, 0.0]])
    out['one cell'] = np.tile([[1.02, 0.1, 0.51]], (50, 1)) + rng.normal(0, 0.001, (50, 3))
    r, t = extrinsics_from_tf(synthetic.T_LR, synthetic.Q_LR)
    rig = StereoRig(synthetic.K_LEFT, synthetic.K_RIGHT, synthetic.SIZE, r, t)
    pts = rig.points(*synthetic.pair(cam_height=0.85, wall_distance=1.5, wall_top=1.0))[0]
    out['real stereo scene'] = level(pts, 27.0)
    return out


class SameAnswers(unittest.TestCase):
    def test_identical_to_the_frozen_reference(self):
        for name, pts in clouds().items():
            for corridor in (0.4, 1.0, 0.05):
                for min_count in (1, 3, 25):
                    self.assertEqual(bin_points(pts, corridor, min_count), reference_bin_points(pts, corridor, min_count),
                                     (name, corridor, min_count))

    def test_accepts_lists_and_other_shapes_like_before(self):
        for raw in ([[1.0, 0.0, 0.0]] * 5, [], np.zeros((0, 3)), np.array([1.0, 0.0, 0.0] * 4)):
            self.assertEqual(bin_points(raw), reference_bin_points(raw), repr(raw)[:40])

    def test_cell_edges_and_negative_values_bin_like_floor(self):
        edge = np.array([[BIN * k, 0.0, BIN * j] for k in (-2, -1, 0, 1, 2) for j in (-2, -1, 0, 1)] * 4)
        self.assertEqual(bin_points(edge, 0.4, 1), reference_bin_points(edge, 0.4, 1))


class Edge(unittest.TestCase):
    def test_empty_when_nothing_is_in_the_corridor(self):
        far = np.array([[1.0, 5.0, 0.0]] * 10)
        self.assertEqual(bin_points(far), [])

    def test_min_count_drops_sparse_cells(self):
        pts = np.array([[1.0, 0.0, 0.0]] * 2 + [[2.0, 0.0, 0.0]] * 5)
        self.assertEqual(bin_points(pts, 0.4, 3), [[40, 0, 5]])

    def test_output_is_plain_python_ints_and_json_ready(self):
        import json
        json.dumps(bin_points(clouds()['wall+floor']))


class Faster(unittest.TestCase):
    def test_at_least_three_times_faster_than_the_reference(self):
        pts = clouds()['real stereo scene']
        self.assertGreater(len(pts), 10000)
        def best(fn, runs=25):
            times = []
            for _ in range(runs):
                t0 = time.perf_counter()
                fn()
                times.append(time.perf_counter() - t0)
            return min(times) * 1e3
        new, old = best(lambda: bin_points(pts, 0.4)), best(lambda: reference_bin_points(pts, 0.4))
        self.assertLess(new, old / 3.0, f'new {new:.2f} ms vs reference {old:.2f} ms (best of 25)')


if __name__ == '__main__':
    unittest.main()
