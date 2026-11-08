"""fit_floor speed-up: same answers, measurably faster.

`reference_fit_floor` below is a FROZEN copy of the implementation before the speed-up (full SVD of the
inlier set). The new fit_floor must (1) return the same plane, (2) be clearly faster in the same process
on the same data, (3) survive degenerate inputs. Profile that motivated it (laptop, 30k points):
SVD 58 % of a fit, scoring the 80 candidates 2.9 ms; np.linalg.svd on an N x 3 matrix builds an N x N U.
"""
import math
from pathlib import Path
import sys
import time
import unittest

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))
from overwall_nav.stereo import StereoRig, extrinsics_from_tf, fit_floor   # noqa: E402
import synthetic                                                            # noqa: E402

# ---- frozen reference (do not edit: it is the behaviour being preserved) --------------------------
def reference_fit_floor(points_cam, iterations=80, tolerance=0.02, seed=0, min_inliers=0.2,
              pitch_deg=None, max_tilt_deg=12.0):
    """RANSAC floor plane in the camera frame -> dict(pitch_deg, roll_deg, height_m, inliers)
    for the camera relative to that plane, or None. Pitch is positive DOWN, roll positive
    right side down; height is the camera's distance above the plane.

    ``pitch_deg`` (the camera's mount pitch, degrees down) turns on an orientation prior: a candidate
    plane is only considered if its pitch is within ``max_tilt_deg`` of it and its roll within the same
    bound. Without it the plane with the most inliers wins, and on a glossy table that is sometimes the
    face of a nearby box (found on skybeta-03), which gives a wrong ground height. A real mount error
    larger than ``max_tilt_deg`` reads as no fit, not as a wrong one."""
    pts = np.asarray(points_cam, dtype=float)
    if len(pts) < 50:
        return None
    rng = np.random.default_rng(seed)
    if len(pts) > 3000:
        pts = pts[rng.choice(len(pts), 3000, replace=False)]
    best, best_count = None, 0
    # Candidate planes from random point triples, generated and gated in one vectorised step. With the
    # orientation prior most triples are not floor-like and are dropped here for the price of a cross
    # product; only the first ``iterations`` survivors are scored, so the gate does not starve the
    # search. Without the prior every candidate survives and this is the old loop.
    tries = iterations * (8 if pitch_deg is not None else 1)
    tri = rng.integers(0, len(pts), (tries, 3))
    a_, b_, c_ = pts[tri[:, 0]], pts[tri[:, 1]], pts[tri[:, 2]]
    normals = np.cross(b_ - a_, c_ - a_)
    norms = np.linalg.norm(normals, axis=1)
    ok = norms > 1e-9
    normals[ok] /= norms[ok][:, None]
    if pitch_deg is not None:
        flip = np.where((-(normals * a_).sum(axis=1) < 0)[:, None], -1.0, 1.0)
        nn = normals * flip
        pitch = np.degrees(np.arctan2(-nn[:, 2], -nn[:, 1]))
        roll = np.degrees(np.arctan2(-nn[:, 0], np.hypot(nn[:, 1], nn[:, 2])))
        ok &= (np.abs(pitch - pitch_deg) <= max_tilt_deg) & (np.abs(roll) <= max_tilt_deg)
    for k in np.flatnonzero(ok)[:iterations]:
        n, a = normals[k], a_[k]
        count = np.count_nonzero(np.abs((pts - a) @ n) < tolerance)
        if count > best_count:
            best, best_count = (n, a), count
    if best is None or best_count < min_inliers * len(pts):
        return None
    n, a = best
    inliers = pts[np.abs((pts - a) @ n) < tolerance]
    centre = inliers.mean(axis=0)
    n = np.linalg.svd(inliers - centre)[2][2]
    d = -float(n @ centre)                      # plane: n . X + d = 0
    if d < 0:                                    # orient n from the floor up towards the camera
        n, d = -n, -d
    # Level up direction in camera coordinates is (0, -cos p, -sin p) for pitch p down.
    pitch = math.degrees(math.atan2(-n[2], -n[1]))
    roll = math.degrees(math.atan2(-n[0], math.hypot(n[1], n[2])))
    return {'pitch_deg': round(pitch, 2), 'roll_deg': round(roll, 2), 'height_m': round(d, 3),
            'inliers': round(len(inliers) / float(len(pts)), 2)}
# ---- end frozen reference ---------------------------------------------------------------------------

PITCH = 27.0
SCENES = {'floor+wall 1.5 m': dict(cam_height=0.85, wall_distance=1.5, wall_top=1.0),
          'table+box 0.45 m': dict(cam_height=0.10, wall_distance=0.45, wall_top=0.35),
          'floor only': dict(cam_height=0.5)}


def scene_points():
    r, t = extrinsics_from_tf(synthetic.T_LR, synthetic.Q_LR)
    rig = StereoRig(synthetic.K_LEFT, synthetic.K_RIGHT, synthetic.SIZE, r, t)
    return {name: rig.points(*synthetic.pair(**kw))[0] for name, kw in SCENES.items()}


def best_ms(fn, runs=25):
    times = []
    for _ in range(runs):
        t0 = time.perf_counter()
        fn()
        times.append(time.perf_counter() - t0)
    return min(times) * 1e3


class SameAnswers(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.pts = scene_points()

    def test_matches_the_frozen_reference_on_every_scene_and_seed(self):
        for name, pts in self.pts.items():
            for seed in range(12):
                for prior in (None, PITCH):
                    new = fit_floor(pts, seed=seed, pitch_deg=prior)
                    old = reference_fit_floor(pts, seed=seed, pitch_deg=prior)
                    if old is None:
                        self.assertIsNone(new, (name, seed, prior))
                        continue
                    self.assertIsNotNone(new, (name, seed, prior))
                    for key, tol in (('pitch_deg', 0.02), ('roll_deg', 0.02), ('height_m', 0.001), ('inliers', 0.0)):
                        self.assertAlmostEqual(new[key], old[key], delta=tol, msg=(name, seed, prior, key, new, old))


class Faster(unittest.TestCase):
    def test_at_least_one_and_a_half_times_faster_than_the_reference(self):
        pts = scene_points()['floor+wall 1.5 m']
        self.assertGreater(len(pts), 10000)
        new = best_ms(lambda: fit_floor(pts, seed=3, pitch_deg=PITCH))
        old = best_ms(lambda: reference_fit_floor(pts, seed=3, pitch_deg=PITCH))
        self.assertLess(new, old / 1.5, f'new {new:.2f} ms vs reference {old:.2f} ms (best of 25)')


class Degenerate(unittest.TestCase):
    def test_too_few_points(self):
        self.assertIsNone(fit_floor(np.zeros((10, 3)), pitch_deg=PITCH))

    def test_all_points_identical(self):
        self.assertIsNone(fit_floor(np.tile([0.1, 0.2, 1.0], (500, 1)), pitch_deg=PITCH))

    def test_collinear_points(self):
        t = np.linspace(0.3, 2.0, 500)[:, None]
        self.assertIsNone(fit_floor(t * np.array([0.0, 0.3, 1.0]), pitch_deg=PITCH))

    def test_a_perfect_plane_is_recovered_exactly(self):
        rng = np.random.default_rng(1)
        f, l = rng.uniform(0.3, 2.0, 800), rng.uniform(-0.5, 0.5, 800)
        level = np.stack([f, l, np.full(800, -0.4)], 1)
        from overwall_nav.stereo import mount_matrix
        fit = fit_floor(level @ mount_matrix(PITCH), pitch_deg=PITCH)
        self.assertAlmostEqual(fit['pitch_deg'], PITCH, delta=0.05)
        self.assertAlmostEqual(fit['height_m'], 0.4, delta=0.002)

    def test_the_reported_normal_points_up_regardless_of_sign(self):
        """The plane normal from the covariance has an arbitrary sign: the fit must still report a
        positive height and the same pitch for the mirrored cloud."""
        rng = np.random.default_rng(2)
        f, l = rng.uniform(0.3, 2.0, 800), rng.uniform(-0.5, 0.5, 800)
        level = np.stack([f, l, np.full(800, -0.4)], 1)
        from overwall_nav.stereo import mount_matrix
        cam = level @ mount_matrix(PITCH)
        fit = fit_floor(cam, pitch_deg=PITCH)
        self.assertGreater(fit['height_m'], 0)
        self.assertAlmostEqual(fit['pitch_deg'], PITCH, delta=0.05)


if __name__ == '__main__':
    unittest.main()
