"""Evals for geometry.py: is the published depth in the frame its camera_info claims?

    python3 -m unittest -v test_geometry

Context: with stereo.i_aligned false the OAK publishes depth in StereoDepth's own reference frame, while its
camera_info describes the left camera. If that reference is actually the rectified RIGHT camera, every obstacle is
offset by the 7.5 cm baseline. These functions let a host stereo match decide which frame the depth is really in.
The SGBM part needs OpenCV and is tested on the drone (skipped here when cv2 is missing).
"""
import math
import unittest

import numpy as np

import geometry as g

K = (399.0, 399.0, 320.0, 200.0)
SIZE = (640, 400)


def plane_depth(z=2.0, size=SIZE):
    return np.full((size[1], size[0]), z, float)


def textured_depth(size=SIZE, seed=0):
    rng = np.random.default_rng(seed)
    coarse = rng.uniform(0.6, 4.0, (size[1] // 20 + 1, size[0] // 20 + 1))
    return np.kron(coarse, np.ones((20, 20)))[:size[1], :size[0]]


def rot_y(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])


class ScaleK(unittest.TestCase):
    def test_half_size(self):
        k = g.scale_k((638.0, 638.0, 527.4, 317.6), (1024, 640), (512, 320))
        self.assertEqual(k, (319.0, 319.0, 263.45, 158.55))    # pixel-centre convention: (c + 0.5) * s - 0.5

    def test_identity(self):
        self.assertEqual(g.scale_k(K, SIZE, SIZE), K)


class Reproject(unittest.TestCase):
    def test_identity_keeps_the_depth(self):
        d = textured_depth()
        out = g.reproject_depth(d, K, np.eye(3), np.zeros(3), K, SIZE)
        m = np.isfinite(out)
        self.assertGreater(m.mean(), 0.99)
        self.assertTrue(np.allclose(out[m], d[m]))

    def test_baseline_translation_shifts_by_the_disparity(self):
        # a camera 7.5 cm to the right sees a fronto-parallel plane at 2 m shifted left by fx*B/Z = 14.96 px
        out = g.reproject_depth(plane_depth(2.0), K, np.eye(3), np.array([-0.075, 0.0, 0.0]), K, SIZE)
        cols = np.where(np.isfinite(out[200]))[0]
        self.assertAlmostEqual(cols.max(), 639 - 15, delta=1)
        self.assertTrue(np.allclose(out[200, cols], 2.0))

    def test_small_rotation_matches_the_analytic_plane(self):
        # depth of the plane Z=2 seen by a camera rotated 1 deg about y: z' = (R X)_z
        r = rot_y(1.0)
        out = g.reproject_depth(plane_depth(2.0), K, r, np.zeros(3), K, SIZE)
        v, u = 200, 400
        ray = np.array([(u - K[2]) / K[0], (v - K[3]) / K[1], 1.0])
        # the point on Z=2 whose image in the rotated camera is (u, v): X = s * R^T ray with (R^T ray)_z * s = 2
        back = r.T @ ray
        s = 2.0 / back[2]
        self.assertAlmostEqual(out[v, u], s * ray[2], delta=0.01)

    def test_nearest_surface_wins_whichever_is_written_last(self):
        d = plane_depth(3.0)
        d[:, 300:340] = 1.0                                      # a near post (1 m) in front of a far wall (3 m)
        # shifts: post fx*B/1 = 29.9 px, wall 10.0 px. Moving the camera right (t = -B) the post lands on columns
        # 270-309 and the left wall on -10..289; moving it left (t = +B) the post lands on 330-369 and the right
        # wall on 350-649. In the second case the wall's points come AFTER the post's in scan order, so only a real
        # depth buffer (not "last write wins") keeps the post where they overlap.
        out = g.reproject_depth(d, K, np.eye(3), np.array([-0.075, 0.0, 0.0]), K, SIZE)
        self.assertTrue(np.all(out[200, 272:288] == 1.0), out[200, 272:288])
        out = g.reproject_depth(d, K, np.eye(3), np.array([0.075, 0.0, 0.0]), K, SIZE)
        self.assertTrue(np.all(out[200, 352:368] == 1.0), out[200, 352:368])

    def test_invalid_pixels_stay_invalid(self):
        d = plane_depth(2.0)
        d[:, :320] = np.nan
        out = g.reproject_depth(d, K, np.eye(3), np.zeros(3), K, SIZE)
        self.assertTrue(np.isnan(out[:, :300]).all())


class Identify(unittest.TestCase):
    def test_picks_the_matching_frame(self):
        left = textured_depth()
        right = g.reproject_depth(left, K, np.eye(3), np.array([-0.075, 0.0, 0.0]), K, SIZE)
        r = g.identify_frame(left, {'left': left, 'right': right})
        self.assertEqual(r['best'], 'left')
        r2 = g.identify_frame(right, {'left': left, 'right': right})
        self.assertEqual(r2['best'], 'right')

    def test_no_candidate_close_enough_is_none(self):
        left = textured_depth()
        r = g.identify_frame(left * 1.2, {'left': left})
        self.assertIsNone(r['best'])
        self.assertAlmostEqual(r['scores']['left']['median_rel'], 0.2, places=6)

    def test_the_closest_of_several_acceptable_frames_wins(self):
        left = textured_depth()
        r = g.identify_frame(left, {'close': left * 1.02, 'exact': left, 'far': left * 1.025})
        self.assertEqual(r['best'], 'exact')

    def test_scores_are_reported_for_every_candidate(self):
        left = textured_depth()
        r = g.identify_frame(left, {'a': left, 'b': left * 1.5})
        self.assertEqual(set(r['scores']), {'a', 'b'})


@unittest.skipUnless(g.HAVE_CV2, 'OpenCV not installed here; runs on the drone')
class HostStereo(unittest.TestCase):
    def test_constant_disparity_gives_the_right_depth_in_both_frames(self):
        rng = np.random.default_rng(1)
        h, w, d = 200, 320, 12
        tex = (rng.random((h, w + d)) * 255).astype(np.uint8)
        # a point at column x in the left image is at x - d in the right one (positive disparity d)
        left, right = tex[:, :w].copy(), tex[:, d:d + w].copy()      # right(x') = tex(x' + d) = left(x' + d)
        fx, base = 300.0, 0.075
        dl, dr = g.host_depth(left, right, fx, base, num_disp=32, block=5)
        expect = fx * base / d
        for depth in (dl, dr):
            m = np.isfinite(depth)
            self.assertGreater(m.mean(), 0.5)
            self.assertAlmostEqual(float(np.median(depth[m])), expect, delta=0.02 * expect)


if __name__ == '__main__':
    unittest.main()
