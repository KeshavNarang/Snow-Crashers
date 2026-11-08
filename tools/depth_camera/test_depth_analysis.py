"""Evals for depth_analysis.py (numpy only; no drone, ROS or OpenCV):

    cd tools/depth_camera && python3 -m unittest -v test_depth_analysis

Each test builds data with a KNOWN answer and requires the analysis to recover it (or to say clearly that
it cannot), because these functions decide whether on-device depth is accepted and whether VIO regressed.
"""
import math
import struct
import unittest

import numpy as np

import depth_analysis as da

K = (399.0, 399.0, 320.0, 200.0)                      # fx, fy, cx, cy at 640x400 (fx 638 at 1024x640, scaled)


def floor_depth(h=0.11, pitch_deg=27.0, size=(640, 400), k=K, max_range=8.0):
    """Depth (m, NaN = no return) of a flat floor h metres below a camera pitched down by pitch_deg."""
    fx, fy, cx, cy = k
    w, hgt = size
    u, v = np.meshgrid(np.arange(w, dtype=float), np.arange(hgt, dtype=float))
    rx, ry = (u - cx) / fx, (v - cy) / fy
    th = math.radians(pitch_deg)
    denom = -math.cos(th) * ry - math.sin(th)          # up . ray, up = (0, -cos, -sin) in the optical frame
    with np.errstate(divide='ignore', invalid='ignore'):
        z = np.where(denom < 0, -h / denom, np.nan)
    z[(z <= 0) | (z > max_range)] = np.nan
    return z


def blocks(shape=(100, 160), block=8, seed=0):
    """Piecewise-constant textured depth, so a horizontal shift is identifiable."""
    rng = np.random.default_rng(seed)
    hb, wb = shape[0] // block + 1, shape[1] // block + 1
    coarse = rng.uniform(0.5, 3.0, (hb, wb))
    return np.kron(coarse, np.ones((block, block)))[:shape[0], :shape[1]]


class Decode(unittest.TestCase):
    def test_16uc1_millimetres_little_endian(self):
        data = struct.pack('<3H', 1500, 0, 65535)
        d = da.decode_depth(data, 1, 3, '16UC1')
        self.assertAlmostEqual(d[0, 0], 1.5)
        self.assertTrue(math.isnan(d[0, 1]))                # 0 = no return
        self.assertAlmostEqual(d[0, 2], 65.535, places=4)     # float32: ~4 um at 65 m, far below the 1 mm step

    def test_16uc1_big_endian(self):
        d = da.decode_depth(struct.pack('>2H', 1500, 250), 1, 2, '16UC1', is_bigendian=1)
        self.assertEqual([round(x, 3) for x in d[0]], [1.5, 0.25])

    def test_32fc1_metres_and_invalid_values(self):
        d = da.decode_depth(struct.pack('<4f', 2.0, 0.0, float('inf'), float('nan')), 1, 4, '32FC1')
        self.assertAlmostEqual(d[0, 0], 2.0)
        self.assertTrue(np.isnan(d[0, 1:]).all())

    def test_row_padding(self):
        row = struct.pack('<2H', 1000, 2000) + b'\x00' * 4       # step 8 for a 2-pixel 16-bit row
        d = da.decode_depth(row * 2, 2, 2, '16UC1', step=8)
        self.assertEqual(d.shape, (2, 2))
        self.assertAlmostEqual(d[1, 1], 2.0)

    def test_errors_are_explicit(self):
        with self.assertRaises(ValueError):
            da.decode_depth(b'\x00' * 3, 1, 2, '16UC1')          # truncated
        with self.assertRaises(ValueError):
            da.decode_depth(b'\x00' * 4, 1, 4, 'mono8')          # not a depth encoding


class Basics(unittest.TestCase):
    def test_valid_fraction(self):
        d = np.array([[1.0, np.nan], [0.0, 2.0]])
        self.assertAlmostEqual(da.valid_fraction(d), 0.5)

    def test_backproject(self):
        d = np.full((400, 640), np.nan)
        d[200, 320] = 2.0
        d[100, 320] = 1.0                                 # 100 px above centre at z=1 -> y = -100/399
        p = da.backproject(d, K)
        self.assertEqual(len(p), 2)
        got = {tuple(np.round(x, 4)) for x in p}
        self.assertIn((0.0, 0.0, 2.0), got)
        self.assertIn((0.0, round(-100 / 399, 4), 1.0), got)


class PlaneFit(unittest.TestCase):
    def check(self, depth, pitch=27.0, h=0.11, tol_deg=0.5, tol_h=0.005):
        r = da.floor_from_depth(depth, K)
        self.assertIsNotNone(r)
        self.assertAlmostEqual(r['pitch_deg'], pitch, delta=tol_deg)
        self.assertAlmostEqual(r['height_m'], h, delta=tol_h)
        return r

    def test_exact_floor(self):
        r = self.check(floor_depth())
        self.assertLess(r['rms_m'], 1e-6)

    def test_other_geometry(self):
        self.check(floor_depth(h=0.80, pitch_deg=10.0), pitch=10.0, h=0.80)

    def test_noise(self):
        d = floor_depth()
        rng = np.random.default_rng(1)
        self.check(d * (1 + rng.normal(0, 0.01, d.shape)))

    def test_outliers_do_not_move_it(self):
        d = floor_depth()
        rng = np.random.default_rng(2)
        mask = rng.random(d.shape) < 0.2
        d[mask] = rng.uniform(0.3, 6.0, mask.sum())
        self.check(d)

    def test_floor_beats_a_smaller_wall(self):
        d = floor_depth()
        d[:, :180] = 1.2                                   # a wall-ish fronto-parallel patch on the left 28 %
        self.check(d)

    def test_too_few_points_is_none_not_a_guess(self):
        d = np.full((400, 640), np.nan)
        d[0, :5] = 1.0
        self.assertIsNone(da.floor_from_depth(d, K))


class Compare(unittest.TestCase):
    def test_scale_error_is_measured(self):
        b = blocks()
        r = da.compare_depth(b * 1.02, b)
        self.assertAlmostEqual(r['median_rel'], 0.02, places=6)
        self.assertAlmostEqual(r['bias_rel'], 0.02, places=6)
        self.assertAlmostEqual(r['p90_rel'], 0.02, places=6)
        self.assertEqual(r['frac_bad'], 0.0)
        self.assertEqual(r['n'], b.size)
        r2 = da.compare_depth(b * 0.97, b)
        self.assertAlmostEqual(r2['bias_rel'], -0.03, places=6)     # the sign of a bias is kept
        self.assertAlmostEqual(r2['median_rel'], 0.03, places=6)

    def test_invalid_pixels_are_excluded(self):
        b = blocks()
        a = b.copy()
        a[:50] = np.nan
        a[50:, :10] = 99.0                                 # gross errors on 10 of 160 columns
        r = da.compare_depth(a, b)
        self.assertEqual(r['n'], 50 * 160)
        self.assertLess(r['median_rel'], 1e-9)             # the median ignores the 6 % gross errors...
        self.assertAlmostEqual(r['frac_bad'], 10 / 160)     # ...and frac_bad (|error| > 10 %) counts them

    def test_too_little_overlap_says_so(self):
        a = np.full((10, 10), np.nan)
        a[0, 0] = 1.0
        r = da.compare_depth(a, np.ones((10, 10)))
        self.assertEqual(r['n'], 1)
        self.assertIsNone(r['median_rel'])


class Shift(unittest.TestCase):
    def test_zero_shift_for_identical(self):
        b = blocks()
        self.assertEqual(da.best_shift(b, b, max_shift=6)[0], 0)

    def test_finds_positive_and_negative_shifts(self):
        a = blocks()
        right = np.full_like(a, np.nan)
        right[:, 3:] = a[:, :-3]                           # a[:, x] == right[:, x + 3]
        self.assertEqual(da.best_shift(a, right, max_shift=6)[0], 3)
        left = np.full_like(a, np.nan)
        left[:, :-2] = a[:, 2:]                            # a[:, x] == left[:, x - 2]
        self.assertEqual(da.best_shift(a, left, max_shift=6)[0], -2)

    def test_error_at_best_shift_is_reported(self):
        a = blocks()
        s, err = da.best_shift(a, a, max_shift=4)
        self.assertEqual(s, 0)
        self.assertLess(err, 1e-9)


class Stamps(unittest.TestCase):
    def test_missing_frames_and_rate(self):
        period = 1 / 30
        keep = [i for i in range(300) if i not in (10, 20, 21)]
        st = da.stamp_stats([1000.0 + i * period for i in keep], nominal_hz=30.0)
        self.assertEqual(st['n'], 297)
        self.assertEqual(st['missing'], 3)
        self.assertAlmostEqual(st['missing_pct'], 1.0, places=6)
        self.assertAlmostEqual(st['worst_gap_ms'], 100.0, places=3)
        self.assertAlmostEqual(st['rate_hz'], 296 / (299 * period), places=6)

    def test_driver_stamp_wobble_is_not_loss(self):
        rng = np.random.default_rng(3)
        t, s = 1000.0, []
        for _ in range(300):
            s.append(t)
            t += rng.uniform(0.0322, 0.0336)               # the driver's measured 32.2-33.6 ms wobble
        self.assertEqual(da.stamp_stats(s, 30.0)['missing'], 0)

    def test_unsorted_and_duplicate_stamps(self):
        s = [1000.0 + i / 30 for i in range(30)]
        st = da.stamp_stats(list(reversed(s)) + [s[5]], 30.0)
        self.assertEqual((st['n'], st['missing'], st['duplicates']), (30, 0, 1))

    def test_too_few(self):
        self.assertIsNone(da.stamp_stats([1.0], 30.0))

    def test_pairing(self):
        left = [1000.0 + i / 30 for i in range(300)]
        right = [x + 0.001 for i, x in enumerate(left) if i not in (5, 6, 7)]
        self.assertAlmostEqual(da.pair_fraction(left, right, tol_s=0.002), 297 / 300)
        self.assertAlmostEqual(da.pair_fraction(left, [x + 0.005 for x in left], tol_s=0.002), 0.0)


def info(**kw):
    base = {'height': 640, 'width': 1024, 'distortion_model': 'rational_polynomial', 'd': [0.0] * 8,
            'k': [638.0, 0, 527.4, 0, 638.0, 317.6, 0, 0, 1], 'r': [1, 0, 0, 0, 1, 0, 0, 0, 1],
            'p': [638.0, 0, 527.4, 0, 0, 638.0, 317.6, 0, 0, 0, 1, 0], 'frame_id': 'oak_left_camera_optical_frame'}
    base.update(kw)
    return base


class Diffs(unittest.TestCase):
    def test_identical_camera_info(self):
        self.assertEqual(da.camera_info_diff(info(), info()), [])

    def test_changed_fields_are_named(self):
        k2 = info()['k'][:]
        k2[2] = 513.7
        self.assertEqual(da.camera_info_diff(info(), info(k=k2)), ['k'])
        self.assertEqual(da.camera_info_diff(info(), info(frame_id='oak_rgb_camera_optical_frame')), ['frame_id'])
        self.assertEqual(sorted(da.camera_info_diff(info(), info(width=640, height=400))), ['height', 'width'])

    def test_float_noise_is_not_a_change(self):
        k2 = [x + 1e-12 for x in info()['k']]
        self.assertEqual(da.camera_info_diff(info(), info(k=k2)), [])

    def test_tf_difference(self):
        a = ((0.105, 0.038, 0.030), (0.0, 0.0, 0.0, 1.0))
        half = math.radians(0.1) / 2
        b = ((0.106, 0.038, 0.030), (0.0, 0.0, math.sin(half), math.cos(half)))
        dist, ang = da.tf_diff(a, b)
        self.assertAlmostEqual(dist, 0.001, places=9)
        self.assertAlmostEqual(ang, 0.1, places=6)

    def test_tf_quaternion_sign_does_not_matter(self):
        a = ((0, 0, 0), (0.0, 0.0, 0.0, 1.0))
        b = ((0, 0, 0), (0.0, 0.0, 0.0, -1.0))              # same rotation
        self.assertAlmostEqual(da.tf_diff(a, b)[1], 0.0, places=9)


if __name__ == '__main__':
    unittest.main()
