"""Evals for the ROS-free parts of exposure_probe.py: python3 -m unittest -v test_exposure_probe"""
import json
import unittest
from types import SimpleNamespace

import numpy as np

import exposure_probe as ep


def msg(arr, encoding, step=None, stamp=(1794104000, 500_000_000)):
    a = np.asarray(arr, np.uint8)
    h, w = a.shape[:2]
    ch = a.shape[2] if a.ndim == 3 else 1
    row = w * ch if step is None else step
    buf = bytearray(h * row)
    flat = a.reshape(h, w * ch)
    for r in range(h):
        buf[r * row:r * row + w * ch] = flat[r].tobytes()
    return SimpleNamespace(height=h, width=w, encoding=encoding, step=row, data=bytes(buf),
                           header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp[0], nanosec=stamp[1])))


class ImageToGray(unittest.TestCase):
    def test_mono8(self):
        a = np.arange(12, dtype=np.uint8).reshape(3, 4)
        g, equal = ep.image_to_gray(msg(a, 'mono8'))
        self.assertTrue(np.array_equal(g, a))
        self.assertTrue(equal)

    def test_bgr8_with_equal_channels_is_grey(self):
        a = np.arange(12, dtype=np.uint8).reshape(3, 4)
        g, equal = ep.image_to_gray(msg(np.dstack([a, a, a]), 'bgr8'))
        self.assertTrue(np.array_equal(g, a))
        self.assertTrue(equal)

    def test_bgr8_with_different_channels_is_flagged_not_assumed(self):
        a = np.zeros((3, 4, 3), np.uint8)
        a[..., 0], a[..., 1], a[..., 2] = 30, 60, 90
        g, equal = ep.image_to_gray(msg(a, 'bgr8'))
        self.assertFalse(equal)                              # the "mono camera" assumption is checked, not trusted
        self.assertEqual(int(g[0, 0]), 60)                   # mean of the channels

    def test_row_padding_is_skipped(self):
        a = np.arange(12, dtype=np.uint8).reshape(3, 4)
        g, _ = ep.image_to_gray(msg(a, 'mono8', step=10))   # 6 padding bytes per row
        self.assertTrue(np.array_equal(g, a))

    def test_unknown_encoding_is_an_error(self):
        with self.assertRaises(ValueError):
            ep.image_to_gray(msg(np.zeros((2, 2), np.uint8), '16UC1'))

    def test_truncated_data_is_an_error(self):
        m = msg(np.zeros((4, 4), np.uint8), 'mono8')
        m.data = m.data[:-3]
        with self.assertRaises(ValueError):
            ep.image_to_gray(m)


class Throttle(unittest.TestCase):
    def test_limits_each_key_independently(self):
        th = ep.Throttle(5.0)                               # one frame per 0.2 s
        self.assertTrue(th.allow('left', 10.00))
        self.assertTrue(th.allow('right', 10.01))           # right is not blocked by left
        self.assertFalse(th.allow('left', 10.05))
        self.assertFalse(th.allow('left', 10.19))
        self.assertTrue(th.allow('left', 10.21))

    def test_a_skipped_frame_does_not_reset_the_clock(self):
        th = ep.Throttle(5.0)
        th.allow('a', 0.0)
        th.allow('a', 0.15)                                  # rejected
        self.assertTrue(th.allow('a', 0.2))                  # still on the original 0.2 s grid

    def test_a_30hz_stream_is_sampled_at_the_requested_rate_without_drift(self):
        # 4 Hz = every 7.5 frames of a 30 Hz stream. A throttle that restarts its clock at each accepted frame
        # drifts to every 8 frames and delivers 38 samples in 10 s instead of 40 (5 Hz would hide this: 0.2 s is
        # an exact number of frames).
        th = ep.Throttle(4.0)
        taken = [i / 30.0 for i in range(300) if th.allow('x', i / 30.0)]
        self.assertEqual(len(taken), 40)
        self.assertLessEqual(float(np.diff(taken).max()), 0.25 + 1 / 30 + 1e-6)

    def test_rejects_nonsense_rates(self):
        with self.assertRaises(ValueError):
            ep.Throttle(0)


class MakeRecord(unittest.TestCase):
    def test_record_has_stamp_arrival_side_and_metrics_and_is_json(self):
        m = msg(np.full((8, 8), 100, np.uint8), 'mono8', stamp=(1794104000, 250_000_000))
        r = ep.make_record('left', m, arrival=1794104000.9)
        self.assertAlmostEqual(r['t'], 1794104000.25)
        self.assertAlmostEqual(r['age_ms'], 650.0, places=3)
        self.assertEqual(r['side'], 'left')
        self.assertAlmostEqual(r['mean'], 100.0)
        self.assertTrue(r['grey'])
        json.dumps(r)                                        # must serialise as is

    def test_non_grey_frames_are_marked(self):
        a = np.zeros((4, 4, 3), np.uint8)
        a[..., 2] = 200
        r = ep.make_record('left', msg(a, 'bgr8'), arrival=1794104000.6)
        self.assertFalse(r['grey'])


if __name__ == '__main__':
    unittest.main()
