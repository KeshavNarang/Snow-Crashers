"""Evals for frames.build_candidates(): where could the device depth be? python3 -m unittest -v test_frames

ROS CameraInfo convention: R rotates raw-camera points into the rectified frame (X_rect = R X_raw), P projects
rectified points (P[0][3] = -fx' * baseline on the second camera). A host stereo match gives depth in the rectified
left and right frames; the candidates are those two plus the raw left frame that the depth's own camera_info
describes (its K, R = identity). The R direction is easy to get backwards, so it is tested with a known rotation.
"""
import math
import unittest

import numpy as np

import frames as fr
import geometry as g

FX, CX, CY = 638.0, 512.0, 320.0
HOST = (1024, 640)
DEV = (640, 400)


def info(k=(FX, FX, CX, CY), r=np.eye(3), p_tx=0.0, size=HOST, frame='oak_left_camera_optical_frame'):
    fx, fy, cx, cy = k
    return {'width': size[0], 'height': size[1], 'k': [fx, 0, cx, 0, fy, cy, 0, 0, 1],
            'r': [float(x) for x in np.asarray(r).ravel()], 'd': [0.0] * 8,
            'p': [FX, 0, CX, p_tx, 0, FX, CY, 0, 0, 0, 1, 0], 'frame_id': frame, 'distortion_model': 'rational_polynomial'}


def rot_y(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])


def wall(z=2.0, size=HOST):
    return np.full((size[1], size[0]), z, float)


class Baseline(unittest.TestCase):
    def test_from_whichever_camera_carries_tx(self):
        self.assertAlmostEqual(fr.baseline(info(), info(p_tx=-FX * 0.075)), 0.075)
        self.assertAlmostEqual(fr.baseline(info(p_tx=-FX * 0.075), info()), 0.075)   # the driver's reversed order

    def test_no_baseline_is_an_error(self):
        with self.assertRaises(ValueError):
            fr.baseline(info(), info())


class Candidates(unittest.TestCase):
    def test_identity_rectification(self):
        c = fr.build_candidates(wall(), wall(), info(), info(p_tx=-FX * 0.075), info(size=DEV), HOST, DEV)
        self.assertEqual(set(c), {'claimed: raw left (its camera_info)', 'rectified left', 'rectified right'})
        for name, d in c.items():
            self.assertEqual(d.shape, (DEV[1], DEV[0]), name)
            m = np.isfinite(d)
            self.assertGreater(m.mean(), 0.9, name)
            self.assertTrue(np.allclose(d[m], 2.0), name)

    def test_rectification_rotation_direction(self):
        # left camera rotated 2 deg relative to the rectified frame: X_rect = R X_raw.  A fronto-parallel wall in the
        # rectified frame is tilted in the raw frame: its depth at raw pixel (u, v) is analytic.
        r = rot_y(2.0)
        k_dev = g.scale_k((FX, FX, CX, CY), HOST, DEV)
        c = fr.build_candidates(wall(), wall(), info(r=r), info(p_tx=-FX * 0.075), info(k=k_dev, size=DEV), HOST, DEV)
        raw = c['claimed: raw left (its camera_info)']
        u, v = 500, 200
        ray_raw = np.array([(u - k_dev[2]) / k_dev[0], (v - k_dev[3]) / k_dev[1], 1.0])
        ray_rect = r @ ray_raw                          # the same ray in the rectified frame
        s = 2.0 / ray_rect[2]                           # hits the wall Z_rect = 2 at s * ray_raw in the raw frame
        self.assertAlmostEqual(raw[v, u], s * ray_raw[2], delta=0.005)
        # with the rotation taken the wrong way round the answer differs measurably
        wrong = 2.0 / (r.T @ ray_raw)[2] * ray_raw[2]
        self.assertGreater(abs(wrong - s * ray_raw[2]), 0.005)


if __name__ == '__main__':
    unittest.main()
