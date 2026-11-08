"""StereoRig's matcher option: SGBM stays the default; VPI (GPU / OFA) is only used when asked and
must give the same wall distances on the exact synthetic pairs. VPI tests skip off the Jetson."""
from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))
from overwall_nav import vpi_stereo                                                # noqa: E402
from overwall_nav.perception import analyze, bin_points                           # noqa: E402
from overwall_nav.stereo import StereoRig, extrinsics_from_tf, level              # noqa: E402
import synthetic                                                                   # noqa: E402


def rig(**kw):
    r, t = extrinsics_from_tf(synthetic.T_LR, synthetic.Q_LR)
    return StereoRig(synthetic.K_LEFT, synthetic.K_RIGHT, synthetic.SIZE, r, t, **kw)


def front(r, d):
    left, right = synthetic.pair(cam_height=0.85, wall_distance=d, wall_top=1.0)
    pts, _ = r.points(left, right)
    return analyze(bin_points(level(pts, 27.0), 0.4), 0.85, r.top_elevation_deg(27.0))['front']


class MatcherOptionTest(unittest.TestCase):
    def test_default_is_cpu_sgbm(self):
        self.assertEqual(rig().matcher_name, 'sgbm')

    def test_unknown_matcher_is_refused(self):
        with self.assertRaises(ValueError):
            rig(matcher='magic')

    def test_points_reports_timings(self):
        left, right = synthetic.pair(cam_height=0.85)
        tm = {}
        rig().points(left, right, timings=tm)
        self.assertEqual(set(tm), {'rectify', 'match', 'post'})


@unittest.skipUnless(vpi_stereo.available(), 'VPI not installed (only on the Jetson)')
class VpiMatcherTest(unittest.TestCase):
    def test_vpi_cuda_measures_walls_like_sgbm(self):
        r = rig(matcher='vpi-cuda')
        for d in (0.5, 0.8, 1.5, 2.5):
            got = front(r, d)
            self.assertIsNotNone(got, d)
            self.assertAlmostEqual(got, d, delta=0.05 * d + 0.02)


if __name__ == '__main__':
    unittest.main()
