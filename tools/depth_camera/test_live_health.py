"""Evals for the health decision used to auto-revert a swap: python3 -m unittest -v test_live_health

assess() turns header stamps and cuVSLAM states from the last few seconds into ok / not ok with a reason.
It must pass a healthy stack and fail each way the swap could break VIO or fail to deliver depth.
"""
import unittest

import live_health as lh


def stream(hz=30.0, secs=8.0, t0=1000.0, drop=()):
    n = int(hz * secs)
    return [t0 + i / hz for i in range(n) if i not in drop]


def meas(**kw):
    m = {'left': stream(), 'right': stream(), 'depth': stream(), 'vo_states': [1] * 240, 'window_s': 8.0}
    m.update(kw)
    return m


class Assess(unittest.TestCase):
    def test_healthy_stack_passes(self):
        r = lh.assess(meas(), want_depth=True)
        self.assertTrue(r['ok'], r)

    def test_depth_not_required_when_not_asked(self):
        self.assertTrue(lh.assess(meas(depth=[]), want_depth=False)['ok'])

    def test_missing_left_camera(self):
        r = lh.assess(meas(left=[]), want_depth=False)
        self.assertFalse(r['ok'])
        self.assertIn('left', r['why'])

    def test_slow_right_camera(self):
        r = lh.assess(meas(right=stream(hz=20.0)), want_depth=False)
        self.assertFalse(r['ok'])
        self.assertIn('right', r['why'])

    def test_frame_loss(self):
        lossy = stream(drop=set(range(0, 240, 10)))             # 10 % of frames missing, rate still ~27 Hz
        r = lh.assess(meas(left=lossy), want_depth=False)
        self.assertFalse(r['ok'])
        self.assertIn('missing', r['why'])

    def test_unpaired_stereo(self):
        r = lh.assess(meas(right=[t + 0.016 for t in stream()]), want_depth=False)   # half a frame apart
        self.assertFalse(r['ok'])
        self.assertIn('pair', r['why'])

    def test_cuvslam_silent_or_lost(self):
        self.assertIn('cuVSLAM', lh.assess(meas(vo_states=[]), want_depth=False)['why'])
        lost = lh.assess(meas(vo_states=[1] * 200 + [2] * 40), want_depth=False)
        self.assertFalse(lost['ok'])
        self.assertIn('tracking', lost['why'])

    def test_tracking_lost_just_now_fails_even_after_a_good_window(self):
        r = lh.assess(meas(vo_states=[1] * 235 + [2] * 5), want_depth=False)   # 98 % tracking, but lost now
        self.assertFalse(r['ok'])
        self.assertIn('tracking', r['why'])

    def test_brief_reinit_at_start_is_tolerated(self):
        self.assertTrue(lh.assess(meas(vo_states=[2] * 10 + [1] * 230), want_depth=False)['ok'])

    def test_depth_missing_or_slow(self):
        self.assertIn('depth', lh.assess(meas(depth=[]), want_depth=True)['why'])
        self.assertFalse(lh.assess(meas(depth=stream(hz=10.0)), want_depth=True)['ok'])

    def test_report_carries_the_numbers(self):
        r = lh.assess(meas(), want_depth=True)
        for k in ('left', 'right', 'depth', 'pair_fraction', 'tracking_fraction'):
            self.assertIn(k, r['numbers'])


if __name__ == '__main__':
    unittest.main()
