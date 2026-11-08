"""Evals for exposure_analysis.py (numpy only; no drone, no ROS, no OpenCV):

    cd tools/drone_tools && python3 -m unittest -v test_exposure_analysis

The point of this module is to answer "how long does the camera take to recover when the light changes
(bright hall -> dark tunnel -> bright again)?". So the tests build brightness histories with a KNOWN
recovery (an exponential with a chosen time constant) and require the analysis to give back the closed-form
answer, and to stay quiet on a steady scene.
"""
import math
import unittest

import numpy as np

import exposure_analysis as ea


def img(value, h=64, w=96):
    return np.full((h, w), value, np.uint8)


class FrameMetrics(unittest.TestCase):
    def test_uniform_mid_grey(self):
        m = ea.frame_metrics(img(100))
        self.assertAlmostEqual(m['mean'], 100.0)
        self.assertEqual((m['p5'], m['p50'], m['p95']), (100, 100, 100))
        self.assertEqual((m['sat'], m['black']), (0.0, 0.0))
        self.assertAlmostEqual(m['sharp'], 0.0)
        self.assertAlmostEqual(m['edges'], 0.0)

    def test_fully_saturated_and_fully_black(self):
        self.assertEqual(ea.frame_metrics(img(255))['sat'], 100.0)
        self.assertEqual(ea.frame_metrics(img(0))['black'], 100.0)
        self.assertEqual(ea.frame_metrics(img(250))['sat'], 100.0)     # >= 250 counts as clipped
        self.assertEqual(ea.frame_metrics(img(249))['sat'], 0.0)

    def test_half_black_half_white(self):
        a = img(0)
        a[:, 48:] = 255
        m = ea.frame_metrics(a)
        self.assertAlmostEqual(m['sat'], 50.0)
        self.assertAlmostEqual(m['black'], 50.0)
        self.assertAlmostEqual(m['mean'], 127.5)
        self.assertGreater(m['edges'], 0.0)
        self.assertLess(m['edges'], 5.0)                  # one vertical edge: a thin strip, not a textured frame

    def test_texture_separates_flat_from_noisy(self):
        rng = np.random.default_rng(0)
        noisy = np.clip(rng.normal(100, 25, (64, 96)), 0, 255).astype(np.uint8)
        flat, tex = ea.frame_metrics(img(100)), ea.frame_metrics(noisy)
        self.assertGreater(tex['edges'], 50.0)
        self.assertLess(flat['edges'], 1.0)
        self.assertGreater(tex['sharp'], 100 * flat['sharp'] + 100)

    def test_percentiles_order(self):
        rng = np.random.default_rng(1)
        m = ea.frame_metrics(rng.integers(0, 256, (64, 96), dtype=np.uint8))
        self.assertLess(m['p5'], m['p50'])
        self.assertLess(m['p50'], m['p95'])

    def test_rejects_non_2d_input_clearly(self):
        with self.assertRaises(ValueError):
            ea.frame_metrics(np.zeros((4, 4, 3), np.uint8))
        with self.assertRaises(ValueError):
            ea.frame_metrics(np.zeros((4, 4), np.float32))


def series(f, t0=0.0, t1=40.0, hz=10.0):
    t = np.arange(t0, t1, 1.0 / hz)
    return t, np.array([f(x) for x in t])


class FindSteps(unittest.TestCase):
    def cover_uncover(self, x):
        if x < 10:
            return 120.0
        if x < 25:
            return 6.0 + (110.0 - 6.0) * (1 - math.exp(-(x - 10) / 3.0))
        return min(255.0, 120.0 + 130.0 * math.exp(-(x - 25) / 0.8))

    def test_finds_both_events_with_direction_and_time(self):
        t, y = series(self.cover_uncover)
        ev = ea.find_steps(t, y)
        self.assertEqual([e['direction'] for e in ev], ['dark', 'bright'])
        self.assertAlmostEqual(ev[0]['t'], 10.0, delta=0.3)
        self.assertAlmostEqual(ev[1]['t'], 25.0, delta=0.3)
        self.assertGreater(ev[0]['before'], 100)
        self.assertLess(ev[0]['after'], 30)

    def test_flat_scene_has_no_events(self):
        t, y = series(lambda x: 120.0)
        self.assertEqual(ea.find_steps(t, y), [])

    def test_noise_does_not_create_events(self):
        rng = np.random.default_rng(2)
        t = np.arange(0, 40, 0.1)
        y = 120 + rng.normal(0, 4, len(t))
        self.assertEqual(ea.find_steps(t, y), [])

    def test_slow_drift_is_not_a_step(self):
        t, y = series(lambda x: 100.0 + 1.5 * x)           # +60 over 40 s, gentle
        self.assertEqual(ea.find_steps(t, y), [])

    def test_a_large_absolute_but_small_relative_change_is_not_a_step(self):
        # the tail of an over-exposure recovery: 255 -> 190 is 65 grey levels but only a 1.34x change
        t, y = series(lambda x: 255.0 if x < 20 else 190.0)
        self.assertEqual(ea.find_steps(t, y), [])

    def test_a_small_absolute_but_large_relative_change_in_the_dark_is_a_step(self):
        # a real dark-to-lit-up change that an absolute-only rule at a high threshold would call noise
        t, y = series(lambda x: 5.0 if x < 20 else 40.0)
        ev = ea.find_steps(t, y)
        self.assertEqual([e['direction'] for e in ev], ['bright'])

    def test_too_short_input_is_handled(self):
        self.assertEqual(ea.find_steps(np.array([0.0, 0.1]), np.array([10.0, 200.0])), [])


class SettleTime(unittest.TestCase):
    def test_matches_the_closed_form_for_an_exponential(self):
        for tau, y0, final in ((3.0, 6.0, 110.0), (0.8, 250.0, 120.0), (1.5, 20.0, 100.0)):
            t = np.arange(0, 30, 0.1)
            y = final + (y0 - final) * np.exp(-t / tau)
            tol = 0.10
            expected = tau * math.log(abs(y0 - final) / (tol * final))
            got = ea.settle_time(t, y, 0.0, 30.0, tol=tol)
            self.assertAlmostEqual(got, expected, delta=0.25, msg=f'tau={tau}')

    def test_returns_none_if_it_never_settles(self):
        t = np.arange(0, 30, 0.1)
        y = 120 + 60 * np.sin(t)                            # keeps oscillating by +-50 %
        self.assertIsNone(ea.settle_time(t, y, 0.0, 30.0, tol=0.1))

    def test_already_settled_is_zero(self):
        t = np.arange(0, 10, 0.1)
        self.assertEqual(ea.settle_time(t, np.full(len(t), 120.0), 0.0, 10.0), 0.0)

    def test_a_late_wobble_counts_against_settling(self):
        t = np.arange(0, 30, 0.1)
        y = np.full(len(t), 120.0)
        y[(t > 20) & (t < 21)] = 60.0                       # settled at 0 s, then a dip at 20 s
        self.assertAlmostEqual(ea.settle_time(t, y, 0.0, 30.0, tol=0.1), 21.0, delta=0.2)


class RecoverTime(unittest.TestCase):
    """Recovery is judged against an explicit TARGET (the pre-event baseline), never against the series' own
    end state: a camera that collapses and stays collapsed has not recovered."""

    def test_time_to_stay_above_half_of_target(self):
        t = np.arange(0, 30, 0.1)
        y = 100 * (1 - np.exp(-t / 2.0))                    # reaches 50 at 2*ln2 = 1.386 s
        self.assertAlmostEqual(ea.recover_time(t, y, 0.0, 30.0, target=100.0, frac=0.5), 2.0 * math.log(2), delta=0.15)

    def test_collapsed_and_stays_collapsed_is_none(self):
        t = np.arange(0, 30, 0.1)
        y = 100.0 * (t < 5) + 5.0 * (t >= 5)                # good at first, then collapses and stays collapsed
        self.assertIsNone(ea.recover_time(t, y, 5.0, 30.0, target=100.0, frac=0.5))

    def test_already_above_threshold_is_zero(self):
        t = np.arange(0, 10, 0.1)
        self.assertEqual(ea.recover_time(t, np.full(len(t), 90.0), 0.0, 10.0, target=100.0, frac=0.5), 0.0)

    def test_only_the_last_sample_up_is_still_none(self):
        t = np.arange(0, 10, 0.1)
        y = np.where(t < 9.95, 0.0, 100.0)
        self.assertIsNone(ea.recover_time(t, y, 0.0, 9.95, target=100.0, frac=0.5))

    def test_a_dip_after_recovering_counts_against_it(self):
        t = np.arange(0, 20, 0.1)
        y = np.full(len(t), 100.0)
        y[(t > 12) & (t < 13)] = 10.0
        self.assertAlmostEqual(ea.recover_time(t, y, 0.0, 20.0, target=100.0, frac=0.5), 13.0, delta=0.2)


class Report(unittest.TestCase):
    def test_report_names_the_numbers_a_human_needs(self):
        def mean(x):
            if x < 10:
                return 120.0
            if x < 25:
                return 6.0 + 104.0 * (1 - math.exp(-(x - 10) / 3.0))
            return min(255.0, 120.0 + 130.0 * math.exp(-(x - 25) / 0.8))
        text = ea.format_report(ea.summarise(frames_for(mean, lambda x, m: 40.0 if m >= 200 else 0.0)))
        for needle in ('baseline', 'dark', 'bright', 'settles', 'saturated', 'darkest'):
            self.assertIn(needle, text.lower())

    def test_report_for_a_steady_scene_says_so(self):
        text = ea.format_report(ea.summarise(frames_for(lambda x: 120.0, lambda x, m: 0.0)))
        self.assertIn('no brightness changes', text.lower())

    def test_load_frames_skips_blank_and_broken_lines(self):
        import os, tempfile
        with tempfile.NamedTemporaryFile('w', delete=False, suffix='.jsonl') as f:
            f.write('{"t": 1.0, "side": "left", "mean": 5.0}\n\nnot json\n{"t": 2.0, "side": "left", "mean": 6.0}\n')
        try:
            fr, skipped = ea.load_frames(f.name)
            self.assertEqual([x['t'] for x in fr], [1.0, 2.0])
            self.assertEqual(skipped, 1)
        finally:
            os.unlink(f.name)


def frames_for(f_mean, f_sat, t1=40.0, hz=10.0, side='left'):
    out = []
    for x in np.arange(0.0, t1, 1.0 / hz):
        m = f_mean(x)
        out.append({'t': float(x), 'side': side, 'mean': m, 'p5': m, 'p50': m, 'p95': m, 'sat': f_sat(x, m),
                    'black': 100.0 if m < 5 else 0.0, 'sharp': m, 'edges': max(0.0, min(100.0, m / 1.2))})
    return out


class Summarise(unittest.TestCase):
    def setUp(self):
        def mean(x):
            if x < 10:
                return 120.0
            if x < 25:
                return 6.0 + 104.0 * (1 - math.exp(-(x - 10) / 3.0))
            return min(255.0, 120.0 + 130.0 * math.exp(-(x - 25) / 0.8))
        self.frames = frames_for(mean, lambda x, m: 40.0 if m >= 200 else 0.0)

    def test_baseline_events_and_settle_times(self):
        s = ea.summarise(self.frames)
        self.assertAlmostEqual(s['baseline']['mean'], 120.0, delta=1.0)
        d, b = s['events']
        self.assertEqual((d['direction'], b['direction']), ('dark', 'bright'))
        self.assertAlmostEqual(d['settle_s'], 3.0 * math.log(104 / 11), delta=0.6)    # ~6.7 s to reach within 10 %
        self.assertAlmostEqual(b['settle_s'], 0.8 * math.log(130 / 12), delta=0.5)    # ~1.9 s

    def test_overexposure_after_uncovering_is_reported(self):
        s = ea.summarise(self.frames)
        self.assertEqual(s['events'][1]['peak_sat_pct'], 40.0)
        self.assertEqual(s['events'][0]['peak_sat_pct'], 0.0)

    def test_darkest_point_and_black_time_after_covering(self):
        s = ea.summarise(self.frames)
        d = s['events'][0]
        self.assertAlmostEqual(d['min_mean'], 6.0, delta=1.5)
        self.assertGreaterEqual(d['black_s'], 0.0)

    def test_steady_scene_reports_no_events(self):
        s = ea.summarise(frames_for(lambda x: 120.0, lambda x, m: 0.0))
        self.assertEqual(s['events'], [])
        self.assertAlmostEqual(s['baseline']['mean'], 120.0)

    def test_too_few_frames_is_an_error_not_a_guess(self):
        with self.assertRaises(ValueError):
            ea.summarise(frames_for(lambda x: 120.0, lambda x, m: 0.0, t1=0.3))


if __name__ == '__main__':
    unittest.main()
