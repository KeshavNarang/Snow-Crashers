"""Evals for slam_gaps.py (no drone, no ROS): python3 -m unittest -v"""
import unittest

import slam_gaps as sg

LINE = ('[component_container_mt-3] [WARN] [{ts}] [visual_slam_node]: Delta between current and previous '
        'frame [{d} ms] is above threshold [40.000000 ms]')


def warn(ts, d):
    return LINE.format(ts=ts, d=d)


class Parse(unittest.TestCase):
    def test_extracts_stamp_and_delta(self):
        g = sg.parse_gaps(warn('1794104509.728434996', '99.999840'))
        self.assertEqual(len(g), 1)
        self.assertAlmostEqual(g[0][0], 1794104509.728435, places=3)
        self.assertAlmostEqual(g[0][1], 99.99984, places=3)

    def test_ignores_other_lines_and_other_nodes(self):
        text = '\n'.join(['[INFO] [1794104509.1] [oak]: started',
                          '[WARN] [1794104509.2] [other_node]: Delta between current and previous frame [80.0 ms] is above threshold [40.0 ms]',
                          '[component_container_mt-3] [WARN] [1794104509.3] [visual_slam_node]: Unknown Tracker Error 2',
                          warn('1794104510.0', '66.666555')])
        g = sg.parse_gaps(text)
        self.assertEqual([round(d) for _, d in g], [67])

    def test_empty_and_garbage(self):
        self.assertEqual(sg.parse_gaps(''), [])
        self.assertEqual(sg.parse_gaps('nothing to see here\n\x00\xff'), [])


class Summarise(unittest.TestCase):
    def test_missed_frames_from_gap_sizes(self):
        # 66.7 ms = one frame missing, 100 ms = two, 166.7 ms = four
        gaps = [(1000.0, 66.667), (1010.0, 100.0), (1020.0, 166.667)]
        s = sg.summarise(gaps, span_s=100.0, fps=30.0)
        self.assertEqual(s['warnings'], 3)
        self.assertEqual(s['missed'], 1 + 2 + 4)
        self.assertEqual(s['histogram'], {2: 1, 3: 1, 5: 1})

    def test_effective_rate(self):
        gaps = [(float(i), 66.667) for i in range(30)]        # 30 single misses in 100 s
        s = sg.summarise(gaps, span_s=100.0, fps=30.0)
        self.assertAlmostEqual(s['loss_pct'], 1.0, places=1)
        self.assertAlmostEqual(s['effective_hz'], 29.7, places=1)

    def test_no_warnings_is_a_clean_feed(self):
        s = sg.summarise([], span_s=60.0, fps=30.0)
        self.assertEqual((s['warnings'], s['missed'], s['loss_pct']), (0, 0, 0.0))
        self.assertEqual(s['effective_hz'], 30.0)

    def test_per_minute_counts_include_quiet_minutes(self):
        gaps = [(1000.0, 66.7), (1001.0, 66.7), (1130.0, 66.7)]   # minute 0 twice, minute 2 once
        s = sg.summarise(gaps, span_s=180.0, fps=30.0, t0=1000.0)
        self.assertEqual(s['per_minute'], [2, 0, 1])

    def test_worst_gap(self):
        s = sg.summarise([(1.0, 66.7), (2.0, 500.0)], span_s=10.0, fps=30.0)
        self.assertAlmostEqual(s['worst_gap_ms'], 500.0)


def rec(t, cpu, rq, top=''):
    return {'epoch': t, 'cpu_pct': cpu, 'run_queue': rq, 'procs': [{'label': top, 'cores': 1.0}] if top else []}


class Correlate(unittest.TestCase):
    def test_gap_seconds_with_higher_load_are_detected(self):
        recs = [rec(1000 + i, 30.0, 2.0, 'stereo_depth') for i in range(60)]
        for i in (10, 11, 40):                              # three busy seconds with gaps right after
            recs[i] = rec(1000 + i, 95.0, 9.0, 'polkitd')
        gaps = [(1010.4, 66.7), (1011.2, 66.7), (1040.4, 66.7)]
        c = sg.correlate(gaps, recs, window=1.0)
        self.assertEqual(c['gap_seconds'], 3)
        self.assertGreater(c['cpu_in_gaps'], c['cpu_elsewhere'] + 50)
        self.assertGreater(c['runq_in_gaps'], c['runq_elsewhere'] + 5)
        self.assertEqual(c['top_in_gaps'][0][0], 'polkitd')
        self.assertIn('coincide', c['verdict'])
        self.assertIn('polkitd', c['verdict'])

    def test_no_relationship_is_reported_as_none(self):
        recs = [rec(1000 + i, 30.0, 2.0, 'stereo_depth') for i in range(60)]
        gaps = [(1010.4, 66.7), (1030.2, 66.7), (1050.9, 66.7)]
        c = sg.correlate(gaps, recs, window=1.0)
        self.assertAlmostEqual(c['cpu_in_gaps'], c['cpu_elsewhere'], places=3)
        self.assertEqual(c['verdict'], 'no relationship between frame gaps and CPU load found')

    def test_gaps_outside_the_log_are_ignored(self):
        recs = [rec(1000 + i, 30.0, 2.0) for i in range(10)]
        c = sg.correlate([(5000.0, 66.7)], recs, window=1.0)
        self.assertEqual(c['gap_seconds'], 0)
        self.assertIn('not enough', c['verdict'])

    def test_time_string_fallback_when_no_epoch(self):
        import time
        t = time.mktime(time.strptime('2026-11-07T18:25:00', '%Y-%m-%dT%H:%M:%S'))
        self.assertAlmostEqual(sg.record_epoch({'time': '2026-11-07T18:25:00'}), t)
        self.assertEqual(sg.record_epoch({'epoch': 12.5, 'time': 'x'}), 12.5)


if __name__ == '__main__':
    unittest.main()
