"""Evals for the acceptance contract (acceptance.py): python3 -m unittest -v test_acceptance

judge(baseline, candidate) applies the pass/fail rules written in agents/logs/2026-11-07-depth-camera-plan.md
BEFORE any measurement existed. A criterion with no data is a failure ("not measured"), never a pass, except
the floor check, which only applies when the floor dominates the view.
"""
import copy
import unittest

import acceptance as ac

INFO_L = {'height': 640, 'width': 1024, 'distortion_model': 'rational_polynomial', 'd': [0.0] * 8,
          'k': [637.9, 0, 527.4, 0, 638.0, 317.6, 0, 0, 1], 'r': [1, 0, 0, 0, 1, 0, 0, 0, 1],
          'p': [638.0, 0, 520.0, -48.06, 0, 638.0, 320.0, 0, 0, 0, 1, 0], 'frame_id': 'oak_left_camera_optical_frame'}
INFO_R = dict(INFO_L, frame_id='oak_right_camera_optical_frame')
TF_L = ((0.105, 0.038, 0.030), (0.0, 0.226, 0.0, 0.974))
TF_R = ((0.105, -0.037, 0.030), (0.0, 0.226, 0.0, 0.974))


def stream(rate=30.0, missing_pct=0.0, n=1800):
    return {'n': n, 'rate_hz': rate, 'missing_pct': missing_pct, 'missing': 0, 'worst_gap_ms': 33.4, 'duplicates': 0}


def baseline():
    return {'left': stream(), 'right': stream(), 'pair_fraction': 1.0, 'tracking_fraction': 1.0, 'odom_rate_hz': 30.0,
            'static_drift_m': 0.004, 'gap_warnings_per_min': 20.0,
            'camera_info': {'left': INFO_L, 'right': INFO_R}, 'tf': {'left': TF_L, 'right': TF_R},
            'cpu': {'total_cores': 2.2, 'saturated_cores': 0}}


def candidate():
    c = baseline()
    c['depth'] = {'stream': stream(rate=29.8), 'encoding': '16UC1', 'size': (640, 400), 'info_size': (640, 400),
                  'age_p95_ms': 60.0, 'vs_host': {'median_rel': 0.015, 'best_shift': 0}, 'valid_fraction': 0.7,
                  'host_valid_fraction': 0.55, 'floor': {'pitch_deg': 27.6, 'inlier_frac': 0.7},
                  'expected_pitch_deg': 27.0}
    return c


def result(rows, cid):
    return next(r for r in rows if r['id'] == cid)


class Contract(unittest.TestCase):
    def test_good_candidate_passes_everything(self):
        v = ac.judge(baseline(), candidate())
        self.assertTrue(v['pass'], [r for r in v['rows'] if not r['pass']])

    def check_fails(self, mutate, cid):
        c = candidate()
        mutate(c)
        v = ac.judge(baseline(), c)
        self.assertFalse(v['pass'])
        self.assertFalse(result(v['rows'], cid)['pass'], result(v['rows'], cid))

    def test_A1_camera_rate_loss_and_pairing(self):
        self.check_fails(lambda c: c.update(left=stream(rate=28.0)), 'A1')
        self.check_fails(lambda c: c.update(right=stream(missing_pct=1.0)), 'A1')     # baseline 0 + 0.5 allowed
        self.check_fails(lambda c: c.update(pair_fraction=0.97), 'A1')

    def test_A1_tolerates_the_allowed_margin(self):
        c = candidate()
        c['left'] = stream(rate=29.6, missing_pct=0.4)
        self.assertTrue(result(ac.judge(baseline(), c)['rows'], 'A1')['pass'])

    def test_A2_cuvslam(self):
        self.check_fails(lambda c: c.update(gap_warnings_per_min=43.0), 'A2')          # > 1.5 x 20 + 2
        self.check_fails(lambda c: c.update(odom_rate_hz=28.5), 'A2')
        self.check_fails(lambda c: c.update(tracking_fraction=0.98), 'A2')
        self.check_fails(lambda c: c.update(static_drift_m=0.03), 'A2')                # > max(2 cm, 2 x 4 mm)

    def test_A3_calibration_and_tf_unchanged(self):
        def k_changed(c):
            c['camera_info'] = copy.deepcopy(c['camera_info'])
            c['camera_info']['left']['k'][2] = 513.7
        self.check_fails(k_changed, 'A3')
        self.check_fails(lambda c: c['tf'].update(right=((0.105, -0.035, 0.030), TF_R[1])), 'A3')   # 2 mm

    def test_A4_cpu(self):
        self.check_fails(lambda c: c.update(cpu={'total_cores': 2.9, 'saturated_cores': 0}), 'A4')
        self.check_fails(lambda c: c.update(cpu={'total_cores': 2.3, 'saturated_cores': 1}), 'A4')

    def test_B1_depth_stream(self):
        self.check_fails(lambda c: c['depth'].update(stream=stream(rate=20.0)), 'B1')
        self.check_fails(lambda c: c['depth'].update(encoding='32FC1'), 'B1')
        self.check_fails(lambda c: c['depth'].update(info_size=(1024, 640)), 'B1')
        self.check_fails(lambda c: c['depth'].update(age_p95_ms=150.0), 'B1')

    def test_B2_geometry_against_the_host(self):
        self.check_fails(lambda c: c['depth'].update(vs_host={'median_rel': 0.05, 'best_shift': 0}), 'B2')
        self.check_fails(lambda c: c['depth'].update(vs_host={'median_rel': 0.01, 'best_shift': 3}), 'B2')

    def test_B3_floor_only_when_it_applies(self):
        self.check_fails(lambda c: c['depth'].update(floor={'pitch_deg': 30.5, 'inlier_frac': 0.7}), 'B3')
        c = candidate()
        c['depth']['floor'] = {'pitch_deg': 40.0, 'inlier_frac': 0.2}     # the floor is not what was seen
        v = ac.judge(baseline(), c)
        self.assertTrue(v['pass'])
        self.assertEqual(result(v['rows'], 'B3')['status'], 'skipped')

    def test_B4_coverage(self):
        self.check_fails(lambda c: c['depth'].update(valid_fraction=0.4), 'B4')

    def test_missing_data_fails_as_not_measured(self):
        c = candidate()
        del c['depth']['vs_host']
        v = ac.judge(baseline(), c)
        self.assertFalse(v['pass'])
        self.assertEqual(result(v['rows'], 'B2')['status'], 'not measured')
        b = baseline()
        del b['gap_warnings_per_min']
        self.assertEqual(result(ac.judge(b, candidate())['rows'], 'A2')['status'], 'not measured')

    def test_restore_check_uses_only_A(self):
        restored = baseline()                                     # no depth: judged on VIO only
        v = ac.judge(baseline(), restored, want_depth=False)
        self.assertTrue(v['pass'])
        self.assertFalse(any(r['id'].startswith('B') for r in v['rows']))


if __name__ == '__main__':
    unittest.main()
