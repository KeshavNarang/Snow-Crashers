"""checks.py: the pre-flight sensor verdicts, from made-up statistics."""
from pathlib import Path
import math
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from overwall_nav.checks import evaluate, imu_pitch_deg, oak_imu_pitch_deg, verdict   # noqa: E402

GOOD = {
    'duration': 10.0, 'left_images': 300, 'right_images': 300, 'imu': 2000, 'odom': 300, 'fronts': 100,
    'left_info': True, 'right_info': True, 'baseline_m': 0.075, 'valid_median': 0.3, 'proc_ms_median': 35,
    'age_ms_median': 70, 'front_on_ground': None, 'floor_pitch_deg': 27.2, 'floor_height_m': 0.14,
    'imu_pitch_deg': 26.5, 'imu_g': 9.8, 'tf_pitch_deg': 27.0, 'vo_state': 1,
    'state': {'px4_link': True, 'armed': False, 'landed': True, 'local_enu': [0, 0, -0.05],
              'origin_enu': [0, 0, 0.0]},
}


def by_name(results):
    return {name: (status, detail) for status, name, detail in results}


class ChecksTest(unittest.TestCase):
    def test_all_good_is_ready(self):
        results = evaluate(GOOD)
        self.assertEqual(verdict(results), 'READY', results)

    def test_every_sensor_is_checked(self):
        names = set(by_name(evaluate(GOOD)))
        for needed in ('camera left', 'camera right', 'camera calibration', 'stereo TF', 'stereo_depth',
                       'stereo matching', 'camera tilt (floor)', 'camera tilt (IMU)', 'stack camera TF', 'IMU',
                       'IMU gravity', 'cuVSLAM odometry', 'cuVSLAM tracking', 'PX4 link', 'PX4 position',
                       'landed', 'disarmed', 'PX4 vs vision height', 'obstacle check'):
            self.assertIn(needed, names)

    def test_skybeta_03_today_warns_about_the_tilted_stack_tf(self):
        stats = dict(GOOD, tf_pitch_deg=0.3, state=dict(GOOD['state'], local_enu=[0, 0, -0.45]))
        results = by_name(evaluate(stats))
        self.assertEqual(results['stack camera TF'][0], 'WARN')
        self.assertIn('CAMERA_PITCH', results['stack camera TF'][1])
        self.assertEqual(results['PX4 vs vision height'][0], 'WARN')
        self.assertEqual(verdict(evaluate(stats)), 'READY (with warnings)')

    def test_wrong_mount_pitch_fails(self):
        results = by_name(evaluate(dict(GOOD, floor_pitch_deg=12.0, imu_pitch_deg=12.5)))
        self.assertEqual(results['camera tilt (IMU)'][0], 'FAIL')

    def test_bad_floor_fit_only_warns_when_the_imu_agrees(self):
        # skybeta-03, 2026-11-07: IMU 25.6, floor fit 51.8 / 0.19 m with little floor in view.
        stats = dict(GOOD, imu_pitch_deg=25.6, floor_pitch_deg=51.8, floor_height_m=0.19, tf_pitch_deg=27.6)
        results = by_name(evaluate(stats))
        self.assertEqual(results['camera tilt (IMU)'][0], 'PASS')
        self.assertEqual(results['camera tilt (floor)'][0], 'WARN')
        self.assertEqual(results['stack camera TF'][0], 'PASS')     # compared with the IMU, not the fit
        self.assertNotEqual(verdict(evaluate(stats)), 'NOT READY')

    def test_missing_streams_fail(self):
        for key, name in (('left_images', 'camera left'), ('imu', 'IMU'), ('odom', 'cuVSLAM odometry'),
                          ('fronts', 'stereo_depth')):
            results = by_name(evaluate(dict(GOOD, **{key: 0})))
            self.assertEqual(results[name][0], 'FAIL', name)

    def test_px4_problems(self):
        self.assertEqual(verdict(evaluate(dict(GOOD, state=None))), 'NOT READY')
        r = by_name(evaluate(dict(GOOD, state=dict(GOOD['state'], px4_link=False))))
        self.assertEqual(r['PX4 link'][0], 'FAIL')
        r = by_name(evaluate(dict(GOOD, state=dict(GOOD['state'], local_enu=None))))
        self.assertEqual(r['PX4 position'][0], 'FAIL')
        r = by_name(evaluate(dict(GOOD, vo_state=2)))
        self.assertEqual(r['cuVSLAM tracking'][0], 'FAIL')

    def test_obstacle_on_the_ground_warns(self):
        r = by_name(evaluate(dict(GOOD, front_on_ground=0.4)))
        self.assertEqual(r['obstacle check'][0], 'WARN')

    def test_imu_pitch_from_gravity(self):
        for pitch in (0.0, 27.0, 45.0):
            p = math.radians(pitch)
            up = (0.0, -math.cos(p) * 9.81, -math.sin(p) * 9.81)   # still IMU reads +g upwards
            self.assertAlmostEqual(imu_pitch_deg(up), pitch, places=6)


    def test_imu_pitch_from_the_oak_imu_axes(self):
        # skybeta-03 flat on a table with the 27 deg holder, 2026-11-07: [8.5, -0.24, -4.06]
        self.assertAlmostEqual(oak_imu_pitch_deg((8.5, -0.24, -4.06)), 25.5, delta=0.5)
        self.assertAlmostEqual(oak_imu_pitch_deg((9.81, 0.0, 0.0)), 0.0, delta=1e-6)


if __name__ == '__main__':
    unittest.main()
