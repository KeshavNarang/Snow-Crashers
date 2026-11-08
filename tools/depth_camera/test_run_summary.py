"""Evals for run_summary.py: turning one recording into the inputs acceptance.judge() needs.

    python3 -m unittest -v test_run_summary

The recording holds raw observations (stamps, arrival times, cuVSLAM states, odometry positions, camera_info,
TF, a depth snapshot); summarise() must compute exactly the quantities the contract is defined on, and leave a
quantity out (so the contract says "not measured") rather than invent it.
"""
import math
import unittest

import numpy as np

import acceptance as ac
import run_summary as rs
from test_depth_analysis import K, floor_depth


def frames(hz=30.0, secs=10.0, t0=1000.0, age=0.04, drop=()):
    n = int(hz * secs)
    st = [t0 + i / hz for i in range(n) if i not in drop]
    return [(s + age, s) for s in st]                          # (arrival, header stamp)


INFO = {'height': 400, 'width': 640, 'distortion_model': 'rational_polynomial', 'd': [0.0] * 8,
        'k': [K[0], 0, K[2], 0, K[1], K[3], 0, 0, 1], 'r': [1, 0, 0, 0, 1, 0, 0, 0, 1],
        'p': [K[0], 0, K[2], 0, 0, K[1], K[3], 0, 0, 0, 1, 0], 'frame_id': 'oak_left_camera_optical_frame'}


def recording(**kw):
    r = {'secs': 10.0, 'left': frames(), 'right': frames(), 'depth': frames(age=0.06),
         'vo_states': [1] * 300, 'odom': [(1000.0 + i / 30, (0.001 * math.sin(i), 0.0, 0.0)) for i in range(300)],
         'camera_info': {'left': INFO, 'right': dict(INFO, frame_id='oak_right_camera_optical_frame'), 'depth': INFO},
         'tf': {'left': ((0.105, 0.038, 0.03), (0, 0.226, 0, 0.974)), 'right': ((0.105, -0.037, 0.03), (0, 0.226, 0, 0.974))},
         'depth_image': {'encoding': '16UC1', 'width': 640, 'height': 400, 'metres': floor_depth()},
         'expected_pitch_deg': 27.0}
    r.update(kw)
    return r


class Summarise(unittest.TestCase):
    def test_fields_the_contract_needs(self):
        s = rs.summarise(recording())
        self.assertAlmostEqual(s['left']['rate_hz'], 30.0, places=3)
        self.assertEqual(s['left']['missing'], 0)
        self.assertAlmostEqual(s['pair_fraction'], 1.0)
        self.assertAlmostEqual(s['tracking_fraction'], 1.0)
        self.assertAlmostEqual(s['odom_rate_hz'], 30.0, places=3)
        self.assertLess(s['static_drift_m'], 0.002)
        d = s['depth']
        self.assertEqual(d['encoding'], '16UC1')
        self.assertEqual(tuple(d['size']), (640, 400))
        self.assertEqual(tuple(d['info_size']), (640, 400))
        self.assertAlmostEqual(d['age_p95_ms'], 60.0, places=3)
        self.assertAlmostEqual(d['floor']['pitch_deg'], 27.0, delta=0.5)
        self.assertGreater(d['valid_fraction'], 0.3)

    def test_static_drift_is_the_largest_excursion(self):
        odom = [(1000.0 + i / 30, (0.0, 0.0, 0.0)) for i in range(300)]
        odom[150] = (odom[150][0], (0.03, 0.04, 0.0))             # one 5 cm excursion
        self.assertAlmostEqual(rs.summarise(recording(odom=odom))['static_drift_m'], 0.05, places=9)

    def test_frame_loss_is_counted(self):
        s = rs.summarise(recording(left=frames(drop={10, 11, 50})))
        self.assertEqual(s['left']['missing'], 3)

    def test_no_depth_means_no_depth_block(self):
        r = recording(depth=[], depth_image=None)
        r['camera_info'] = dict(r['camera_info'])
        del r['camera_info']['depth']
        self.assertNotIn('depth', rs.summarise(r))

    def test_missing_odometry_is_left_out_not_invented(self):
        s = rs.summarise(recording(odom=[]))
        self.assertNotIn('static_drift_m', s)
        self.assertNotIn('odom_rate_hz', s)

    def test_host_side_numbers_are_merged(self):
        s = rs.summarise(recording(), host={'gap_warnings_per_min': 12.0,
                                            'cpu': {'total_cores': 2.1, 'saturated_cores': 0}})
        self.assertEqual(s['gap_warnings_per_min'], 12.0)
        self.assertEqual(s['cpu']['saturated_cores'], 0)

    def test_summary_feeds_the_contract_end_to_end(self):
        host = {'gap_warnings_per_min': 10.0, 'cpu': {'total_cores': 2.0, 'saturated_cores': 0}}
        base = rs.summarise(recording(depth=[], depth_image=None), host=host)
        cand = rs.summarise(recording(), host=host)
        cand['depth']['vs_host'] = {'median_rel': 0.01, 'best_shift': 0}
        cand['depth']['host_valid_fraction'] = 0.2
        v = ac.judge(base, cand)
        self.assertTrue(v['pass'], [r for r in v['rows'] if not r['pass']])


class CpuJson(unittest.TestCase):
    def test_cpu_watch_json_to_contract_fields(self):
        cw = {'ncpu': 6, 'cpu_pct': 40.0, 'core_pct': [40.0, 95.0, 30.0, 35.0, 20.0, 20.0]}
        self.assertEqual(rs.cpu_from_watch(cw), {'total_cores': 2.4, 'saturated_cores': 1})


if __name__ == '__main__':
    unittest.main()
