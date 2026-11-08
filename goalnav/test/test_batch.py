"""localsim.batch: drone-measured scenarios, honest pass rates, paired A/B, deployability.

    cd goalnav && python -m unittest discover -s test
"""
import json
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from localsim import batch     # noqa: E402


class ScenarioTest(unittest.TestCase):
    def test_every_scenario_cites_a_source_and_maps_to_goalrun_flags(self):
        scenarios = batch.load_scenarios()
        self.assertIn('drone', scenarios)
        for name, s in scenarios.items():
            self.assertTrue(s.get('source'), f'{name} has no source')
            flags = batch.scenario_flags(name)
            self.assertEqual(len(flags) % 2, 0, name)
            for flag in flags[::2]:
                self.assertIn(flag, ('--camera-hz', '--camera-latency', '--vehicle'), name)

    def test_drone_scenario_is_the_measured_depth(self):
        # agents/logs/2026-11-07-goalnav-hardware.md: stereo_node, 2 worker threads, 4.0 Hz, 234 ms old.
        flags = batch.scenario_flags('drone')
        self.assertEqual(flags[flags.index('--camera-hz') + 1], '4.0')
        self.assertEqual(flags[flags.index('--camera-latency') + 1], '0.234')

    def test_drone_scenario_is_slower_than_the_sim_default(self):
        # The gap this guards: goalrun's defaults (--camera-hz 4.5, --camera-latency 0.13) are kinder
        # than the drone, so a batch run without a scenario overstates the pass rate.
        flags = batch.scenario_flags('drone')
        self.assertLess(float(flags[flags.index('--camera-hz') + 1]), 4.5)
        self.assertGreater(float(flags[flags.index('--camera-latency') + 1]), 0.13)

    def test_unknown_scenario_is_an_error(self):
        with self.assertRaises(KeyError):
            batch.scenario_flags('no-such-scenario')


class PassRateTest(unittest.TestCase):
    def test_wilson_lower_bound(self):
        self.assertAlmostEqual(batch.wilson_low(16, 16), 0.806, places=3)    # 16/16 is not "99 %"
        self.assertAlmostEqual(batch.wilson_low(0, 10), 0.0)
        self.assertGreater(batch.wilson_low(300, 300), 0.987)
        self.assertLess(batch.wilson_low(299, 300), batch.wilson_low(300, 300))
        self.assertEqual(batch.wilson_low(0, 0), 0.0)

    def test_runs_needed_for_a_target(self):
        # 99 % lower bound with no failures needs ~380 runs; 95 % needs ~73.
        self.assertEqual(batch.runs_needed(0.99), 381)
        self.assertEqual(batch.runs_needed(0.95), 73)

    def test_summary_reports_the_bound(self):
        rows = [_row(i, ok=True) for i in range(1, 17)]
        s = batch.summarise(rows)
        self.assertEqual(s['success'], 16)
        self.assertAlmostEqual(s['success_ci95_low'], 0.806, places=3)


class ReturnLegTest(unittest.TestCase):
    """The referee stops counting touches at the pad (referee.update -> _update_return),
    and goalrun's crash check is off once finished: a scrape on the way home shows
    nowhere in ``touches`` or ``crashed``. The batch must report it from clearance."""

    def test_return_scrape_is_counted_even_with_no_touches(self):
        rows = [_row(1, ok=True, touches=0, return_clear=-0.05), _row(2, ok=True, return_clear=0.06)]
        s = batch.summarise(rows)
        self.assertEqual(s['touch_runs'], 0)                 # the referee saw nothing...
        self.assertEqual(s['return_scrape_runs'], ['s1'])    # ...the batch does

    def test_home_time_and_full_success(self):
        rows = [_row(1, ok=True, home=85.), _row(2, ok=True, home=None), _row(3, ok=False)]
        s = batch.summarise(rows)
        self.assertEqual(s['home'], 1)                       # s2 reached the pad, never got home
        self.assertAlmostEqual(s['mean_home_s'], 85.)

    def test_old_rows_without_return_fields_still_summarise(self):
        row = _row(1, ok=True)
        for k in ('home_s', 'return_clear_m'):
            row.pop(k)
        s = batch.summarise([row])
        self.assertEqual(s['return_scrape_runs'], [])
        self.assertIsNone(s['mean_home_s'])


class SimErrorTest(unittest.TestCase):
    """Seen 2026-11-07: 48 runs at once, 18 died of MemoryError building their course voxels.
    A run the simulator never finished is not a flight failure, and not a pass."""

    def test_errors_are_neither_passes_nor_failures(self):
        rows = [_row(1, ok=True), _row(2, ok=False), {'run': 's3', 'error': 'no report'}]
        s = batch.summarise(rows)
        self.assertEqual(s['runs'], 2)                       # flown
        self.assertEqual(s['sim_errors'], ['s3'])
        self.assertEqual(s['success'], 1)
        self.assertAlmostEqual(s['success_rate'], 0.5)
        self.assertAlmostEqual(s['success_ci95_low'], round(batch.wilson_low(1, 2), 3))
        self.assertEqual([f['run'] for f in s['failures']], ['s2'])

    def test_missing_reports_are_found_for_a_retry(self):
        with tempfile.TemporaryDirectory() as tmp:
            for name, done in (('s1', True), ('s2', False), ('s3', True)):
                (Path(tmp) / name).mkdir()
                if done:
                    (Path(tmp) / name / 'report.json').write_text('{}')
            runs = [Path(tmp) / n for n in ('s1', 's2', 's3')]
            self.assertEqual([p.name for p in batch.missing(runs)], ['s2'])

    def test_compare_skips_seeds_either_side_did_not_fly(self):
        with tempfile.TemporaryDirectory() as tmp:
            a, b = Path(tmp) / 'a', Path(tmp) / 'b'
            _write(a, [_row(1, ok=True), {'run': 's2', 'error': 'no report'}])
            _write(b, [_row(1, ok=True), _row(2, ok=False)])
            c = batch.compare(a, b)
        self.assertEqual(c['shared'], 1)
        self.assertEqual(c['b_breaks'], [])


class DeployableTest(unittest.TestCase):
    def test_only_the_launch_default_profile_without_tune_is_deployable(self):
        # The drone starts `ros2 launch drone_ref goalnav.launch.py` with no arguments
        # (skybetav1/container_scripts/run_competitor.sh): profile safe, empty tuning.
        self.assertTrue(batch.deployable('safe', []))
        self.assertFalse(batch.deployable('safe', ['--tune', 'radius=0.48']))
        self.assertFalse(batch.deployable('fast', []))
        self.assertFalse(batch.deployable('safe', ['--set', 'profile=fast']))

    def test_the_launch_default_is_read_from_the_launch_file(self):
        self.assertEqual(batch.launch_default_profile(), 'safe')


class CompareTest(unittest.TestCase):
    def test_paired_on_shared_seeds(self):
        with tempfile.TemporaryDirectory() as tmp:
            a, b = Path(tmp) / 'a', Path(tmp) / 'b'
            _write(a, [_row(1, ok=True, t=60, clear=0.05), _row(2, ok=False), _row(3, ok=True, t=70, clear=0.02)])
            _write(b, [_row(1, ok=True, t=66, clear=0.09), _row(2, ok=True, t=70, clear=0.04),
                       _row(4, ok=True, t=50, clear=0.1)])
            c = batch.compare(a, b)
        self.assertEqual(c['shared'], 2)                     # seeds 1 and 2; 3 and 4 are not paired
        self.assertEqual(c['b_fixes'], ['s2'])
        self.assertEqual(c['b_breaks'], [])
        self.assertAlmostEqual(c['mean_dt_s'], 6.0)          # seed 1 only: both finished
        self.assertAlmostEqual(c['mean_dclear_m'], 0.04)


def _row(seed, ok, t=65., clear=0.05, touches=0, home=88., return_clear=0.05):
    return {'run': f's{seed}', 'ok': ok, 'finish_s': t if ok else None, 'total_s': None, 'checkpoints': 5 if ok else 4,
            'touches': touches, 'crashed': None if ok else [90., 'pushed deep into a wall'], 'final': ['DONE', ''],
            'clear_m': clear, 'clear_at': {}, 'window_clear_m': 0.08, 'under_target_s': 10.,
            'home_s': home if ok else None, 'return_clear_m': return_clear}


def _write(directory, rows):
    directory.mkdir(parents=True)
    (directory / 'scores.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in rows))


if __name__ == '__main__':
    unittest.main()
