"""Evals for cpu_watch.py and topic_rates.py. They need no drone and no ROS:

    cd tools/drone_tools && python3 -m unittest -v

The bottleneck detector is only useful if (a) it names the right culprit and (b) it stays quiet when
nothing is wrong, so both directions are tested. A live end-to-end eval for the drone is in
eval_live.sh.
"""
import unittest

import numpy as np

import cpu_watch as cw
import topic_rates as tr

NCPU = cw.NCPU


def stat_line(comm, utime, stime, cpu, pid=1):
    """A valid 52-field /proc/<pid>/task/<tid>/stat line."""
    rest = ['S'] + ['0'] * 51                      # fields 3..52
    rest[11], rest[12] = str(utime), str(stime)    # fields 14, 15
    rest[36] = str(cpu)                            # field 39
    return f'{pid} ({comm}) ' + ' '.join(rest)


def fake_proc(pid, label, cmd='', stack=True, container=False, uid=0, role=''):
    cw._meta[pid] = {'cmd': cmd or label, 'label': label, 'role': role, 'uid': uid,
                     'container': container, 'stack': stack}


def report(core_pct, procs, run=1.0, misc=None, peak=None):
    """procs: {pid: (cores, top_thread_name, top_thread_cores, {core: cores})}"""
    cores = list(core_pct) + [5.0] * (NCPU - len(core_pct))
    p = {}
    for pid, (c, tn, tc, on) in procs.items():
        p[pid] = {'cores': c, 'peak': c, 'top': (tn, tc, next(iter(on), 0)), 'on': dict(on)}
    base = {'mem_avail_mb': 4000, 'swap_used_mb': 0, 'freq': [], 'temp_max': 50.0, 'gpu_pct': 0.0}
    base.update(misc or {})
    return {'cores': cores, 'peak': peak or cores, 'procs': p, 'run': run, 'misc': base, 'secs': 2.0, 'n': 1}


class ParseStat(unittest.TestCase):
    def test_plain(self):
        self.assertEqual(cw.parse_stat(stat_line('python3', 100, 20, 3)), ('python3', 120, 3))

    def test_name_with_spaces_and_parens(self):
        # kernel threads and tmux use names like "tmux: server" and "(sd-pam)"
        self.assertEqual(cw.parse_stat(stat_line('tmux: server', 7, 3, 1))[1:], (10, 1))
        self.assertEqual(cw.parse_stat(stat_line('(sd-pam)', 5, 5, 0))[1:], (10, 0))


class Verdict(unittest.TestCase):
    def setUp(self):
        cw._meta.clear()

    def test_quiet_system_says_nothing(self):
        fake_proc(10, 'vslam_container', stack=True)
        r = report([20] * NCPU, {10: (0.4, 'vslam', 0.3, {0: 0.4})})
        self.assertEqual(cw.verdict(r), [])
        self.assertIn('no bottleneck found', cw.render(r))

    def test_saturated_core_names_who_ran_on_it(self):
        fake_proc(10, 'stereo_depth')
        fake_proc(11, 'oak_container')
        busy = [100.0] + [20.0] * (NCPU - 1)
        r = report(busy, {10: (0.9, 'stereo', 0.5, {0: 0.8}), 11: (0.3, 'oak', 0.2, {0: 0.15})})
        v = cw.verdict(r)
        self.assertTrue(any(x.startswith('CORE 0 SATURATED') and 'stereo_depth' in x for x in v), v)
        self.assertLess(next(x for x in v if 'CORE 0' in x).index('stereo_depth'),
                        next(x for x in v if 'CORE 0' in x).index('oak_container'), 'heaviest first')

    def test_single_thread_bound_is_called_out(self):
        fake_proc(10, 'stereo_depth')
        r = report([30] * NCPU, {10: (1.4, 'stereo', 0.97, {0: 0.97, 1: 0.43})})
        self.assertTrue(any('SINGLE-THREAD BOUND' in x and 'stereo_depth' in x for x in cw.verdict(r)))

    def test_many_light_threads_are_not_single_thread_bound(self):
        fake_proc(10, 'oak_container')
        r = report([30] * NCPU, {10: (1.3, 'worker', 0.3, {0: 0.3})})
        self.assertFalse(any('SINGLE-THREAD' in x for x in cw.verdict(r)))

    def test_overcommit_by_utilisation_and_by_run_queue(self):
        self.assertTrue(any('OVER-COMMITTED' in x for x in cw.verdict(report([95] * NCPU, {}))))
        self.assertTrue(any('OVER-COMMITTED' in x for x in cw.verdict(report([50] * NCPU, {}, run=NCPU + 4))))
        self.assertFalse(any('OVER-COMMITTED' in x for x in cw.verdict(report([50] * NCPU, {}, run=2))))

    def test_stray_process_is_flagged_but_stack_is_not(self):
        fake_proc(20, 'bash', cmd='bash -c while true; do :; done', stack=False, uid=1000)
        fake_proc(21, 'stereo_depth', stack=True)
        r = report([40] * NCPU, {20: (1.0, 'bash', 1.0, {2: 1.0}), 21: (1.5, 'stereo', 0.5, {1: 0.5})})
        v = cw.verdict(r)
        self.assertTrue(any('STRAY' in x and 'pid 20' in x for x in v), v)
        self.assertFalse(any('STRAY' in x and 'pid 21' in x for x in v), v)

    def test_small_stray_is_ignored(self):
        fake_proc(20, 'sshd', stack=False)
        self.assertEqual(cw.verdict(report([10] * NCPU, {20: (0.05, 'sshd', 0.05, {0: 0.05})})), [])

    def test_throttling_heat_and_memory(self):
        misc = {'freq': [(900000, 1728000)] * NCPU, 'temp_max': 86.0, 'mem_avail_mb': 200, 'swap_used_mb': 900}
        v = ' | '.join(cw.verdict(report([10] * NCPU, {}, misc=misc)))
        for word in ('CLOCKS BELOW MAX', 'HOT', 'MEMORY PRESSURE'):
            self.assertIn(word, v)

    def test_full_speed_clocks_are_fine(self):
        misc = {'freq': [(1728000, 1728000)] * NCPU}
        self.assertEqual(cw.verdict(report([10] * NCPU, {}, misc=misc)), [])


class SysfsReads(unittest.TestCase):
    """Found on the drone: reading a thermal_zone file raised TypeError (not OSError) from the codec
    layer. A monitor that dies on a sensor file is useless exactly when the machine is struggling."""

    def test_read_int_survives_any_read_failure(self):
        import builtins
        real_open = builtins.open
        for exc in (OSError('gone'), TypeError("can't concat NoneType to bytes"), ValueError('x'), PermissionError()):
            def fake(path, *a, **k):
                if 'fake_sysfs' in str(path):
                    raise exc
                return real_open(path, *a, **k)
            builtins.open = fake
            try:
                self.assertIsNone(cw.read_int('/sys/fake_sysfs/temp'), type(exc).__name__)
            finally:
                builtins.open = real_open

    def test_read_int_parses_a_number(self):
        import os, tempfile
        with tempfile.NamedTemporaryFile('w', delete=False) as f:
            f.write('51125\n')
        try:
            self.assertEqual(cw.read_int(f.name), 51125)
        finally:
            os.unlink(f.name)

    def test_read_int_of_garbage_is_none(self):
        import os, tempfile
        with tempfile.NamedTemporaryFile('w', delete=False) as f:
            f.write('not a number')
        try:
            self.assertIsNone(cw.read_int(f.name))
        finally:
            os.unlink(f.name)


class VanishedProcesses(unittest.TestCase):
    def test_meta_of_a_pid_that_does_not_exist_is_none(self):
        cw._meta.clear()
        self.assertIsNone(cw.meta(4194303))          # above the kernel's pid limit: cannot exist

    def test_meta_survives_a_comm_file_that_disappears(self):
        # cmdline of a kernel thread is empty, which makes meta() fall back to /proc/<pid>/comm
        import builtins
        real = builtins.open
        cw._meta.clear()

        def fake(path, *a, **k):
            if str(path).endswith('/comm'):
                raise FileNotFoundError(path)
            return real(path, *a, **k)
        builtins.open = fake
        try:
            for pid in (2,):                              # kthreadd on Linux; absent elsewhere: either is fine
                cw.meta(pid)                              # must not raise
        finally:
            builtins.open = real


HEX = 'a1b2c3d4' * 8


class ContainerDetection(unittest.TestCase):
    def test_real_containers(self):
        self.assertTrue(cw.in_container(f'0::/system.slice/docker-{HEX}.scope\n'))
        self.assertTrue(cw.in_container(f'12:cpu,cpuacct:/docker/{HEX}\n'))
        self.assertTrue(cw.in_container(f'0::/kubepods/burstable/pod1/{HEX}\n'))

    def test_docker_daemon_itself_is_the_host(self):
        self.assertFalse(cw.in_container('0::/system.slice/docker.service\n'))
        self.assertFalse(cw.in_container('0::/system.slice/containerd.service\n'))

    def test_plain_host_process(self):
        self.assertFalse(cw.in_container('0::/user.slice/user-1000.slice/session-3.scope\n'))
        self.assertFalse(cw.in_container(''))


class OwnProcessIgnored(unittest.TestCase):
    def test_monitor_does_not_flag_itself(self):
        import os
        cw._meta.clear()
        fake_proc(os.getpid(), 'cpu_watch.py', stack=False, uid=1000)
        r = report([10] * NCPU, {os.getpid(): (0.5, 'python3', 0.5, {0: 0.5})})
        self.assertEqual([x for x in cw.verdict(r) if 'STRAY' in x], [])


class Discovery(unittest.TestCase):
    """ROS needs a moment to discover topics after a node starts; asking once, immediately, found nothing."""

    class FakeNode:
        def __init__(self, appear_after):
            self.calls, self.appear_after = 0, appear_after

        def get_topic_names_and_types(self):
            self.calls += 1
            return [] if self.calls < self.appear_after else [('/a', ['sensor_msgs/msg/Image']), ('/b', ['x/msg/Y'])]

    def test_waits_until_topics_appear(self):
        n = self.FakeNode(appear_after=4)
        found = tr.resolve_types(n, ['/a'], timeout=5.0, poll=0.0)
        self.assertEqual(found, {'/a': 'sensor_msgs/msg/Image'})
        self.assertGreaterEqual(n.calls, 4)

    def test_gives_up_on_topics_that_never_appear(self):
        n = self.FakeNode(appear_after=10 ** 9)
        self.assertEqual(tr.resolve_types(n, ['/a'], timeout=0.05, poll=0.0), {})

    def test_returns_as_soon_as_all_are_found(self):
        n = self.FakeNode(appear_after=1)
        tr.resolve_types(n, ['/a', '/b'], timeout=5.0, poll=0.0)
        self.assertEqual(n.calls, 1)


class Merge(unittest.TestCase):
    def test_mean_and_peak(self):
        def s(c0, cores):
            return {'cores': [c0] + [10.0] * (NCPU - 1), 'run': 1.0, 'dt': 1.0, 'misc': {},
                    'procs': {7: {'cores': cores, 'top': ('t', cores, 0), 'on': {0: cores}}}}
        m = cw.merge([s(100.0, 1.0), s(0.0, 0.0)])
        self.assertAlmostEqual(m['cores'][0], 50.0)
        self.assertAlmostEqual(m['peak'][0], 100.0)
        self.assertAlmostEqual(m['procs'][7]['cores'], 0.5)
        self.assertAlmostEqual(m['procs'][7]['peak'], 1.0)


class Json(unittest.TestCase):
    def test_json_roundtrip(self):
        import json
        cw._meta.clear()
        fake_proc(10, 'stereo_depth')
        out = cw.to_json(report([95] * NCPU, {10: (1.2, 'stereo', 0.9, {0: 0.9})}))
        back = json.loads(json.dumps(out))
        self.assertEqual(back['procs'][0]['label'], 'stereo_depth')
        self.assertTrue(back['verdict'])


class Cdr(unittest.TestCase):
    def test_little_endian_stamp(self):
        raw = bytes([0, 1, 0, 0]) + (1794097481).to_bytes(4, 'little', signed=True) + (500_000_000).to_bytes(4, 'little')
        self.assertAlmostEqual(tr.cdr_stamp(raw), 1794097481.5)

    def test_big_endian_stamp(self):
        raw = bytes([0, 0, 0, 0]) + (12).to_bytes(4, 'big', signed=True) + (250_000_000).to_bytes(4, 'big')
        self.assertAlmostEqual(tr.cdr_stamp(raw), 12.25)

    def test_short_message_is_none(self):
        self.assertIsNone(tr.cdr_stamp(b'\x00\x01\x00'))


class Summarise(unittest.TestCase):
    def rec(self, gaps, age=0.05, t0=1000.0):
        arr = t0 + np.concatenate([[0.0], np.cumsum(gaps)])
        return {'arr': list(arr), 'stamp': list(arr - age)}

    def test_steady_30hz(self):
        out = tr.summarise({'/a': self.rec([1 / 30] * 299)}, 10.0)['/a']
        self.assertAlmostEqual(out['hz'], 30.0, delta=0.1)
        self.assertEqual(out['late_gaps'], 0)
        self.assertEqual(out['age_median_ms'], 50)

    def test_drops_show_up_as_late_gaps_and_worst_gap(self):
        gaps = [1 / 30] * 100
        gaps[40] = 0.4
        gaps[70] = 0.2
        out = tr.summarise({'/a': self.rec(gaps)}, 3.4)['/a']
        self.assertEqual(out['late_gaps'], 2)
        self.assertEqual(out['worst_gap_ms'], 400)

    def test_latency_percentiles(self):
        r = self.rec([0.1] * 99)
        r['stamp'] = [a - (0.5 if i == 50 else 0.05) for i, a in enumerate(r['arr'])]
        out = tr.summarise({'/a': r}, 10.0)['/a']
        self.assertEqual(out['age_median_ms'], 50)
        self.assertEqual(out['age_max_ms'], 500)

    def test_missing_and_sparse_topics_report_errors_not_crashes(self):
        out = tr.summarise({'/gone': None, '/few': {'arr': [1.0], 'stamp': [1.0]}}, 10.0)
        self.assertIn('error', out['/gone'])
        self.assertIn('error', out['/few'])

    def test_stereo_pair_offset(self):
        left = self.rec([1 / 30] * 99)
        right = {'arr': left['arr'], 'stamp': [s + 1 / 30 for s in left['stamp']]}   # one frame late
        out = tr.summarise({'/oak/left/image_mono': left, '/oak/right/image_mono': right}, 3.3)
        self.assertAlmostEqual(out['/oak/left/image_mono vs /oak/right/image_mono']['stamp_offset_median_ms'], 0.0, delta=0.5)
        # shifted by exactly one frame period the nearest right frame is the next one: offset ~0, not 33 ms,
        # so the check must flag a *constant* offset smaller than the period instead
        right2 = {'arr': left['arr'], 'stamp': [s + 0.012 for s in left['stamp']]}
        out2 = tr.summarise({'/oak/left/image_mono': left, '/oak/right/image_mono': right2}, 3.3)
        self.assertAlmostEqual(out2['/oak/left/image_mono vs /oak/right/image_mono']['stamp_offset_median_ms'], 12.0, delta=0.5)


if __name__ == '__main__':
    unittest.main()
