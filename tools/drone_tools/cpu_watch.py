#!/usr/bin/env python3
"""cpu_watch: who is using the Jetson's cores, and is anything bottlenecking them?

Stdlib only; reads /proc, so it needs no root. Run it on the drone's HOST (not inside the
container) so it sees every process, container ones included:

    python3 cpu_watch.py                 live view, refreshed every 2 s (Ctrl-C to stop)
    python3 cpu_watch.py --once 15       sample 15 s, print one report with a verdict, exit
    python3 cpu_watch.py --once 15 --json   the same report as JSON (for scripts / logs)
    python3 cpu_watch.py --log run.jsonl    live view and append one JSON line per sample

What it reports
  * per-core utilisation (mean and peak) and the system run queue;
  * per-process CPU in "cores" (1.00 = one full core), its busiest thread, and a label that says
    what the process is (ROS node name, launch file, container or host);
  * a VERDICT: cores that are saturated and who ran on them, single-thread-bound processes
    (more cores will not help them), CPU over-commit, stray host processes that are not part of
    the stack, clock throttling, heat, and low memory.

Nothing here changes the system: it only reads. CPU percentages are over the sample interval.
"""
import argparse
import glob
import json
import os
import re
import sys
import time

TICK = os.sysconf('SC_CLK_TCK')
NCPU = os.cpu_count() or 1

# Substring of the command line -> what it is. First match wins.
ROLES = [
    ('oak_container', 'camera driver (depthai, host side)'),
    ('vslam_container', 'cuVSLAM visual odometry'),
    ('stereo_depth', 'our stereo depth (OpenCV SGBM, CPU)'),
    ('flight_interface', 'flight_interface (organizers)'),
    ('px4_odom_bridge', 'odometry -> PX4 bridge'),
    ('MicroXRCEAgent', 'uXRCE-DDS agent'),
    ('overwall_mission', 'our mission node'),
    ('mission_node', 'our mission node'),
    ('sensor_check', 'sensor_check'),
    ('image_format_converter', 'mono8 converter'),
    ('ros2-daemon', 'ros2 CLI daemon'),
    ('ros2 launch', 'ros2 launch'),
    ('tmux', 'tmux'),
    ('static_transform_publisher', 'static TF'),
    ('skybeta-status', 'organizer status agent'),
    ('skybeta_status', 'organizer status agent'),
    ('tailscaled', 'tailscale (organizers)'),
    ('tegrastats', 'tegrastats'),
    ('dockerd', 'docker'),
    ('containerd', 'docker'),
]
# Processes that belong to the stack: never flagged as strays even when busy.
STACK = ('oak_container', 'vslam_container', 'stereo_depth', 'flight_interface', 'px4_odom_bridge',
         'MicroXRCEAgent', 'overwall_mission', 'mission_node', 'image_format_converter',
         'static_transform_publisher', 'ros2 launch', 'ros2-daemon', 'dockerd', 'containerd',
         'skybeta', 'tailscaled', 'tmux', 'sensor_check', 'tegrastats')

SAT_CORE = 90.0        # a core this busy is saturated
SAT_THREAD = 0.85      # a thread using this much of one core is single-thread-bound
OVERCOMMIT = 85.0      # whole-machine utilisation %
STRAY_MIN = 0.25       # a non-stack process using this many cores is worth a line


def read_cpu():
    """(per-core [busy, total] jiffies, procs_running)"""
    cores, running = [], 0
    with open('/proc/stat') as f:
        for line in f:
            if re.match(r'cpu\d+ ', line):
                v = [int(x) for x in line.split()[1:]]
                idle = v[3] + (v[4] if len(v) > 4 else 0)
                cores.append((sum(v[:8]) - idle, sum(v[:8])))
            elif line.startswith('procs_running'):
                running = int(line.split()[1])
    return cores, running


def parse_stat(text):
    """(comm, utime+stime ticks, last cpu) from a /proc/<pid>/task/<tid>/stat line."""
    i = text.rindex(')')
    comm = text[text.index('(') + 1:i]
    rest = text[i + 2:].split()
    return comm, int(rest[11]) + int(rest[12]), int(rest[36])


_meta = {}
_CONTAINER_CG = re.compile(r'(/docker/[0-9a-f]{12,}|docker-[0-9a-f]{12,}\.scope|/kubepods|libpod-[0-9a-f]{12,}\.scope)')


def in_container(cgroup_text):
    """True for a process inside a container. The docker daemon itself (docker.service) is the host."""
    return bool(_CONTAINER_CG.search(cgroup_text))



def meta(pid):
    """Cached identity of a process: cmdline, label, whether it is in a container, owner."""
    m = _meta.get(pid)
    if m is not None:
        return m
    try:
        cmd = open(f'/proc/{pid}/cmdline', 'rb').read().replace(b'\0', b' ').decode(errors='replace').strip()
        uid = os.stat(f'/proc/{pid}').st_uid
        cg = open(f'/proc/{pid}/cgroup').read()
        cmd = cmd or open(f'/proc/{pid}/comm').read().strip()
    except Exception:                                  # the process exited while we were looking
        return None
    node = re.search(r'__node:=(\S+)', cmd)
    role = next((r for key, r in ROLES if key in cmd), '')
    if node:
        label = node.group(1)
    elif 'ros2 launch' in cmd:
        m2 = re.search(r'ros2 launch (\S+) (\S+)', cmd)       # "ros2 launch <pkg> <file> ..."
        label = f'launch:{m2.group(2)}' if m2 else 'launch'
    else:
        argv = cmd.split()
        exe = os.path.basename(argv[0]) if argv else '?'
        if exe.startswith('python') and len(argv) > 1:
            exe = os.path.basename(next((a for a in argv[1:] if not a.startswith('-')), argv[1]))
        label = exe
    m = {'cmd': cmd, 'label': label, 'role': role, 'uid': uid,
         'container': in_container(cg),
         'stack': any(k in cmd for k in STACK)}
    _meta[pid] = m
    return m


def read_threads():
    """{(pid, tid): (comm, ticks, cpu)} for every thread on the machine."""
    out = {}
    for tdir in glob.glob('/proc/[0-9]*/task/[0-9]*'):
        parts = tdir.split('/')
        try:
            out[(int(parts[2]), int(parts[4]))] = parse_stat(open(tdir + '/stat').read())
        except Exception:                              # thread exited between the glob and the read
            continue
    return out


def read_int(path):
    """An integer from a sysfs/proc file, or None if it cannot be read for ANY reason (a sensor that
    vanishes or misbehaves must never kill the monitor; on the Jetson a thermal read raised TypeError)."""
    try:
        with open(path, 'rb') as f:
            return int(f.read().split()[0])
    except Exception:
        return None


def read_misc():
    mem = {}
    with open('/proc/meminfo') as f:
        for line in f:
            k, v = line.split(':')
            mem[k] = int(v.split()[0])
    freq = []
    for i in range(NCPU):
        cur = read_int(f'/sys/devices/system/cpu/cpu{i}/cpufreq/scaling_cur_freq')
        top = read_int(f'/sys/devices/system/cpu/cpu{i}/cpufreq/cpuinfo_max_freq')
        if cur is not None and top:
            freq.append((cur, top))
    temps = [t / 1000.0 for t in (read_int(p) for p in glob.glob('/sys/devices/virtual/thermal/thermal_zone*/temp'))
             if t is not None]
    gpu_files = glob.glob('/sys/devices/platform/bus@0/*.gpu/load')
    load = read_int(gpu_files[0]) if gpu_files else None
    gpu = load / 10.0 if load is not None else None
    return {'mem_avail_mb': mem.get('MemAvailable', 0) // 1024,
            'swap_used_mb': (mem.get('SwapTotal', 0) - mem.get('SwapFree', 0)) // 1024,
            'freq': freq, 'temp_max': max(temps) if temps else None, 'gpu_pct': gpu}


def sample(prev, interval):
    """Sleep `interval`, then return (new_prev, sample dict) for the interval just ended."""
    time.sleep(interval)
    now = {'t': time.time(), 'cpu': read_cpu(), 'thr': read_threads()}
    dt = now['t'] - prev['t']
    cores = []
    for (b0, t0), (b1, t1) in zip(prev['cpu'][0], now['cpu'][0]):
        cores.append(100.0 * (b1 - b0) / max(t1 - t0, 1))
    procs = {}
    for key, (comm, ticks, cpu) in now['thr'].items():
        old = prev['thr'].get(key)
        d = (ticks - old[1]) if old else 0       # a thread that appeared mid-interval counts from 0
        if d <= 0:
            continue
        pid = key[0]
        p = procs.setdefault(pid, {'cores': 0.0, 'top': ('', 0.0, 0), 'on': {}})
        c = d / TICK / dt
        p['cores'] += c
        p['on'][cpu] = p['on'].get(cpu, 0.0) + c
        if c > p['top'][1]:
            p['top'] = (comm, c, cpu)
    s = {'t': now['t'], 'dt': dt, 'cores': cores, 'run': now['cpu'][1], 'procs': procs, 'misc': read_misc()}
    return now, s


def merge(samples):
    """Average a list of samples into one report-shaped dict (with per-core peaks)."""
    n = len(samples)
    cores = [sum(s['cores'][i] for s in samples) / n for i in range(NCPU)]
    peak = [max(s['cores'][i] for s in samples) for i in range(NCPU)]
    procs = {}
    for s in samples:
        for pid, p in s['procs'].items():
            q = procs.setdefault(pid, {'cores': 0.0, 'peak': 0.0, 'top': ('', 0.0, 0), 'on': {}})
            q['cores'] += p['cores'] / n
            q['peak'] = max(q['peak'], p['cores'])
            if p['top'][1] > q['top'][1]:
                q['top'] = p['top']
            for c, v in p['on'].items():
                q['on'][c] = q['on'].get(c, 0.0) + v / n
    last = samples[-1]['misc']
    return {'cores': cores, 'peak': peak, 'procs': procs, 'run': sum(s['run'] for s in samples) / n,
            'misc': last, 'secs': sum(s['dt'] for s in samples), 'n': n}


def verdict(r):
    """Plain-language findings, most important first. Empty list means nothing is wrong."""
    out = []
    total = sum(r['cores']) / NCPU
    named = []
    for pid, p in r['procs'].items():
        m = meta(pid)
        if m:
            named.append((pid, p, m))
    named.sort(key=lambda x: -x[1]['cores'])

    for i, (u, pk) in enumerate(zip(r['cores'], r['peak'])):
        if u >= SAT_CORE:
            who = sorted(((p['on'].get(i, 0.0), m['label']) for _, p, m in named if p['on'].get(i, 0.0) > 0.05), reverse=True)[:3]
            out.append(f'CORE {i} SATURATED ({u:.0f}% mean): ' + ', '.join(f'{l} {c:.2f}' for c, l in who))
    for pid, p, m in named:
        comm, c, cpu = p['top']
        if c >= SAT_THREAD:
            out.append(f'SINGLE-THREAD BOUND: {m["label"]} (pid {pid}) thread "{comm}" uses {c:.2f} of one core; '
                       f'more cores will not speed it up, only less work or a faster path will')
    if total >= OVERCOMMIT or r['run'] > NCPU + 1:
        out.append(f'CPU OVER-COMMITTED: {total:.0f}% of all {NCPU} cores in use, run queue {r["run"]:.1f} '
                   f'(> {NCPU} means tasks are waiting for a core); real-time nodes will stall')
    for pid, p, m in named:
        if not m['stack'] and p['cores'] >= STRAY_MIN and pid != os.getpid():
            where = 'in a container' if m['container'] else 'on the host'
            out.append(f'STRAY / NON-STACK PROCESS: pid {pid} "{m["cmd"][:90]}" uses {p["cores"]:.2f} cores {where} '
                       f'(uid {m["uid"]}); is it supposed to be running?')
    ms = r['misc']
    if ms['freq']:
        slow = [i for i, (c, t) in enumerate(ms['freq']) if c < 0.9 * t]
        if slow:
            out.append(f'CLOCKS BELOW MAX on cores {slow} ({ms["freq"][slow[0]][0] // 1000} of '
                       f'{ms["freq"][slow[0]][1] // 1000} MHz): thermal or power limit, or an idle governor')
    if ms['temp_max'] and ms['temp_max'] >= 80:
        out.append(f'HOT: {ms["temp_max"]:.0f} C, throttling is near')
    if ms['mem_avail_mb'] < 500 or ms['swap_used_mb'] > 200:
        out.append(f'MEMORY PRESSURE: {ms["mem_avail_mb"]} MB available, {ms["swap_used_mb"]} MB swap in use')
    return out


def render(r, top=10, header=''):
    lines = []
    total = sum(r['cores']) / NCPU
    ms = r['misc']
    lines.append(f'{header}CPU {total:5.1f}% of {NCPU} cores | run queue {r["run"]:.1f}'
                 + (f' | GPU {ms["gpu_pct"]:.0f}%' if ms['gpu_pct'] is not None else '')
                 + (f' | {ms["temp_max"]:.0f} C' if ms['temp_max'] else '')
                 + f' | mem free {ms["mem_avail_mb"]} MB')
    lines.append('cores  ' + ' '.join(f'{i}:{u:3.0f}' + (f'(^{pk:.0f})' if r['n'] > 1 else '')
                                      for i, (u, pk) in enumerate(zip(r['cores'], r['peak']))))
    lines.append(f'{"cores":>5} {"peak":>5}  {"pid":>6}  {"where":<5} {"process":<26} {"busiest thread":<22} what')
    rows = []
    for pid, p in r['procs'].items():
        m = meta(pid)
        if m:
            rows.append((p['cores'], pid, p, m))
    for c, pid, p, m in sorted(rows, key=lambda x: -x[0])[:top]:
        if c < 0.02:
            break
        where = 'cont' if m['container'] else 'host'
        tn, tc, _ = p['top']
        lines.append(f'{c:5.2f} {p["peak"]:5.2f}  {pid:>6}  {where:<5} {m["label"][:26]:<26} '
                     f'{(tn[:14] + f" {tc:.2f}"):<22} {m["role"]}' + ('' if m['stack'] else '  <-- not part of the stack'))
    v = verdict(r)
    lines.append('VERDICT: ' + ('no bottleneck found' if not v else ''))
    lines += ['  - ' + x for x in v]
    return '\n'.join(lines)


def to_json(r):
    procs = []
    for pid, p in r['procs'].items():
        m = meta(pid)
        if m and p['cores'] >= 0.02:
            procs.append({'pid': pid, 'label': m['label'], 'role': m['role'], 'container': m['container'],
                          'stack': m['stack'], 'cores': round(p['cores'], 3), 'peak': round(p['peak'], 3),
                          'top_thread': p['top'][0], 'top_thread_cores': round(p['top'][1], 3)})
    procs.sort(key=lambda x: -x['cores'])
    return {'time': time.strftime('%Y-%m-%dT%H:%M:%S'), 'ncpu': NCPU, 'cpu_pct': round(sum(r['cores']) / NCPU, 1),
            'core_pct': [round(x, 1) for x in r['cores']], 'core_peak': [round(x, 1) for x in r['peak']],
            'run_queue': round(r['run'], 1), 'misc': r['misc'], 'procs': procs[:15], 'verdict': verdict(r)}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--once', type=float, metavar='SECS', help='sample for SECS, print one report, exit')
    ap.add_argument('--interval', type=float, default=1.0, help='sampling step in seconds (default 1)')
    ap.add_argument('--refresh', type=float, default=2.0, help='live view window in seconds (default 2)')
    ap.add_argument('--top', type=int, default=10)
    ap.add_argument('--json', action='store_true', help='with --once: print JSON')
    ap.add_argument('--log', metavar='FILE', help='append one JSON line per refresh')
    a = ap.parse_args()
    if not os.path.isdir('/proc/1'):
        sys.exit('needs Linux /proc: run it on the drone')

    prev = {'t': time.time(), 'cpu': read_cpu(), 'thr': read_threads()}
    if a.once:
        samples = []
        end = time.time() + a.once
        while time.time() < end:
            prev, s = sample(prev, a.interval)
            samples.append(s)
        r = merge(samples)
        print(json.dumps(to_json(r), indent=1) if a.json else render(r, a.top))
        return
    window, tty = [], sys.stdout.isatty()
    try:
        while True:
            prev, s = sample(prev, a.interval)
            window.append(s)
            if sum(x['dt'] for x in window) >= a.refresh:
                r = merge(window)
                window = []
                if a.log:
                    with open(a.log, 'a') as f:
                        f.write(json.dumps(to_json(r)) + '\n')
                print(('\033[H\033[J' if tty else '\n') + render(r, a.top, time.strftime('%H:%M:%S  ')), flush=True)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
