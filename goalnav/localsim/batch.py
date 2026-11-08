"""Fly many seeds in parallel and score them: success, time, clearance.

    python3 -m localsim.batch --seeds 1-40 --scenario drone --out localsim/out/batch/base
    python3 -m localsim.batch --seeds 1-40 --scenario drone --out localsim/out/batch/try --tune radius=0.48
    python3 -m localsim.batch --summarise localsim/out/batch/base     # re-score finished runs
    python3 -m localsim.batch --compare localsim/out/batch/base localsim/out/batch/try

Any other flag (``--camera rgbd``, ``--tune k=v``, ``--vehicle f=v``, ...) goes to
``localsim.goalrun`` unchanged.

``--scenario`` (``localsim/drone_measured.yaml``) sets the depth rate, its age and the
vehicle from numbers measured on skybeta-03; without it goalrun's own defaults apply,
which are kinder than the drone.

A run succeeds if it finishes (all checkpoints, back home) without a crash. The
success rate comes with its 95 % Wilson lower bound: 16/16 only shows >= 81 %, and
showing 99 % takes 381 runs without a failure.

**Deployable** means the drone would fly what was simulated: the drone starts
``ros2 launch drone_ref goalnav.launch.py`` with no arguments
(``skybetav1/container_scripts/run_competitor.sh``), so only the launch file's default
profile, without ``--tune`` or ``--set``, is what flies. A winning ``--tune`` must be
written into ``config/goalnav_profiles.yaml`` and re-run before it counts.

Clearance (``localsim.clearance``) is from the body's surface; the target is a
quarter of the tunnel radius. The window is reported apart: it is 0.80-0.95 m
wide for a 0.58 m rotor span, so even centred the gap there is 0.11-0.19 m.
"""
import argparse
import hashlib
import json
import math
import re
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

WINDOW_GOALS = ('window', 'window_back')
WINDOW_REACH = 0.5      # centre this close to the window wall's line: passing the window
SCENARIOS = ROOT / 'localsim/drone_measured.yaml'
LAUNCH_FILE = ROOT / 'launch/goalnav.launch.py'
PROFILES = ROOT / 'config/goalnav_profiles.yaml'
Z95 = 1.959964


def seeds(text):
    out = []
    for part in text.split(','):
        a, _, b = part.partition('-')
        out += list(range(int(a), int(b or a) + 1))
    return out


# -- what the sim assumes, and whether it is what flies ----------------------------

def load_scenarios(path=SCENARIOS):
    return yaml.safe_load(Path(path).read_text())


def scenario_flags(name, path=SCENARIOS):
    """goalrun flags for one scenario of drone_measured.yaml (KeyError if there is none)."""
    s = load_scenarios(path)[name]
    flags = []
    if 'camera_hz' in s:
        flags += ['--camera-hz', str(s['camera_hz'])]
    if 'camera_latency' in s:
        flags += ['--camera-latency', str(s['camera_latency'])]
    for field, value in (s.get('vehicle') or {}).items():
        flags += ['--vehicle', f'{field}={value}']
    return flags


def launch_default_profile(path=LAUNCH_FILE):
    """The profile the drone flies: the launch file's default (it is started without arguments)."""
    text = Path(path).read_text()
    # The launch's argument table, ('profile', 'safe', ...), or the older {'profile': ('safe', ...)}.
    found = re.search(r"\(\s*'profile'\s*,\s*'([^']+)'", text) or re.search(r"'profile':\s*\('([^']+)'", text)
    if not found:
        raise ValueError(f'no profile default in {path}')
    return found.group(1)


def deployable(profile, extra):
    """True when the drone, as launched, would fly the configuration simulated."""
    return profile == launch_default_profile() and not any(a in ('--tune', '--set') or
                                                           a.startswith(('--tune=', '--set=')) for a in extra)


def provenance(args, extra):
    """What was simulated, so a result can be traced to code and settings."""
    def git(*cmd):
        try:
            return subprocess.run(['git', *cmd], cwd=ROOT, capture_output=True, text=True, timeout=10).stdout.strip()
        except (OSError, subprocess.SubprocessError):
            return None
    return {'commit': git('rev-parse', '--short', 'HEAD'),
            'dirty': bool(git('status', '--porcelain', '--', '.')),
            'profiles_sha': hashlib.sha256(PROFILES.read_bytes()).hexdigest()[:12],
            'profile': args.profile, 'scenario': args.scenario,
            'scenario_flags': scenario_flags(args.scenario) if args.scenario else [],
            'extra': extra, 'severity': args.severity, 'seeds': args.seeds,
            'deployable': deployable(args.profile, extra),
            'camera': extra[extra.index('--camera') + 1] if '--camera' in extra else 'rgbstereo'}


# -- running and scoring -----------------------------------------------------------

def fly(seed, args, extra):
    run = Path(args.out) / f's{seed}'
    cmd = [sys.executable, '-m', 'localsim.goalrun', '--seed', str(seed), '--severity', str(args.severity),
           '--profile', args.profile, '--duration', str(args.duration), '--no-viewer', '--no-camera',
           '--out', str(run), *(scenario_flags(args.scenario) if args.scenario else []), *extra]
    with open(f'{run}.log', 'w') as log:
        subprocess.run(cmd, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
    return run


def missing(runs):
    """Runs the simulator did not finish (no report.json): to fly again, not to count."""
    return [Path(r) for r in runs if not (Path(r) / 'report.json').exists()]


def _segment_distance(p, a, b):
    ax, ay = b[0] - a[0], b[1] - a[1]
    t = max(0., min(1., ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / (ax * ax + ay * ay)))
    return math.hypot(p[0] - a[0] - t * ax, p[1] - a[1] - t * ay)


def score(run):
    """One run's result and clearance, split at the window."""
    from localsim import clearance
    from localsim.messy_world import MessyWorld
    run = Path(run)
    if not (run / 'report.json').exists():
        return {'run': run.name, 'error': 'no report'}
    meta = json.loads((run / 'course/course.json').read_text())
    world = MessyWorld(run / 'course/course.obj', meta, cache_dir=ROOT / 'localsim/.cache')
    trace = json.loads((run / 'trace.json').read_text())
    report = json.loads((run / 'report.json').read_text())
    window_wall = [w for w in meta['walls'] if w['name'].startswith('C2a')]
    pieces = window_wall[0]['pieces'] if window_wall else []
    best = {'course': (math.inf, None), 'window': (math.inf, None)}
    homeward = math.inf          # The return leg alone: the referee does not count touches there.
    under = 0
    for f in trace['frames']:
        p = f['p']
        if p[2] - clearance.FEET_BELOW_ORIGIN < clearance.AIRBORNE_M:
            continue
        body, net = clearance.body_clearance(world, p), clearance.net_clearance(trace['nets'], p)
        c = min(body, net)
        at_window = f.get('goal') in WINDOW_GOALS and any(
            _segment_distance(p, a, b) < WINDOW_REACH for a, b in pieces)
        key = 'window' if at_window else 'course'
        under += key == 'course' and c < clearance.TARGET_M
        if str(f.get('state', '')).startswith('RETURN'):
            homeward = min(homeward, c)
        if c < best[key][0]:
            best[key] = (c, {'t': f['t'], 'goal': f.get('goal'), 'state': f.get('state'),
                             'what': 'net' if net < body else 'wall', 'p': [round(v, 2) for v in p]})
    ok = report['finish_s'] is not None and not report['crashed']
    back = report.get('return') or {}
    home_s = back.get('done_s') if ok and not back.get('failed') else None
    return {'run': run.name, 'ok': ok, 'finish_s': report['finish_s'], 'total_s': report['total_s'],
            'home_s': home_s, 'return_failed': back.get('failed'),
            'return_clear_m': round(homeward, 3) if homeward < math.inf else None,
            'checkpoints': report['checkpoints'], 'touches': report['touches'], 'crashed': report['crashed'],
            'final': report['final'],
            'clear_m': round(best['course'][0], 3), 'clear_at': best['course'][1],
            'window_clear_m': round(best['window'][0], 3), 'under_target_s': round(under * 0.1, 1)}


# -- statistics --------------------------------------------------------------------

def wilson_low(k, n, z=Z95):
    """95 % Wilson score lower bound on a pass rate of k in n (0 for no runs)."""
    if n == 0:
        return 0.
    p = k / n
    centre = p + z * z / (2 * n)
    spread = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return max(0., (centre - spread) / (1 + z * z / n))


def runs_needed(target, z=Z95):
    """Runs without a failure for the Wilson lower bound to reach ``target``."""
    return math.ceil(target * z * z / (1 - target))


def summarise(rows):
    from localsim.clearance import TARGET_M
    done = [r for r in rows if 'error' not in r]       # flown; an errored run is no pass and no failure
    ok = [r for r in done if r['ok']]
    times = [r['finish_s'] for r in ok]
    home = [r['home_s'] for r in ok if r.get('home_s') is not None]
    return {'runs': len(done), 'sim_errors': [r['run'] for r in rows if 'error' in r],
            'success': len(ok), 'success_rate': round(len(ok) / max(1, len(done)), 3),
            'success_ci95_low': round(wilson_low(len(ok), len(done)), 3),
            'runs_for_99pct': runs_needed(0.99),
            'mean_finish_s': round(sum(times) / len(times), 1) if times else None,     # on the pad (CP5)
            'max_finish_s': max(times) if times else None,
            'home': len(home),                                                       # and back on the launch mark
            'mean_home_s': round(sum(home) / len(home), 1) if home else None,
            'max_home_s': max(home) if home else None,
            'touch_runs': sum(r['touches'] > 0 for r in done),                       # up to the pad only (referee)
            'touches': sum(r['touches'] for r in done),
            'return_scrape_runs': [r['run'] for r in done if r.get('return_clear_m') is not None
                                   and r['return_clear_m'] <= 0],
            'clearance_target_m': round(TARGET_M, 3),
            'runs_meeting_clearance': sum(r['clear_m'] >= TARGET_M for r in done),
            'worst_clear_m': min((r['clear_m'] for r in done), default=None),
            'worst_window_clear_m': min((r['window_clear_m'] for r in done), default=None),
            'failures': [{k: r.get(k) for k in ('run', 'final', 'crashed', 'checkpoints')}
                         for r in done if not r['ok']]}


def _rows(directory):
    return {r['run']: r for r in (json.loads(line) for line in
                                  (Path(directory) / 'scores.jsonl').read_text().splitlines() if line.strip())}


def compare(a_dir, b_dir):
    """B against A on the seeds both flew: which B fixes or breaks, and the change in
    finish time and minimum clearance over the seeds both finished."""
    a, b = _rows(a_dir), _rows(b_dir)
    shared = sorted((s for s in set(a) & set(b) if 'error' not in a[s] and 'error' not in b[s]),
                    key=lambda s: int(s[1:]))
    both = [s for s in shared if a[s].get('ok') and b[s].get('ok')]
    mean = (lambda xs: round(sum(xs) / len(xs), 4) if xs else None)
    return {'shared': len(shared),
            'a_success': sum(bool(a[s].get('ok')) for s in shared),
            'b_success': sum(bool(b[s].get('ok')) for s in shared),
            'b_fixes': [s for s in shared if b[s].get('ok') and not a[s].get('ok')],
            'b_breaks': [s for s in shared if a[s].get('ok') and not b[s].get('ok')],
            'mean_dt_s': mean([b[s]['finish_s'] - a[s]['finish_s'] for s in both]),
            'mean_dclear_m': mean([b[s]['clear_m'] - a[s]['clear_m'] for s in both]),
            'b_closer_than_a': [s for s in both if b[s]['clear_m'] < a[s]['clear_m'] - 0.01],
            'touches': [sum(a[s].get('touches', 0) for s in shared), sum(b[s].get('touches', 0) for s in shared)]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--seeds', default='1-24')
    parser.add_argument('--severity', type=float, default=0.7)
    parser.add_argument('--profile', default='safe')
    parser.add_argument('--scenario', default=None, help='drone-measured numbers, localsim/drone_measured.yaml')
    parser.add_argument('--duration', type=float, default=200.)
    parser.add_argument('--jobs', type=int, default=16, help='runs at once; each new course voxelises at ~1 GB: 48 at once ran out of 32 GB')
    parser.add_argument('--out', default=str(ROOT / 'localsim/out/batch/run'))
    parser.add_argument('--summarise', default=None, metavar='DIR', help='only score the runs already in DIR')
    parser.add_argument('--compare', nargs=2, default=None, metavar=('A', 'B'), help='B against A, seed by seed')
    args, extra = parser.parse_known_args(argv)
    if args.compare:
        print(json.dumps(compare(*args.compare), indent=1))
        return 0
    if args.scenario:
        scenario_flags(args.scenario)          # An unknown name stops here, before any run.
    if args.summarise:
        runs = sorted(Path(args.summarise).glob('s*/'), key=lambda p: int(p.name[1:]))
    else:
        Path(args.out).mkdir(parents=True, exist_ok=True)
        (Path(args.out) / 'provenance.json').write_text(json.dumps(provenance(args, extra), indent=1))
        with ThreadPoolExecutor(args.jobs) as pool:
            runs = list(pool.map(lambda s: fly(s, args, extra), seeds(args.seeds)))
        again = missing(runs)
        if again:      # Usually memory: every new course voxelises at once. Retry a few at a time.
            print(f'retrying {len(again)} runs the simulator did not finish: {[p.name for p in again]}')
            with ThreadPoolExecutor(max(1, args.jobs // 4)) as pool:
                list(pool.map(lambda p: fly(int(p.name[1:]), args, extra), again))
    with ProcessPoolExecutor(min(args.jobs, len(runs) or 1)) as pool:
        rows = list(pool.map(score, runs))
    base = Path(args.summarise or args.out)
    (base / 'scores.jsonl').write_text(''.join(json.dumps(r, default=str) + '\n' for r in rows))
    summary = summarise(rows)
    if (base / 'provenance.json').exists():
        summary['provenance'] = json.loads((base / 'provenance.json').read_text())
    (base / 'summary.json').write_text(json.dumps(summary, indent=1, default=str))
    for r in rows:
        print(json.dumps({k: r.get(k) for k in ('run', 'ok', 'finish_s', 'touches', 'clear_m', 'window_clear_m',
                                                'clear_at', 'error')}, default=str))
    print(json.dumps(summary, indent=1, default=str))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
