"""Fly many seeds in parallel and score them: success, time, clearance.

    python3 -m localsim.batch --seeds 1-40 --severity 0.7 --jobs 24 --out localsim/out/batch/base
    python3 -m localsim.batch --seeds 1-40 --out localsim/out/batch/try --tune radius=0.48
    python3 -m localsim.batch --summarise localsim/out/batch/base     # re-score finished runs

A run succeeds if it finishes (all checkpoints, back home) without a crash.
Clearance (``localsim.clearance``) is from the body's surface; the target is a
quarter of the tunnel radius. The window is reported apart: it is 0.80-0.95 m
wide for a 0.58 m rotor span, so even centred the gap there is 0.11-0.19 m.
"""
import argparse
import json
import math
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

WINDOW_GOALS = ('window', 'window_back')
WINDOW_REACH = 0.5      # centre this close to the window wall's line: passing the window


def seeds(text):
    out = []
    for part in text.split(','):
        a, _, b = part.partition('-')
        out += list(range(int(a), int(b or a) + 1))
    return out


def fly(seed, args, extra):
    run = Path(args.out) / f's{seed}'
    cmd = [sys.executable, '-m', 'localsim.goalrun', '--seed', str(seed), '--severity', str(args.severity),
           '--profile', args.profile, '--duration', str(args.duration), '--no-viewer', '--no-camera',
           '--out', str(run), *extra]
    with open(f'{run}.log', 'w') as log:
        subprocess.run(cmd, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
    return run


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
        if c < best[key][0]:
            best[key] = (c, {'t': f['t'], 'goal': f.get('goal'), 'state': f.get('state'),
                             'what': 'net' if net < body else 'wall', 'p': [round(v, 2) for v in p]})
    ok = report['finish_s'] is not None and not report['crashed']
    return {'run': run.name, 'ok': ok, 'finish_s': report['finish_s'], 'total_s': report['total_s'],
            'checkpoints': report['checkpoints'], 'touches': report['touches'], 'crashed': report['crashed'],
            'final': report['final'],
            'clear_m': round(best['course'][0], 3), 'clear_at': best['course'][1],
            'window_clear_m': round(best['window'][0], 3), 'under_target_s': round(under * 0.1, 1)}


def summarise(rows):
    from localsim.clearance import TARGET_M
    done = [r for r in rows if 'error' not in r]
    ok = [r for r in done if r['ok']]
    times = [r['finish_s'] for r in ok]
    clear_ok = [r for r in done if r['clear_m'] >= TARGET_M]
    out = {'runs': len(rows), 'success': len(ok), 'success_rate': round(len(ok) / max(1, len(rows)), 3),
           'mean_finish_s': round(sum(times) / len(times), 1) if times else None,
           'max_finish_s': max(times) if times else None,
           'touch_runs': sum(r['touches'] > 0 for r in done),
           'clearance_target_m': round(TARGET_M, 3),
           'runs_meeting_clearance': len(clear_ok),
           'worst_clear_m': min((r['clear_m'] for r in done), default=None),
           'worst_window_clear_m': min((r['window_clear_m'] for r in done), default=None),
           'failures': [{k: r.get(k) for k in ('run', 'final', 'crashed', 'checkpoints', 'error')}
                        for r in rows if not r.get('ok')]}
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--seeds', default='1-24')
    parser.add_argument('--severity', type=float, default=0.7)
    parser.add_argument('--profile', default='safe')
    parser.add_argument('--duration', type=float, default=200.)
    parser.add_argument('--jobs', type=int, default=24)
    parser.add_argument('--out', default=str(ROOT / 'localsim/out/batch/run'))
    parser.add_argument('--summarise', default=None, metavar='DIR', help='only score the runs already in DIR')
    args, extra = parser.parse_known_args(argv)
    if args.summarise:
        runs = sorted(Path(args.summarise).glob('s*/'), key=lambda p: int(p.name[1:]))
    else:
        Path(args.out).mkdir(parents=True, exist_ok=True)
        with ThreadPoolExecutor(args.jobs) as pool:
            runs = list(pool.map(lambda s: fly(s, args, extra), seeds(args.seeds)))
    with ProcessPoolExecutor(min(args.jobs, len(runs) or 1)) as pool:
        rows = list(pool.map(score, runs))
    base = Path(args.summarise or args.out)
    (base / 'scores.jsonl').write_text(''.join(json.dumps(r, default=str) + '\n' for r in rows))
    summary = summarise(rows)
    (base / 'summary.json').write_text(json.dumps(summary, indent=1, default=str))
    for r in rows:
        print(json.dumps({k: r.get(k) for k in ('run', 'ok', 'finish_s', 'touches', 'clear_m', 'window_clear_m',
                                                'clear_at', 'error')}, default=str))
    print(json.dumps(summary, indent=1, default=str))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
