"""Build a self-contained animated HTML replay of a local-sim run.

    python3 -m localsim.replay localsim/out            # writes localsim/out/replay.html

Open the file in any browser: top-down map, plan, checkpoint lines, the drone
(footprint circle and nose) flying the recorded trace, trail coloured by speed,
and a timeline with mission state.
"""
import argparse
from collections import defaultdict
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TEMPLATE = Path(__file__).with_name('replay.html')


def _runs(grid, coarsen=1):
    rows = defaultdict(set)
    for cx, cy in grid['occupied']:
        rows[cy // coarsen].add(cx // coarsen)
    rows = {cy: list(xs) for cy, xs in rows.items()}
    runs = []
    for cy, xs in rows.items():
        xs.sort()
        start = prev = xs[0]
        for cx in xs[1:] + [None]:
            if cx is not None and cx == prev + 1:
                prev = cx
                continue
            runs.append((start, cy, prev - start + 1))
            if cx is not None:
                start = prev = cx
    return runs


def build(out_dir, course_path=ROOT / 'config/course_v3.json', every=1, target=None, compact=False):
    """compact: coarser map and plan for a small embeddable page."""
    out_dir = Path(out_dir)
    course = json.loads(Path(course_path).read_text())
    report = json.loads((out_dir / 'report.json').read_text())
    trace = json.loads((out_dir / 'trace.json').read_text())
    plan = json.loads((out_dir / 'plan.json').read_text()) if (out_dir / 'plan.json').exists() else []
    lines = json.loads((out_dir / 'lines.json').read_text()) if (out_dir / 'lines.json').exists() else []
    grid = course['grid']
    data = {
        'grid': {'res': grid['resolution'] * (2 if compact else 1), 'origin': grid['origin'],
                 'runs': _runs(grid, 2 if compact else 1)},
        'pad': course['landing'], 'launch': course['launch'], 'radius': course['radius_m'],
        'plan': [[round(x, 2), round(y, 2)] for x, y in plan[::2 if compact else 1]],
        'lines': lines,
        # t, x, y, z, speed, yaw, state, contact
        'trace': [[round(r[0], 2), round(r[1], 2), round(r[2], 2), round(r[3], 2), round(r[4], 2),
                   round(r[5], 2), r[6], r[7]] for r in trace[::every]],
        'referee': report['referee'], 'final': [report['final_state'], report['final_reason']],
    }
    html = TEMPLATE.read_text().replace('/*DATA*/null', json.dumps(data, separators=(',', ':')))
    target = Path(target or out_dir / 'replay.html')
    target.write_text(html)
    return target


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('out', nargs='?', default=str(ROOT / 'localsim/out'))
    parser.add_argument('--every', type=int, default=1, help='keep every Nth trace sample')
    parser.add_argument('--target', help='output file (default OUT/replay.html)')
    parser.add_argument('--compact', action='store_true', help='coarser map and plan, smaller file')
    args = parser.parse_args(argv)
    print(build(args.out, every=args.every, target=args.target, compact=args.compact))


if __name__ == '__main__':
    main()
