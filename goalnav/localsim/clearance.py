"""How close the drone's body came to anything solid, from a finished run.

    python3 -m localsim.clearance localsim/out/batch/s1 [more run dirs...]

Clearance is measured from the Sky Beta's surface, not its centre, with the
same three-layer shape the contact judge uses (``goalrun.drone_contacts``):
core 0.13 m, arms 0.20 m and props ``BODY_RADIUS`` (0.29 m), each over its own
height band. A layer's clearance is the horizontal distance from its rim to
the nearest contact voxel in its band; the body's is the smallest of the
three, and the nets count too (the props' rim to each net plane). Negative
means the body overlaps something (a touch).

The target is a quarter of the tunnel's radius (its half width,
``messy_course.TUBE['hw']``, 0.74 m): ``TARGET_M`` = 0.18 m. Samples are the
trace's 10 Hz true positions while airborne (feet over ``AIRBORNE_M``): the
launch spot and the pad are given by the course, not chosen by the navigator.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from localsim.messy_course import BODY_RADIUS, TUBE     # noqa: E402

TARGET_M = TUBE['hw'] / 4
AIRBORNE_M = 0.15            # feet this far over the floor (origin is 7 cm over the feet)
FEET_BELOW_ORIGIN = 0.07
SEARCH_M = 0.6               # look this far past each rim; farther counts as SEARCH_M
# (z offset of the band's centre from the origin, rim radius, band half height): goalrun.drone_contacts.
LAYERS = ((-0.01, 0.13, 0.06), (-0.014, 0.20, 0.018), (0.013, BODY_RADIUS, 0.009))


def body_clearance(world, p, search=SEARCH_M):
    """Smallest gap (m) from the body's rim to a contact voxel; ``search`` if none is near."""
    best = search
    for dz, radius, half in LAYERS:
        centre = np.array([p[0], p[1], p[2] + dz])
        reach = radius + search
        low = np.floor((centre - [reach, reach, half] - world.origin) / world.voxel).astype(int)
        high = np.floor((centre + [reach, reach, half] - world.origin) / world.voxel).astype(int) + 1
        low, high = np.maximum(low, 0), np.minimum(high, world.shape)
        if np.any(high <= low):
            continue
        block = world.contact[low[0]:high[0], low[1]:high[1], low[2]:high[2]]
        ix, iy = np.nonzero(block.any(axis=2))
        if not len(ix):
            continue
        dx = world.origin[0] + (low[0] + ix + .5) * world.voxel - centre[0]
        dy = world.origin[1] + (low[1] + iy + .5) * world.voxel - centre[1]
        best = min(best, float(np.hypot(dx, dy).min()) - radius)
    return best


def net_clearance(nets, p, radius=BODY_RADIUS):
    """The props' rim to the nearest net plane (as goalrun's net-touch check)."""
    x, y = p[0], p[1]
    return float(min(x + nets['west'], nets['east'] - x, y + nets['south'], nets['north'] - y) - radius)


def analyse(run_dir, world=None):
    """Clearance over one run: the minimum, where it was, and the time spent under the target."""
    run_dir = Path(run_dir)
    trace = json.loads((run_dir / 'trace.json').read_text())
    report = json.loads((run_dir / 'report.json').read_text())
    if world is None:
        from localsim.messy_world import MessyWorld
        meta = json.loads((run_dir / 'course/course.json').read_text())
        world = MessyWorld(run_dir / 'course/course.obj', meta, cache_dir=ROOT / 'localsim/.cache')
    nets = trace['nets']
    worst = {'clear_m': math.inf}
    under = 0
    samples = 0
    for f in trace['frames']:
        p = f['p']
        if p[2] - FEET_BELOW_ORIGIN < AIRBORNE_M:
            continue
        samples += 1
        body, net = body_clearance(world, p), net_clearance(nets, p)
        c = min(body, net)
        under += c < TARGET_M
        if c < worst['clear_m']:
            worst = {'clear_m': round(c, 3), 't': f['t'], 'p': p, 'what': 'net' if net < body else 'wall',
                     'goal': f.get('goal'), 'state': f.get('state')}
    return {'run': run_dir.name, 'finish_s': report['finish_s'], 'crashed': report['crashed'],
            'checkpoints': report['checkpoints'], 'touches': report['touches'], 'final': report['final'][0],
            'min': worst, 'under_target_s': round(under * 0.1, 1), 'samples': samples}


def main(argv=None):
    rows = [analyse(d) for d in (argv or sys.argv[1:])]
    for r in rows:
        print(json.dumps(r, default=str))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
