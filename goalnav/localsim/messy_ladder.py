"""The stepladder the organizers added to the course (photos, 2026-11-07).

A 6 ft fiberglass stepladder (Werner 6000 series, 6006 type: blue rails,
yellow top, 5 aluminium steps; spec: 22 in base, 41 1/8 in open spread) stands
open in the landing area, between the launch corridor's barrier panel and the
mat, its rear legs toward the west net and its steps facing east-north-east,
toward A1.

Placement and size were measured from ``photos/Ladder_*.jpg`` (agents/logs/
2026-11-07-ladder.md): the three cameras solved against the scan's barrier
panel 1, the drone on the launch mark and the net's 5 ft rail, the feet cast
onto the floor, the four feet fitted as one rectangle. Footprint centre
(-2.31, -0.62), spread axis 16 deg, top 1.70-1.71 m; about +-0.1 m and
+-5 deg, more if the barrier has moved since the scan. The mat in the same
photos is centred near (-1.94, -1.47), about 0.6 m square.

``ladder(centre, yaw)`` returns the mesh groups; ``place`` draws a pose near
the measured one that stands clear of everything already built.
"""
import math

import numpy as np

from localsim import messy_mesh as mm

CENTRE = (-2.31, -0.62)          # footprint centre, course frame (m)
YAW_DEG = 16.                    # rear feet -> front (step) feet, CCW from +x
TOP_Z = 1.71                     # top of the top cap (measured 1.70-1.71)
SPREAD = 0.96                    # rear to front foot centres (measured 0.94, spec ~0.98)
BASE_WIDTH = 0.50                # foot centres across (measured 0.48, spec ~0.50)
TOP_WIDTH = 0.36                 # rail centres across under the top cap
TOP_FRONT_U, TOP_REAR_U = 0.12, -0.11     # where the front and rear rails meet the top, from the centre
CAP = (0.22, 0.38, 0.06)         # top cap: depth (along), width (across), thickness
FRONT_RAIL = (0.08, 0.03)        # 3 1/8 in channel: along the spread, across
REAR_RAIL = (0.045, 0.025)
STEP_Z = (0.29, 0.57, 0.85, 1.13, 1.41)   # 5 steps, ~11 in apart
STEP_DEPTH, STEP_T = 0.076, 0.02
REAR_BRACE_Z = (0.27, 0.95)
SPREADER_Z = 1.02
JITTER_XY, JITTER_YAW = 0.10, 6.  # placement spread at severity 1 (before messy_course.JITTER)


def _frame(yaw):
    a = math.radians(yaw)
    return np.array([math.cos(a), math.sin(a)]), np.array([-math.sin(a), math.cos(a)])


def _stick(p0, p1, along_dir, along, across):
    """A rail of rectangular section from p0 to p1: ``along`` metres in the
    horizontal direction ``along_dir`` (projected off the axis), ``across`` in the other."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    axis = (p1 - p0) / np.linalg.norm(p1 - p0)
    a = np.array([along_dir[0], along_dir[1], 0.])
    a = a - axis * (a @ axis)
    a /= np.linalg.norm(a)
    b = np.cross(axis, a)
    ring = [a * along / 2 + b * across / 2, -a * along / 2 + b * across / 2,
            -a * along / 2 - b * across / 2, a * along / 2 - b * across / 2]
    vertices = np.array([p0 + r for r in ring] + [p1 + r for r in ring])
    faces = [(0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6)]
    for k in range(4):
        n = (k + 1) % 4
        faces += [(k, n + 4, n), (k, k + 4, n + 4)]
    return vertices, np.array(faces)


def ladder(centre=CENTRE, yaw=YAW_DEG):
    """{group: (vertices, triangles)}: 'Ladder rails' (fiberglass), 'Ladder top', 'Ladder metal'."""
    u, v = _frame(yaw)
    c = np.asarray(centre, float)
    at = lambda du, dv, z: np.array([*(c + u * du + v * dv), z])
    rail_top = TOP_Z - CAP[2]
    rails, metal = [], []
    front = {s: (at(SPREAD / 2, s * BASE_WIDTH / 2, 0.), at(TOP_FRONT_U, s * TOP_WIDTH / 2, rail_top)) for s in (-1, 1)}
    rear = {s: (at(-SPREAD / 2, s * BASE_WIDTH / 2, 0.), at(TOP_REAR_U, s * TOP_WIDTH / 2, rail_top)) for s in (-1, 1)}
    for s in (-1, 1):
        rails.append(_stick(*front[s], u, *FRONT_RAIL))
        rails.append(_stick(*rear[s], u, *REAR_RAIL))

    def on(rail, z):
        a, b = rail
        return a + (b - a) * (z / b[2])
    for z in STEP_Z:                       # flat aluminium treads between the front rails
        a, b = on(front[-1], z), on(front[1], z)
        metal.append(_stick(a, b, u, STEP_DEPTH, STEP_T))
    for z in REAR_BRACE_Z:                 # rear rungs, the bottom one with a diagonal down to each rail
        a, b = on(rear[-1], z), on(rear[1], z)
        metal.append(_stick(a, b, u, 0.03, 0.015))
    a, b = on(rear[-1], REAR_BRACE_Z[0]), on(rear[1], REAR_BRACE_Z[0])
    mid = (a + b) / 2
    for s in (-1, 1):
        metal.append(_stick(mid, on(rear[s], 0.08), u, 0.02, 0.015))
    for s in (-1, 1):                      # the spreader arms, one each side
        metal.append(_stick(on(rear[s], SPREADER_Z), on(front[s], SPREADER_Z), v, 0.02, 0.025))
    cap_c = (TOP_FRONT_U + TOP_REAR_U) / 2
    top = mm.box(at(cap_c - CAP[0] / 2, 0., 0.)[:2], at(cap_c + CAP[0] / 2, 0., 0.)[:2], CAP[1], rail_top, TOP_Z)
    return {'Ladder rails': mm.merge(rails), 'Ladder metal': mm.merge(metal), 'Ladder top': top}


def footprint(centre, yaw, margin=0.):
    """The four foot centres pushed out by ``margin`` (for clearance checks)."""
    u, v = _frame(yaw)
    c = np.asarray(centre, float)
    return [c + u * su * (SPREAD / 2 + margin) + v * sv * (BASE_WIDTH / 2 + margin) for su, sv in
            ((-1, -1), (1, -1), (1, 1), (-1, 1))]


STAND_FEET_Z = 0.10              # the stands' battens and pavers are lower than this


def place(rng, spread, groups, raster, tries=40, slide=0.20):
    """A pose (centre, yaw_deg, clear) near the measured one that touches
    nothing in ``groups``. ``spread`` scales the jitter (severity x JITTER);
    ``raster(groups, z0, z1)`` -> (grid, res, extent) as messy_course._band_raster.

    First a jittered pose clear of everything; failing that (the stands'
    floor battens are drawn generically and the real ladder stands right
    beside A1's), one clear above the battens; each time the measured pose
    slid up to ``slide`` m before giving up. ``clear`` is False only if even
    that failed and the measured pose is returned as it is."""
    solid = {k: g for k, g in groups.items() if not k.startswith(('Floor', 'Landing'))}
    full = raster(solid, 0.0, TOP_Z + 0.05)[0]
    above = raster(solid, STAND_FEET_Z, TOP_Z + 0.05)[0]
    jittered = []
    for i in range(tries):
        shrink = 1. - i / tries
        jittered.append((np.asarray(CENTRE) + rng.uniform(-1, 1, 2) * JITTER_XY * spread * shrink,
                         YAW_DEG + float(rng.uniform(-1, 1)) * JITTER_YAW * spread * shrink))
    step = 0.05
    offsets = [np.array([dx, dy]) for dx in np.arange(-slide, slide + 1e-9, step)
               for dy in np.arange(-slide, slide + 1e-9, step)]
    slid = [(np.asarray(CENTRE) + d, YAW_DEG) for d in sorted(offsets, key=np.linalg.norm)
            if np.linalg.norm(d) <= slide + 1e-9]
    for grid, z0 in ((full, 0.0), (above, STAND_FEET_Z)):
        for centre, yaw in jittered + slid:
            if not (raster(ladder(centre, yaw), z0, TOP_Z + 0.05)[0] & grid).any():
                return (float(centre[0]), float(centre[1])), float(yaw), True
    return tuple(CENTRE), YAW_DEG, False
