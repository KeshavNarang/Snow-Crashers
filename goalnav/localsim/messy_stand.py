"""How the real course pieces are built, from photos of the event net.

Walls are freestanding plywood panels on wooden stands:

- a ~2 cm plywood sheet, its bottom edge 18-32 cm above the floor (a gap
  the depth camera sees through and the floor line runs under);
- a printed vinyl tarp on one side only (grey sci-fi panels or red rock),
  stretched over the face with shallow bulges, sometimes wrapping an end;
- on the bare side, at each end (and mid-span on long walls): an upright
  post and a diagonal strut from the top of the panel down to the floor,
  ~0.29 m out at its foot (a triangle in section: ~0.1 m out at 1.15 m
  cruise, ~0.16 m at 0.75 m); a batten foot across the floor on both sides,
  held down with paver bricks;
- the vinyl wraps around each end edge onto the back as far as the strut,
  and no further (no loose flaps).

The tunnel is a wire-mesh hoop tube (see-through to stereo, solid to touch)
with metal hoops, part-covered by a red rock-print tarp draped over its top
and hanging off its mouth.
"""
import math

import numpy as np

from localsim import messy_mesh as mm

PANEL_T = 0.018
POST_W = 0.04
FOOT_BACK, FOOT_FRONT = 0.62, 0.32
PAVER = (0.20, 0.10, 0.07)


def _support_positions(length, hole=None):
    ends = [0.06, length - 0.06]
    if hole is None and length > 2.2:          # Only the long barrier spans need a middle stand.
        ends.insert(1, length / 2)
    return ends


def _wrap(a, along, normal, side, u_end, inward, low, top):
    """The vinyl round one end edge: from the tarped face, round the edge,
    along the back to the strut at ``u_end + inward * 0.06``."""
    t = PANEL_T / 2 + 0.012
    stations = [(u_end, side * t), (u_end - inward * 0.012, side * t * 0.5), (u_end - inward * 0.012, -side * t * 0.5),
                (u_end, -side * t), (u_end + inward * 0.06, -side * t)]
    xy = [a + along * u + normal * n for u, n in stations]
    vertices = np.array([(*p, z) for z in (low, top) for p in xy])
    k = len(xy)
    triangles = [(i, i + 1, i + 1 + k) for i in range(k - 1)] + [(i, i + 1 + k, i + k) for i in range(k - 1)]
    return vertices, np.array(triangles)


def stand(name, pieces, height, gap, tarp_side, lean, rng, hole=None, flaps=(0., 0.), strut=(1.05, 0.29)):
    """Groups for one wall: panel, tarp, frame (posts, struts, feet), pavers."""
    panel, tarp, frame, pavers = [], [], [], []
    for k, (a, b) in enumerate(pieces):
        a, b = np.asarray(a, float), np.asarray(b, float)
        length = float(np.linalg.norm(b - a))
        along = (b - a) / length
        normal = np.array([-along[1], along[0]])
        if hole is not None:
            panel.append(mm.window_wall(a, b, PANEL_T, gap, height, *hole, lean=lean))
        else:
            panel.append(mm.box(a, b, PANEL_T, gap, height, lean))
        low = max(0.04, gap - rng.uniform(0., 0.12))
        tarp.append(mm.tarp(a, b, tarp_side, PANEL_T, low, height + 0.02, rng, hole=hole,
                            lean=lean, bulge=0.03, skirt=0.))
        for u_end, inward in ((0., 1.), (length, -1.)):         # Wrapped round to the struts, no further.
            tarp.append(_wrap(a, along, normal, tarp_side, u_end, inward, low, height + 0.02))
        bare = -tarp_side * normal
        for u in _support_positions(length, hole):
            base = a + along * u
            top = height - 0.05                            # The strut runs from the panel's top...
            out = strut[1] + rng.uniform(-0.01, 0.01)       # ...to the floor this far out (one build).
            post = base + bare * (PANEL_T / 2 + POST_W / 2)
            frame.append(mm.beam((*post, 0.), (*post, height - 0.05), POST_W))
            frame.append(mm.beam((*post, top), (*(base + bare * out), 0.02), POST_W * 0.8))
            foot_a, foot_b = base - bare * FOOT_FRONT, base + bare * FOOT_BACK
            frame.append(mm.beam((*foot_a, 0.012), (*foot_b, 0.012), 0.04, 0.022))
            for end, chance in ((foot_b - bare * 0.12, 0.85), (foot_a + bare * 0.1, 0.4)):
                if rng.random() < chance:
                    yaw = math.atan2(along[1], along[0]) + rng.normal(0., 0.25)
                    size = tuple(s * rng.uniform(0.9, 1.15) for s in PAVER)
                    pavers.append(mm.block(end + rng.normal(0., 0.02, 2), size, yaw))
    return {f'Panel {name}': mm.merge(panel), f'Tarp {name}': mm.merge(tarp),
            f'Frame {name}': mm.merge(frame), f'Paver {name}': mm.merge(pavers)}


def corner_rocks(place, tube, mouths, rng):
    """The tall rocks standing at each corner of the tunnel, just outside its
    mouths, flanking the opening."""
    rocks, info = [], []
    for m in mouths.values():
        for side in (-1, 1):
            local = np.array([[tube['cx'] + side * (tube['hw'] - rng.uniform(0.0, 0.08)),
                               m['y_local'] + m['outward'] * rng.uniform(0.12, 0.25), 0.]])
            xy = place(local)[0, :2]
            radius, height = rng.uniform(0.15, 0.22), rng.uniform(0.6, 0.95)
            rocks.append(mm.rock(xy, radius, height, rng))
            info.append({'xy': [float(v) for v in xy], 'radius': float(radius), 'height': float(height)})
    return mm.merge(rocks), info


def hoops(place, tube, y_south, y_north, count=5, wire=0.012):
    """Metal hoops of the wire tunnel, in course coordinates via ``place``."""
    parts = []
    angles = np.linspace(0, 2 * math.pi, 25)
    for y in np.linspace(y_south + 0.02, y_north - 0.02, count):
        points = [(tube['cx'] + tube['hw'] * math.cos(t), y, tube['cz'] + tube['hh'] * math.sin(t)) for t in angles]
        points = place(np.array(points))
        for p, q in zip(points, points[1:]):
            parts.append(mm.beam(p, q, wire))
    return mm.merge(parts)


def drape(place, tube, y_from, y_to, rng, steps=16, rows=10):
    """The rock-print tarp thrown over the tube's top, sagging and wrinkled,
    hanging lower on one side."""
    low_side = rng.choice([-1, 1])
    start, end = (math.radians(rng.uniform(5, 30)), math.radians(rng.uniform(150, 175)))
    if low_side < 0:
        end = math.radians(rng.uniform(175, 200))
    else:
        start = math.radians(rng.uniform(-20, 5))
    angles = np.linspace(start, end, steps)
    ys = np.linspace(y_from, y_to, rows)
    vertices = []
    for y in ys:
        wobble = rng.normal(0., 0.025, steps)
        r = 1.04 + wobble
        vertices += [(tube['cx'] + tube['hw'] * r[i] * math.cos(t), y, tube['cz'] + tube['hh'] * r[i] * math.sin(t))
                     for i, t in enumerate(angles)]
    vertices = place(np.array(vertices))
    triangles = []
    for r in range(rows - 1):
        for i in range(steps - 1):
            p = r * steps + i
            triangles += [(p, p + steps, p + steps + 1), (p, p + steps + 1, p + 1)]
    return vertices, np.array(triangles)
