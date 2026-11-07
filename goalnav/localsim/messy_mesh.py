"""Mesh pieces for the irregular test course (``localsim.messy_course``).

Every function returns ``(vertices, triangles)`` as NumPy arrays: vertices
(N, 3) in course metres (ENU, z up), triangles (M, 3) indices into them.
"""
import math

import numpy as np


def merge(parts):
    """Concatenate (vertices, triangles) pairs into one mesh."""
    vertices, triangles, base = [], [], 0
    for v, t in parts:
        if len(t) == 0:
            continue
        vertices.append(np.asarray(v, float))
        triangles.append(np.asarray(t, int) + base)
        base += len(v)
    if not vertices:
        return np.zeros((0, 3)), np.zeros((0, 3), int)
    return np.concatenate(vertices), np.concatenate(triangles)


def _frame(a, b):
    """Unit along-wall direction and its left normal for a centreline a -> b."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    along = (b - a) / np.linalg.norm(b - a)
    return along, np.array([-along[1], along[0]])


def box(a, b, thickness, z0, z1, lean=0.):
    """A wall slab on the centreline a -> b (XY), leaning ``lean`` radians
    toward its left normal (the top moves, the base stays)."""
    along, normal = _frame(a, b)
    a, b = np.asarray(a, float), np.asarray(b, float)
    half = normal * thickness / 2
    shift = normal * math.tan(lean) * (z1 - z0)
    corners = []
    for z, top in ((z0, False), (z1, True)):
        for p in (a - half, b - half, b + half, a + half):
            q = p + (shift if top else 0.)
            corners.append((q[0], q[1], z))
    faces = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4),
             (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    return np.array(corners), np.array(faces)


def window_wall(a, b, thickness, z0, z1, hole_u, hole_z, lean=0.):
    """A slab with a rectangular hole: ``hole_u`` (start, end) metres along
    a -> b and ``hole_z`` (bottom, top)."""
    along, _ = _frame(a, b)
    a = np.asarray(a, float)
    length = float(np.linalg.norm(np.asarray(b, float) - a))
    u0, u1 = hole_u
    at = lambda u: a + along * u
    parts = [box(at(0.), at(u0), thickness, z0, z1, lean),
             box(at(u1), at(length), thickness, z0, z1, lean),
             box(at(u0), at(u1), thickness, z0, hole_z[0], lean),
             box(at(u0), at(u1), thickness, hole_z[1], z1, lean)]
    return merge(parts)


def _bumps(rng, length, height, count, amplitude):
    """A smooth random bulge field on (u, v) in [0, length] x [0, height]."""
    centres = rng.uniform((0, 0), (length, height), (count, 2))
    sizes = rng.uniform(0.15, 0.45, count)
    heights = rng.uniform(0., amplitude, count)

    def field(u, v):
        total = np.zeros_like(u)
        for (cu, cv), s, h in zip(centres, sizes, heights):
            total += h * np.exp(-((u - cu) ** 2 + (v - cv) ** 2) / (2 * s * s))
        return total
    return field


def tarp(a, b, side, thickness, z0, z1, rng, hole=None, flap=(0., 0.), lean=0., step=0.1,
         bulge=0.06, skirt=0.12):
    """A tarp draped over one face of the wall a -> b.

    ``side`` +1 is the left face. The sheet sits 1-3 cm off the face with
    random bulges up to ``bulge``, flares out by up to ``skirt`` over the
    bottom 30 cm, and may run past the wall ends as loose flaps (``flap``:
    metres past a, past b). ``hole`` ((u0, u1), (z0, z1)) leaves a window
    open, the tarp edge sagging a little into it.
    """
    along, normal = _frame(a, b)
    a = np.asarray(a, float)
    length = float(np.linalg.norm(np.asarray(b, float) - a))
    start, end = -flap[0], length + flap[1]
    us = np.arange(start, end + step / 2, step)
    vs = np.arange(z0, z1 + step / 2, step)
    field = _bumps(rng, length, z1 - z0, max(2, int(length * 2)), bulge)
    uu, vv = np.meshgrid(us, vs, indexing='ij')
    out = thickness / 2 + rng.uniform(0.01, 0.03) + field(uu, vv - z0)
    out += np.clip((z0 + 0.3 - vv) / 0.3, 0., 1.) * skirt * rng.uniform(0.3, 1.)
    beyond = np.maximum(start - uu + 0., 0.) + np.maximum(uu - length, 0.)
    out += beyond * 0.4                                   # Flaps hang away from the end.
    lean_shift = math.tan(lean) * (vv - z0)
    offset = side * out + lean_shift
    xy = a + uu[..., None] * along + offset[..., None] * normal
    vertices = np.dstack([xy, vv]).reshape(-1, 3)
    nv = len(vs)
    keep = np.ones((len(us) - 1, nv - 1), bool)
    if hole is not None:
        (h0, h1), (hz0, hz1) = hole
        sag = rng.uniform(0., 0.08)
        cu = (us[:-1] + us[1:]) / 2
        cv = (vs[:-1] + vs[1:]) / 2
        half = step / 2                  # Cut the vinyl back to clear the opening, never into it.
        keep &= ~((cu[:, None] > h0 - half) & (cu[:, None] < h1 + half) & (cv[None] > hz0 - half)
                  & (cv[None] < hz1 + half - sag))
    triangles = []
    for i, j in zip(*np.nonzero(keep)):
        p = i * nv + j
        triangles += [(p, p + nv, p + nv + 1), (p, p + nv + 1, p + 1)]
    return vertices, np.array(triangles, int).reshape(-1, 3)


def rock(centre, radius, height, rng, subdivisions=2):
    """An irregular boulder: a noisy squashed icosphere resting on the floor."""
    t = (1 + 5 ** 0.5) / 2
    v = [(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0), (0, -1, t), (0, 1, t), (0, -1, -t), (0, 1, -t),
         (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1)]
    f = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11), (1, 5, 9), (5, 11, 4), (11, 10, 2),
         (10, 7, 6), (7, 1, 8), (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9), (4, 9, 5),
         (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)]
    vertices = [np.array(p, float) / np.linalg.norm(p) for p in v]
    for _ in range(subdivisions):
        cache, faces = {}, []
        for tri in f:
            mids = []
            for i, j in ((tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])):
                key = (min(i, j), max(i, j))
                if key not in cache:
                    m = vertices[i] + vertices[j]
                    vertices.append(m / np.linalg.norm(m))
                    cache[key] = len(vertices) - 1
                mids.append(cache[key])
            a, b, c = tri
            faces += [(a, mids[0], mids[2]), (b, mids[1], mids[0]), (c, mids[2], mids[1]), tuple(mids)]
        f = faces
    p = np.array(vertices)
    p *= rng.uniform(0.8, 1.2, (len(p), 1))
    p[:, 2] = np.maximum(p[:, 2], -0.3)                 # Flattened base.
    p[:, 2] = (p[:, 2] + 0.3) / 1.3                       # 0 .. 1
    p[:, :2] *= radius
    p[:, 2] *= height / max(p[:, 2].max(), 1e-6)
    p[:, :2] += centre
    return p, np.array(f)


def bevel(centre_x, mouth_y, outward, centre_z, half_width, half_height, drop, reach, rng, steps=12):
    """A tarp hanging off the top of a tube mouth: a sloped sheet from the
    upper rim forward ``reach`` (along ``outward``, +1/-1 in y) and down
    ``drop``, narrowing the opening's top."""
    angles = np.linspace(math.radians(20), math.radians(160), steps)
    rows = []
    for k, f in enumerate(np.linspace(0., 1., 4)):
        wobble = rng.normal(0., 0.02, steps)
        x = centre_x + half_width * np.cos(angles) * (1 - 0.1 * f)
        z = centre_z + half_height * np.sin(angles) - drop * f + wobble * f
        y = np.full(steps, mouth_y + outward * reach * f)
        rows.append(np.stack([x, y, z], axis=1))
    vertices = np.concatenate(rows)
    triangles = []
    for r in range(len(rows) - 1):
        for i in range(steps - 1):
            p = r * steps + i
            triangles += [(p, p + steps, p + steps + 1), (p, p + steps + 1, p + 1)]
    return vertices, np.array(triangles)


def write_obj(path, groups):
    """Write ``{name: (vertices, triangles)}`` as a Z-up OBJ in metres."""
    lines, base = ['# Irregular localsim course (localsim.messy_course)', 'o messy_course'], 1
    for name, (vertices, triangles) in groups.items():
        if len(triangles) == 0:
            continue
        lines.append(f'g {name}')
        lines += [f'v {x:.4f} {y:.4f} {z:.4f}' for x, y, z in vertices]
        lines += [f'f {a + base} {b + base} {c + base}' for a, b, c in triangles]
        base += len(vertices)
    path.write_text('\n'.join(lines) + '\n')


def beam(p0, p1, width, depth=None):
    """A square-section stick from 3D point p0 to p1 (struts, posts, battens)."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    axis = p1 - p0
    axis /= np.linalg.norm(axis)
    helper = np.array([0., 0., 1.]) if abs(axis[2]) < 0.9 else np.array([1., 0., 0.])
    u = np.cross(axis, helper)
    u /= np.linalg.norm(u)
    v = np.cross(axis, u)
    a, b = width / 2, (depth or width) / 2
    ring = [u * a + v * b, -u * a + v * b, -u * a - v * b, u * a - v * b]
    vertices = np.array([p0 + r for r in ring] + [p1 + r for r in ring])
    faces = [(0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6)]
    for k in range(4):
        n = (k + 1) % 4
        faces += [(k, n + 4, n), (k, k + 4, n + 4)]
    return vertices, np.array(faces)


def block(centre, size, yaw):
    """An axis-aligned-in-yaw box resting on the floor (paver bricks)."""
    c, s = math.cos(yaw), math.sin(yaw)
    hx, hy = size[0] / 2, size[1] / 2
    corners = [np.array(centre[:2]) + np.array([c * x - s * y, s * x + c * y]) for x, y in
               ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))]
    v, t = box(corners[0] + (corners[3] - corners[0]) / 2, corners[1] + (corners[2] - corners[1]) / 2,
               size[1], 0., size[2])
    return v, t
