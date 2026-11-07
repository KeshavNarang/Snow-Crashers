"""A phone scan of the course (a Scaniverse PLY), brought into the course frame.

    python3 tools/course_scan.py ~/Downloads/dronehacks-course-scan-2026-11-06.ply

Writes ``assets/course-scan.npz``: the points inside the net (and 0.25 m past
it), floor to just over the wall tops, on 2 cm voxels (the bare floor 4 cm) with
isolated specks removed, in the frame the simulator and goals.yaml use (CAD axes, the nets'
square centred on the origin, the floor at z = 0). Ceiling, hall and the rest of
the room are dropped.

The scan is placed by fitting its walls to the nets' square and the barrier
line (the references least likely to move): a coarse search over rotation and
offset against the CAD's walls, then a fine fit on the nets and barrier alone.
"""
import argparse
import math
from pathlib import Path

import numpy as np
from scipy import ndimage, optimize

ROOT = Path(__file__).resolve().parents[1]
NET, BARRIER_Y = 3.048, 0.4572


def read_ply(path):
    raw = Path(path).read_bytes()
    head = raw.index(b'end_header\n') + len(b'end_header\n')
    count = int(next(l for l in raw[:head].decode().splitlines() if l.startswith('element vertex')).split()[-1])
    dtype = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('r', 'u1'), ('g', 'u1'), ('b', 'u1')])
    a = np.frombuffer(raw, dtype, count=count, offset=head)
    return np.column_stack([a['x'], a['y'], a['z']]).astype(float), np.column_stack([a['r'], a['g'], a['b']])


def _rot(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def _distance_field(segments, res=0.01, extent=3.6):
    n = int(2 * extent / res)
    free = np.ones((n, n), bool)
    for a, b in segments:
        a, b = np.asarray(a, float), np.asarray(b, float)
        for t in np.linspace(0, 1, int(np.linalg.norm(b - a) / res * 2) + 2):
            i, j = ((a + (b - a) * t + extent) / res).astype(int)
            if 0 <= i < n and 0 <= j < n:
                free[j, i] = False
    return ndimage.distance_transform_edt(free) * res, res, extent


def _cost(points, field, pose, trunc):
    dist, res, extent = field
    xy = points @ _rot(pose[0]).T + pose[1:]
    ij = ((xy + extent) / res).astype(int)
    ok = (ij >= 0).all(1) & (ij < dist.shape[0]).all(1)
    d = np.full(len(xy), trunc)
    d[ok] = np.minimum(dist[ij[ok, 1], ij[ok, 0]], trunc)
    return float(np.mean(d ** 2))


def _refine(points, field, pose, truncs):
    for trunc in truncs:
        step = [pose, pose + (0.01, 0, 0), pose + (0, 0.03, 0), pose + (0, 0, 0.03)]
        pose = optimize.minimize(lambda x: _cost(points, field, x, trunc), pose, method='Nelder-Mead',
                                 options={'xatol': 1e-5, 'fatol': 1e-9, 'maxiter': 3000, 'initial_simplex': step}).x
    return pose


def register(p):
    """(theta, tx, ty) taking the scan's xy into the course frame."""
    from drone_ref.course import read_obj
    from localsim.messy_course import CAD, _cad_wall
    mesh = read_obj(CAD)
    walls = [_cad_wall(mesh, g) for g in mesh.groups if g.startswith(('M', 'C1', 'C2a', 'A1', 'Barrier'))]
    rng = np.random.default_rng(0)
    mid = p[(p[:, 2] > 0.4) & (p[:, 2] < 1.5), :2]
    mid = mid[rng.choice(len(mid), min(40000, len(mid)), replace=False)]
    field = _distance_field(walls, res=0.02, extent=3.5)
    best = min(((_cost(mid, field, np.array([th, tx, ty]), 0.4), th, tx, ty)
                for th in np.radians(np.arange(0, 360, 3)) for tx in np.arange(-3, 3.01, 0.15)
                for ty in np.arange(-3, 3.01, 0.15)))
    pose = _refine(mid, field, np.array(best[1:]), (0.3, 0.15, 0.08))
    # Then the nets' square and the barrier only (the walls inside can move).
    xy = p[:, :2] @ _rot(pose[0]).T + pose[1:]
    edge = np.min(np.abs(np.concatenate([xy - NET, xy + NET], axis=1)), axis=1)
    net = (edge < 0.25) & (p[:, 2] > 0.3) & (p[:, 2] < 2.5) & (np.abs(xy) < NET + 0.25).all(1)
    barrier = (np.abs(xy[:, 1] - BARRIER_Y) < 0.25) & (xy[:, 0] > -2.9) & (xy[:, 0] < 1.4) \
        & (p[:, 2] > 0.5) & (p[:, 2] < 1.5)
    square = [((-NET, NET), (NET, NET)), ((-NET, -NET), (NET, -NET)), ((NET, -NET), (NET, NET)),
              ((-NET, -NET), (-NET, NET)), ((-NET, BARRIER_Y), (1.6, BARRIER_Y))]
    return _refine(p[net | barrier, :2], _distance_field(square), pose, (0.2, 0.1, 0.06))


def clean(p, rgb, pose, margin=0.25, top=1.95, voxel=0.02, neighbours=4, floor_voxel=0.04):
    """Points in the course frame, cropped, levelled, one per voxel, specks
    removed; the bare floor (below 3 cm) thinned to ``floor_voxel``."""
    w = p.copy()
    w[:, :2] = p[:, :2] @ _rot(pose[0]).T + pose[1:]
    near = (np.abs(w[:, :2]) < NET - 0.15).all(1) & (w[:, 2] < 0.25)
    counts, edges = np.histogram(w[near, 2], bins=np.arange(-0.2, 0.3, 0.01))
    w[:, 2] -= edges[np.argmax(counts)] + 0.005
    keep = (np.abs(w[:, :2]) < NET + margin).all(1) & (w[:, 2] > -0.05) & (w[:, 2] < top)
    w, rgb = w[keep], rgb[keep]
    cell = np.floor(w / voxel).astype(np.int64) + 500
    key = (cell[:, 0] * 1000 + cell[:, 1]) * 1000 + cell[:, 2]
    keys, first = np.unique(key, return_index=True)
    occupied = set(keys.tolist())
    offsets = [(dx * 1000 + dy) * 1000 + dz for dx in range(-2, 3) for dy in range(-2, 3) for dz in range(-2, 3)
               if (dx, dy, dz) != (0, 0, 0)]
    good = np.array([sum(k + o in occupied for o in offsets) >= neighbours for k in keys.tolist()])
    w, rgb = w[first[good]], rgb[first[good]]
    return thin_floor(w, rgb, floor_voxel)


def thin_floor(w, rgb, voxel, below=0.03):
    floor = w[:, 2] < below
    _, keep = np.unique(np.floor(w[floor, :2] / voxel).astype(np.int64), axis=0, return_index=True)
    pick = np.concatenate([np.where(floor)[0][keep], np.where(~floor)[0]])
    return w[pick], rgb[pick]


def main():
    import sys
    sys.path.insert(0, str(ROOT))
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('ply')
    parser.add_argument('--out', default=str(ROOT / 'assets/course-scan.npz'))
    args = parser.parse_args()
    p, rgb = read_ply(args.ply)
    pose = register(p)
    points, colours = clean(p, rgb, pose)
    np.savez_compressed(args.out, xyz_mm=np.round(points * 1000).astype(np.int16), rgb=colours.astype(np.uint8),
                        source=np.array(Path(args.ply).name), pose=pose)
    print(f'rotation {math.degrees(pose[0]):.2f} deg, offset {pose[1]:.3f} {pose[2]:.3f} m; '
          f'{len(p)} -> {len(points)} points -> {args.out}')


if __name__ == '__main__':
    main()
