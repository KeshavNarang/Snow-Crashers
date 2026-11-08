"""What the stereo points say about the wall (no ROS, no OpenCV: tested on its own).

stereo_depth bins the level-frame points of a drone-wide corridor into BIN x BIN metre cells
(F forward, U up, from the camera) and publishes the cell counts. Working on counts, not raw
points, is what keeps false matches out: they are scattered, a real surface is dense. On exact
synthetic pairs (test/synthetic.py) the densest false cell held 24 points, a wall 0.5 / 1.0 /
2.5 m ahead 1482 / 603 / 67 (median cell), so a cell needs ``min_cell`` points to count.

    front        horizontal distance to the nearest dense surface at the drone's own height
                 (band_low below the camera to band_high above it), floor rejected: the wall
    top_u        top of the highest dense cell of the wall face (m above the camera; <0 below)
    top_visible  that is clearly below the camera's highest ray (about -0.5 deg on skybeta-03:
                 a camera pitched 27 deg down never sees above its own height), so above it
                 the camera sees past the wall: the top edge is in view and measured
    floor_free   how far ahead the floor is still seen. A wall hides the floor behind it, so
                 this ends at the wall even when the wall itself has no texture to match
"""
import math

import numpy as np

BIN = 0.05
_DENSE_CELLS = 1 << 20      # bincount path while the cell grid spanned is at most this big (8 MB)


def bin_points(flu, corridor=0.4, min_count=3):
    """N x 3 level-frame points (F, L, U) -> list of [fi, ui, count] cells (|L| < corridor)."""
    flu = np.asarray(flu, dtype=float).reshape(-1, 3)
    flu = flu[np.abs(flu[:, 1]) < corridor]
    if len(flu) == 0:
        return []
    idx = np.floor(flu[:, [0, 2]] / BIN).astype(np.int64)
    f, u = idx[:, 0], idx[:, 1]
    f0, u0 = int(f.min()), int(u.min())
    width = int(u.max()) - u0 + 1
    if (int(f.max()) - f0 + 1) * width <= _DENSE_CELLS:
        # Dense counting: one integer per cell (forward-major, so ascending order = the sorted
        # (forward, up) order np.unique gave), counted with bincount. np.unique(axis=0) on an N x 2
        # array is a slow structured sort (44 % of a frame's post-matching time before this).
        counts = np.bincount((f - f0) * width + (u - u0))
        cells = np.flatnonzero(counts >= max(min_count, 1))
        return [[int(c // width) + f0, int(c % width) + u0, int(counts[c])] for c in cells]
    keys, counts = np.unique(idx, axis=0, return_counts=True)     # huge spread: the original path
    keep = counts >= min_count
    return [[int(f), int(u), int(n)] for (f, u), n in zip(keys[keep], counts[keep])]


def _runs(rows, counts):
    """Vertically connected runs of cell rows (one empty row allowed): [(rows, total), ...]."""
    order = np.argsort(rows)
    rows, counts = rows[order], counts[order]
    runs, start = [], 0
    for i in range(1, len(rows) + 1):
        if i == len(rows) or rows[i] - rows[i - 1] > 2:
            runs.append((rows[start:i], float(counts[start:i].sum())))
            start = i
    return runs


def _by_row(ui, n):
    rows = np.unique(ui)
    return rows, np.array([n[ui == r].sum() for r in rows])


def analyze(cells, cam_height, top_edge_deg, wall_distance=None, band_low=0.35, band_high=0.25,
            floor_tol=0.12, min_cell=40, min_rows=3, near_depth=0.35, edge_margin=0.05):
    """A surface only counts if it is dense (min_cell points per cell) AND vertical: at least
    ``min_rows`` connected 5 cm rows tall. False stereo matches come from the top rows (the
    camera grazing the far floor) and so sit in one row at camera height; a wall fills the band."""
    out = {'front': None, 'top_u': None, 'top_visible': False, 'n_band': 0, 'n_near': 0,
           'floor_free': None}
    c = np.asarray(cells, dtype=float).reshape(-1, 3)
    if len(c) == 0:
        return out
    fi, ui, n = c[:, 0].astype(int), c[:, 1].astype(int), c[:, 2]
    f_lo, u_lo = fi * BIN, ui * BIN
    f_mid = f_lo + BIN / 2
    dense = (n >= min_cell) & (f_lo > 0.0)
    if cam_height is not None:
        # How far ahead the floor is seen: a wall hides the floor behind it, textured or not.
        floor = (n >= max(3.0, min_cell / 8.0)) & (np.abs(u_lo + BIN / 2 + float(cam_height)) <= floor_tol)
        if floor.any():
            out['floor_free'] = round(float(f_mid[floor].max()), 3)
        dense &= u_lo > -float(cam_height) + floor_tol          # not the floor
    band = dense & (u_lo + BIN > -band_low) & (u_lo < band_high)
    out['n_band'] = int(n[band].sum())
    for f0 in np.unique(f_mid[band]):                            # nearest first
        group = band & (f_mid >= f0) & (f_mid <= f0 + 0.1)
        rows, counts = _by_row(ui[group], n[group])
        if max(len(r) for r, _ in _runs(rows, counts)) >= min_rows:
            out['front'] = round(float(np.average(f_mid[group], weights=n[group])), 3)
            break
    reference = wall_distance if wall_distance is not None else out['front']
    if reference is None:
        return out
    near = dense & (f_mid >= reference - 0.1) & (f_mid <= reference + near_depth)
    out['n_near'] = int(n[near].sum())
    if not near.any():
        return out
    rows, counts = _by_row(ui[near], n[near])
    face, _total = max(_runs(rows, counts), key=lambda run: run[1])   # the wall face, not a stray cluster
    top_u = float((face.max() + 1) * BIN)
    in_face = near & (ui >= face.min()) & (ui <= face.max())
    edge_u = float(np.average(f_mid[in_face], weights=n[in_face])) * math.tan(math.radians(top_edge_deg))
    out['top_u'] = round(top_u, 3)
    out['top_visible'] = top_u < edge_u - edge_margin
    return out
