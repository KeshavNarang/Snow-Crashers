"""The black tape line on the orange floor, from the downward-tilted RGB camera.

Dark, colourless pixels below the horizon are cast onto the floor plane and
counted in a grid over many frames (a wall pixel does not land on the same
floor cell from different poses; the tape does). The route uses one feature
of it: where it turns east at the end of the first corridor, which is CP1.
"""
import numpy as np


def dark_floor_points(image, fx, fy, cx, cy, camera_origin, camera_rotation, floor_z, max_range=3.0, stride=2):
    """Floor-plane XY of the tape's pixels (dark and grey) in one frame."""
    rgb = image[::stride, ::stride].astype(float)
    brightness = rgb.sum(axis=2)
    chroma = rgb.max(axis=2) - rgb.min(axis=2)
    v, u = np.nonzero((brightness < 150) & (chroma < 35))
    if not len(v):
        return np.zeros((0, 2))
    rays = np.column_stack([(u * stride + 0.5 * stride - cx) / fx, (v * stride + 0.5 * stride - cy) / fy,
                            np.ones(len(u))]) @ camera_rotation.T
    down = rays[:, 2] < -0.05
    t = (floor_z - camera_origin[2]) / rays[down, 2]
    points = camera_origin[:2] + rays[down, :2] * t[:, None]
    near = np.linalg.norm(points - camera_origin[:2], axis=1) < max_range
    return points[near]


class LineMap:
    """Counts of tape pixels per 5 cm floor cell (same grid as the live map)."""

    def __init__(self, live_map):
        self.map = live_map
        self.counts = np.zeros((live_map.n, live_map.n), np.int32)

    def add(self, points):
        ij = self.map.cell(points)
        ok = self.map.inside(ij)
        np.add.at(self.counts, (ij[ok, 0], ij[ok, 1]), 1)

    def cells(self, min_count=3):
        return self.map.centre_of(np.argwhere(self.counts >= min_count))


def turn_along(line_map, origin, direction, turn_side, along=(-0.6, 0.25), across=(0.15, 0.6), min_cells=6):
    """Where the line turns: along ``direction`` from ``origin``, where the
    tape cells off to ``turn_side`` (+1 left, -1 right), between ``across``
    metres from the corridor's line, cross it. None until seen enough.
    ``along`` stops short of the net's floor rail (the net is 0.46 m past
    the turn, the goal's prior ~0.08 m)."""
    cells = line_map.cells()
    if not len(cells):
        return None
    d = np.asarray(direction, float)
    n = np.array([-d[1], d[0]]) * turn_side
    rel = cells - np.asarray(origin, float)
    a, c = rel @ d, rel @ n
    pick = (a > along[0]) & (a < along[1]) & (c > across[0]) & (c < across[1])
    if pick.sum() < min_cells:
        return None
    other = (a > along[0]) & (a < along[1]) & (c < -0.1) & (c > -0.45)
    # The crossing tape is the strongest straight run of cells within ~20
    # degrees of square to the corridor (the course heading may not be
    # learned yet, so it can lie tilted in this frame); the tape leading up to
    # the turn, slanted with the wall beside it, is spread along. The net's
    # black PVC floor rail looks like tape too, but runs on across the
    # corridor (the tape turns one way only): a run with cells on the other
    # side as well is the rail, dropped. Read off where the tape meets the
    # corridor's line (across 0).
    a_other, c_other = a[other], c[other]
    a, c = a[pick], c[pick]
    for _ in range(3):
        slope, at_line = _strongest_run(a, c, along)
        row = np.abs(a - (slope * c + at_line)) < 0.05
        if np.sum(np.abs(a_other - (slope * c_other + at_line)) < 0.05) < 3:
            fit = _crossing(c[row], a[row], min_cells)
            return fit if fit is not None else float(at_line)
        a, c = a[~row], c[~row]
        if len(a) < min_cells:
            return None
    return None


def _strongest_run(a, c, along):
    """(slope, distance along at across 0) of the straight run of cells
    within ~20 degrees of square to the corridor with the most cells."""
    best = (-1, 0., 0.)
    for slope in np.linspace(-0.35, 0.35, 15):
        counts, edges = np.histogram(a - slope * c, bins=np.arange(along[0] - 0.3, along[1] + 0.3, 0.025))
        if counts.max() > best[0]:
            best = (counts.max(), slope, edges[np.argmax(counts)] + 0.0125)
    return best[1], best[2]


def _crossing(c, a, min_cells, keep=0.03):
    """Distance along where a straight run of tape cells (across ``c``,
    along ``a``) meets across 0: a line fit, refitted twice without the cells
    more than ``keep`` off it (the slanted tape beside the turn)."""
    for _ in range(3):
        if len(c) < min_cells or np.ptp(c) < 0.15:
            return None
        slope, at_line = np.polyfit(c, a, 1)
        near = np.abs(a - (slope * c + at_line)) < keep
        c, a = c[near], a[near]
    return float(at_line) if abs(slope) < 0.35 else None        # within ~20 degrees: crossing tape
