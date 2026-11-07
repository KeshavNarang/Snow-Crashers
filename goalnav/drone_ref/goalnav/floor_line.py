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


def turn_along(line_map, origin, direction, turn_side, along=(-0.6, 0.4), across=(0.15, 0.6), min_cells=6):
    """Where the line turns: along ``direction`` from ``origin``, the median
    position of tape cells off to ``turn_side`` (+1 left, -1 right), between
    ``across`` metres from the corridor's line. None until seen enough."""
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
    return float(np.median(a[pick]))
