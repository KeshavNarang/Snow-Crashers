"""The drone net's PVC frame (team observation, 2026-11-07).

Each side of the 20 x 20 ft, 10 ft tall net hangs on a PVC grid of 5 ft
squares, 4 wide and 2 tall: uprights every 5 ft (the corners shared) and
rails on the floor, at 5 ft and at the top, all black. The pipes stand just
outside the net. A centre column of two pipes, one behind the other going
south from the barrier, holds the roof up; the scan puts it just south of
the barrier at the net's middle.
"""
import numpy as np

from localsim import messy_mesh as mm

PIPE = 0.048                     # 1 1/2 in schedule 40 PVC, outside diameter
HEIGHT = 3.048                   # 10 ft
COLUMNS, ROWS = 4, 2             # squares per side
CENTRE_X = 0.04                  # as scanned (assets/course-scan.npz, above the wall tops)
CENTRE_FROM_BARRIER = 0.22       # the pair's middle, south of the barrier's centreline
CENTRE_PAIR_GAP = 0.15           # between the two pipes' centres, north-south


def net_frame(nets):
    """{group name: (vertices, triangles)}, one grid per side, on the nets' planes."""
    r = PIPE / 2
    x0, x1 = -nets['west'] - r, nets['east'] + r
    y0, y1 = -nets['south'] - r, nets['north'] + r
    sides = {'north': ((x0, y1), (x1, y1)), 'south': ((x0, y0), (x1, y0)),
             'west': ((x0, y0), (x0, y1)), 'east': ((x1, y0), (x1, y1))}
    return {f'PVC net frame {side}': _grid(np.array(a), np.array(b)) for side, (a, b) in sides.items()}


def _grid(a, b):
    parts = []
    for f in np.linspace(0., 1., COLUMNS + 1):
        p = a + (b - a) * f
        parts.append(mm.beam((*p, 0.), (*p, HEIGHT), PIPE))
    for z in np.linspace(PIPE / 2, HEIGHT, ROWS + 1):
        parts.append(mm.beam((*a, z), (*b, z), PIPE))
    return mm.merge(parts)


def centre_column(barrier_y):
    """``barrier_y``: the barrier's centreline y at the column's x."""
    x, y = CENTRE_X, barrier_y - CENTRE_FROM_BARRIER
    half = CENTRE_PAIR_GAP / 2
    return {'PVC centre column': mm.merge([mm.beam((x, y + dy, 0.), (x, y + dy, HEIGHT), PIPE)
                                           for dy in (-half, half)])}
