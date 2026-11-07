"""Stereo depth errors fitted to the OAK-D S2 on the drone (/tmp/oakdata/OAKD_FINDINGS.md).

Measured at 640x400 with left-right check and subpixel, DEFAULT preset (the
one that leaves holes in gaps instead of phantom near returns):

- disparity noise 0.13 px median, 0.59 px at the 90th percentile: a mixture
  of 0.2 px (most pixels) and 0.9 px (one in ten);
- 2.7 % gross outliers;
- no depth within 5-10 px of a depth jump, and a strip at the left edge of
  the frame about 30 px x (1 m / depth) wide;
- minimum range 0.31 m (the disparity search limit; ``Camera.min_depth``);
- errors depend on the surface: plain ones get holes (spatially correlated,
  not salt-and-pepper), repeating prints and mesh get patches of confident
  wrong depth. The per-surface rates below are estimates until the plywood,
  tarp, floor and net captures are made.
"""
import numpy as np

BASELINE_M = 0.075
NOISE_PX = ((0.9, 0.2), (0.1, 0.9))          # (share, sigma in px)
OUTLIER = 0.027
EDGE_JUMP = 0.12                             # relative depth step that leaves a hole band
LEFT_STRIP_PX_M = 30.                        # strip width x depth (px m)
# kind: (fill rate, share of confident wrong patches)
SURFACES = {'floor': (0.80, 0.005), 'plywood': (0.75, 0.005), 'tarp': (0.88, 0.02), 'mesh': (0.60, 0.06),
            'beyond': (0.70, 0.01), 'net': (1.0, 0.)}
KINDS = list(SURFACES)


def _field(rng, shape, cells=(6, 10)):
    """A smooth random field in [0, 1) over the ray grid (blob-shaped holes)."""
    coarse = rng.random(cells)
    rows = np.linspace(0, cells[0] - 1, shape[0])
    cols = np.linspace(0, cells[1] - 1, shape[1])
    r0, c0 = np.minimum(rows.astype(int), cells[0] - 2), np.minimum(cols.astype(int), cells[1] - 2)
    fr, fc = (rows - r0)[:, None], (cols - c0)[None, :]
    f = (coarse[np.ix_(r0, c0)] * (1 - fr) * (1 - fc) + coarse[np.ix_(r0 + 1, c0)] * fr * (1 - fc)
         + coarse[np.ix_(r0, c0 + 1)] * (1 - fr) * fc + coarse[np.ix_(r0 + 1, c0 + 1)] * fr * fc)
    ranks = np.argsort(np.argsort(f, axis=None)).reshape(f.shape)
    return ranks / f.size                     # uniform marginal, smooth in space


def apply(depth, kinds, grid, camera, rng):
    """Corrupt true ray depths (m, inf = no surface) the way the OAK-D does.

    ``kinds``: index into KINDS per ray; ``grid``: (rows, cols) of the ray
    grid, row-major; ``camera``: fx, stride, min_depth."""
    rows, cols = grid
    d = depth.reshape(rows, cols).copy()
    k = kinds.reshape(rows, cols)
    finite = np.isfinite(d)
    fill = np.array([SURFACES[name][0] for name in KINDS])[k]
    wrong = np.array([SURFACES[name][1] for name in KINDS])[k]
    holes = _field(rng, (rows, cols)) > fill
    patches = _field(rng, (rows, cols), (8, 13)) < wrong
    out = d.copy()
    factor = rng.uniform(0.6, 1.5)
    out[patches & finite] *= factor           # a repeating texture matched one period off
    sigma_px = np.where(rng.random(d.shape) < NOISE_PX[0][0], NOISE_PX[0][1], NOISE_PX[1][1])
    z = np.where(finite, out, 0.)
    out = out + rng.normal(0., 1., d.shape) * sigma_px * z ** 2 / (camera.fx * BASELINE_M)
    gross = (rng.random(d.shape) < OUTLIER) & finite
    out[gross] = camera.min_depth + rng.random(gross.sum()) * np.maximum(np.minimum(d[gross], 6.) - camera.min_depth, 0.)
    jump = np.zeros_like(finite)
    with np.errstate(invalid='ignore'):
        for axis in (0, 1):
            a, b = np.take(d, range(d.shape[axis] - 1), axis=axis), np.take(d, range(1, d.shape[axis]), axis=axis)
            step = np.abs(a - b) > EDGE_JUMP * np.minimum(a, b)
            pad = [(0, 0), (0, 0)]
            pad[axis] = (0, 1)
            jump |= np.pad(step, pad)
            pad[axis] = (1, 0)
            jump |= np.pad(step, pad)
    u_px = (np.arange(cols) + 0.5) * camera.stride
    strip = u_px[None, :] < LEFT_STRIP_PX_M / np.maximum(z, 0.05)
    out[holes | jump | strip | ~finite] = np.inf
    return out.reshape(-1)
