"""The Jetson's stereo depth (drone_ref/stereo_node.py), as measured on skybeta-03 on 2026-11-08.

stereo_node runs overwall_nav.stereo.StereoRig on the mono pair: scale 0.4 (410x256), 96
disparities, every 2nd pixel, rows 0-90 %, columns 96-410 (no match is possible left of the
disparity range). Measured from a read-only capture (agents/logs/2026-11-08-sim-stereo-drone.md):

- intrinsics at scale 0.4 fx 263.9, cx 210.2, cy 129.4; baseline 0.0753 m, so fB = 19.87 px m;
- depth only in -22.8..+37.5 deg across, -26.3..+20.6 deg down;
- disparity noise ~0.3 px (sigma_z ~ 0.015 z^2); 3.5 % of points > 15 cm off the wall's plane;
- gaps follow texture: a dark floor returned nothing; ~39 % of the matchable grid valid, ~6700 points;
- 3.77 Hz with jittered frame intervals and ages (deciles below).

The per-surface fill rates are estimates (the floor, plywood and course tarps were NOT captured);
the overall valid share was tuned to the measured ~0.4.
"""
import numpy as np

from localsim import oakd_model

FX, CX, CY = 263.9, 210.2, 129.4               # CameraInfo at scale 0.4 (410x256)
SIZE = (410, 256)
BASELINE_M = 0.0753
FB = FX * BASELINE_M                           # 19.87 px m
NUM_DISPARITIES, STEP, ROW_SPAN = 96, 2, (0.0, 0.9)
RANGE_M = (0.2, 6.0)
FLOOR_Z = 0.03
NOISE_PX = 0.3
OUTLIER = 0.03                                 # gross mismatches, of the matched points
OUTLIER_PX = (2., 12.)                         # their disparity error
# Share of matchable pixels that return on each surface (oakd_model.KINDS): printed tarps and the
# busy hall match; plain plywood, the mottled floor and the thin wire mesh mostly do not.
TEXTURE = {'floor': 0.38, 'plywood': 0.35, 'tarp': 0.8, 'mesh': 0.3, 'beyond': 0.55, 'net': 1.0}
# Deciles measured on skybeta-03 (47 frames), each over its mean (0.265 s, 0.159 s).
INTERVAL_DECILES = np.array([0.1, 0.167, 0.2, 0.233, 0.233, 0.267, 0.267, 0.267, 0.333, 0.383, 0.567])
AGE_DECILES = np.array([0.077, 0.087, 0.089, 0.097, 0.108, 0.111, 0.157, 0.182, 0.221, 0.281, 0.52])


def grid():
    """(u, v) of every pixel StereoRig searches, at scale 0.4, row-major."""
    w, h = SIZE
    us = np.arange(NUM_DISPARITIES, w, STEP)
    vs = np.arange(int(ROW_SPAN[0] * h), int(ROW_SPAN[1] * h), STEP)
    uu, vv = np.meshgrid(us, vs)
    return uu.reshape(-1).astype(float), vv.reshape(-1).astype(float)


def grid_shape():
    w, h = SIZE
    return len(range(int(ROW_SPAN[0] * h), int(ROW_SPAN[1] * h), STEP)), len(range(NUM_DISPARITIES, w, STEP))


def disparity_noise(z, rng):
    """Depths (m) after the matcher: 0.3 px disparity noise and a few gross mismatches."""
    d = FB / z + rng.normal(0., NOISE_PX, z.shape)
    gross = rng.random(z.shape) < OUTLIER
    d[gross] += rng.choice([-1., 1.], gross.sum()) * rng.uniform(*OUTLIER_PX, gross.sum())
    with np.errstate(divide='ignore'):
        return np.where(d > 0, FB / np.maximum(d, 1e-9), np.inf)


def points(world, camera, position, rotation, rng, dark_floor=False):
    """(N, 3) float32 points in the left optical frame, as stereo_node publishes them.

    ``camera`` gives the mount (pitch, offset); ``world`` a MessyWorld. ``dark_floor``: the floor
    has no texture at all and returns nothing (seen on the drone over a dark floor)."""
    u, v = grid()
    rays = np.column_stack([(u - CX) / FX, (v - CY) / FX, np.ones_like(u)])
    directions = rays @ (rotation @ camera.body_from_optical()).T
    norms = np.linalg.norm(directions, axis=1)
    directions /= norms[:, None]
    origin = position + rotation @ np.asarray(camera.offset)
    hit = world.cast(origin, directions, RANGE_M[0] * 0.5, RANGE_M[1] * 1.6)
    out, kinds = world.surfaces(origin, directions, hit)
    with np.errstate(invalid='ignore'):     # Any floor, the course's or the hall's beyond the net.
        kinds = np.where(origin[2] + directions[:, 2] * out < FLOOR_Z, oakd_model.KINDS.index('floor'), kinds)
    fill = np.array([0. if dark_floor and name == 'floor' else TEXTURE[name] for name in oakd_model.KINDS])[kinds]
    matched = oakd_model._field(rng, grid_shape()).reshape(-1) < fill
    z = np.full(len(u), np.inf)
    ok = np.isfinite(out) & matched
    z[ok] = disparity_noise(out[ok] / norms[ok], rng)
    ok &= (z >= max(RANGE_M[0], FB / NUM_DISPARITIES)) & (z <= RANGE_M[1])
    z, u, v = z[ok], u[ok], v[ok]
    return np.column_stack([(u - CX) * z / FX, (v - CY) * z / FX, z]).astype(np.float32)


def valid_mask(pts):
    """Which grid cells returned a point (rows, cols), from published points."""
    rows, cols = grid_shape()
    mask = np.zeros((rows, cols), bool)
    p = np.asarray(pts, float)
    u = np.rint((p[:, 0] / p[:, 2] * FX + CX - NUM_DISPARITIES) / STEP).astype(int)
    v = np.rint((p[:, 1] / p[:, 2] * FX + CY - int(ROW_SPAN[0] * SIZE[1])) / STEP).astype(int)
    mask[v, u] = True
    return mask


def depth_image(pts, camera):
    """A uint16 mm depth image at ``camera``'s size (nearest per stride tile), for the viewer."""
    s = camera.stride
    rows, cols = -(-camera.height // s), -(-camera.width // s)
    tiles = np.full(rows * cols, np.inf)
    p = np.asarray(pts, float)
    if len(p):
        u = (p[:, 0] / p[:, 2] * camera.fx + camera.cx) // s
        v = (p[:, 1] / p[:, 2] * camera.fx + camera.cy) // s
        inside = (u >= 0) & (u < cols) & (v >= 0) & (v < rows)
        np.minimum.at(tiles, (v[inside] * cols + u[inside]).astype(int), p[inside, 2])
    mm = np.where(np.isfinite(tiles), np.round(tiles * 1000), 0).astype(np.uint16).reshape(rows, cols)
    return np.repeat(np.repeat(mm, s, axis=0), s, axis=1)[:camera.height, :camera.width]


def _draw(deciles, n, mean, rng):
    """``n`` samples from the measured deciles (linear between them), scaled to ``mean``."""
    shape = deciles / np.mean((deciles[1:] + deciles[:-1]) / 2)
    return np.interp(rng.random(n), np.linspace(0., 1., len(deciles)), shape) * mean


def schedule(hz, latency, seconds, rng):
    """(N, 2) capture and publish times (s) of the depth frames: measured jitter, the given means,
    published in capture order (as on the drone)."""
    n = int(seconds * hz * 1.5) + 2
    capture = np.cumsum(_draw(INTERVAL_DECILES, n, 1. / hz, rng)) - 1. / hz * rng.random()
    capture = capture[(capture >= 0.) & (capture < seconds)]
    publish = capture + _draw(AGE_DECILES, len(capture), latency, rng)
    for i in range(1, len(publish)):
        publish[i] = max(publish[i], publish[i - 1] + 1e-3)
    return np.column_stack([capture, publish])
