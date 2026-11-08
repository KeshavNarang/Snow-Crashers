"""Exact synthetic stereo pairs: ray-trace skybeta-03's two cameras into a textured scene.

Calibration read off skybeta-03 on 2026-11-07 (/oak/*/camera_info, tf2_echo left -> right
optical). The rig is mounted pitched ``pitch_deg`` down (27 on skybeta-03). Scene, in the level
frame (F forward, L left, U up) with the left camera at the origin: a floor ``cam_height`` below,
an optional wall whose near face is ``wall_distance`` ahead and whose top is ``wall_top`` above
the floor, and a far wall 8 m ahead (beyond the 6 m range). Every surface carries the same
world-fixed texture in both cameras, so the pair is geometrically exact.
"""
from pathlib import Path
import math
import sys

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from overwall_nav.stereo import mount_matrix, quaternion_to_matrix   # noqa: E402

K_LEFT = [637.9068, 0.0, 527.4264, 0.0, 638.0483, 317.6430, 0.0, 0.0, 1.0]
K_RIGHT = [640.2593, 0.0, 513.6563, 0.0, 640.1569, 329.3553, 0.0, 0.0, 1.0]
SIZE = (1024, 640)
T_LR = (0.075, -0.0003, -0.001)          # right camera's origin in the left optical frame


def rpy_quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


Q_LR = rpy_quaternion(*(math.radians(a) for a in (0.321, 0.667, -0.014)))
_TEXTURE = {}


def texture(seed=0):
    if seed not in _TEXTURE:
        rng = np.random.default_rng(seed)
        noise = rng.integers(0, 256, (256, 256)).astype(np.uint8)
        _TEXTURE[seed] = cv2.GaussianBlur(cv2.resize(noise, (1024, 1024), interpolation=cv2.INTER_LINEAR),
                                          (5, 5), 0).astype(np.float32)
    return _TEXTURE[seed]


def _sample(tex, a, b, scale=0.0015):
    n = tex.shape[0]
    i = np.mod(np.floor(a / scale).astype(np.int64), n)
    j = np.mod(np.floor(b / scale).astype(np.int64), n)
    return tex[i, j]


def render(camera, pitch_deg=27.0, cam_height=0.85, wall_distance=None, wall_top=1.0,
           wall_thickness=0.1, wall_textured=True, seed=3):
    """mono8 image of ``camera`` ('left' or 'right')."""
    k = np.array(K_LEFT if camera == 'left' else K_RIGHT).reshape(3, 3)
    w, h = SIZE
    u, v = np.meshgrid(np.arange(w, dtype=np.float64) + 0.5, np.arange(h, dtype=np.float64) + 0.5)
    rays = np.stack([u, v, np.ones_like(u)], axis=-1) @ np.linalg.inv(k).T
    origin = np.zeros(3)
    if camera == 'right':
        r_lr = quaternion_to_matrix(*Q_LR)
        rays = rays @ r_lr.T
        origin = np.asarray(T_LR, dtype=float)
    m = mount_matrix(pitch_deg)
    d = rays @ m.T
    o = m @ origin
    df, dl, du = d[..., 0], d[..., 1], d[..., 2]
    inf = np.full(df.shape, np.inf)
    with np.errstate(divide='ignore', invalid='ignore'):
        t_floor = np.where(du < 0, (-cam_height - o[2]) / du, inf)
        t_far = np.where(df > 0, (8.0 - o[0]) / df, inf)
        t_face = t_top = inf
        if wall_distance is not None:
            t_face = np.where(df > 0, (wall_distance - o[0]) / df, inf)
            hit_u = o[2] + t_face * du
            t_face = np.where((hit_u >= -cam_height) & (hit_u <= wall_top - cam_height), t_face, inf)
            t_top = np.where(du < 0, (wall_top - cam_height - o[2]) / du, inf)
            hit_f = o[0] + t_top * df
            t_top = np.where((hit_f >= wall_distance) & (hit_f <= wall_distance + wall_thickness), t_top, inf)
    ts = np.stack([t_floor, t_far, t_face, t_top])
    which = np.argmin(ts, axis=0)
    t = np.min(ts, axis=0)
    pf, pl, pu = o[0] + t * df, o[1] + t * dl, o[2] + t * du
    tex = texture(seed)
    image = np.full(df.shape, 128.0, dtype=np.float32)
    floor, far, face, top = (which == 0), (which == 1), (which == 2), (which == 3)
    image[floor] = _sample(tex, pf[floor], pl[floor])
    image[far] = _sample(tex, pl[far] + 7.0, pu[far], scale=0.01)
    if wall_textured:
        image[face] = _sample(tex, pl[face] + 3.0, pu[face])
        image[top] = _sample(tex, pf[top] + 5.0, pl[top])
    image[~np.isfinite(t)] = 128.0
    return np.clip(image, 0, 255).astype(np.uint8)


def pair(**scene):
    return render('left', **scene), render('right', **scene)
