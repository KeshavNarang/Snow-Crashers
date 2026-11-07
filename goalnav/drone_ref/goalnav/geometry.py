"""Frames for the goal navigator: PX4 NED/FRD to ENU/FLU, depth to points.

The navigator works in one frame: PX4's local frame expressed as ENU, which
starts where the drone was placed (x right of the launch heading, y along
it, z up). Nothing is registered to the course; the CAD only says roughly
where things are relative to the launch.
"""
import math

import numpy as np

NED_TO_ENU = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
FLU_TO_FRD = np.diag([1., -1., -1.])
OPTICAL_TO_FLU = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])


def quaternion_matrix(q):
    """Rotation matrix of a (w, x, y, z) unit quaternion."""
    w, x, y, z = (float(v) for v in q)
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def px4_attitude_enu(q_ned_frd):
    """Body FLU -> local ENU rotation from PX4's FRD -> NED attitude."""
    return NED_TO_ENU @ quaternion_matrix(q_ned_frd) @ FLU_TO_FRD


def ned_to_enu(v):
    return NED_TO_ENU @ np.asarray(v, float)


def enu_to_ned(v):
    return NED_TO_ENU @ np.asarray(v, float)        # The swap is its own inverse.


def yaw_of(rotation):
    """ENU yaw (CCW from +x) of a body rotation's forward axis."""
    return math.atan2(rotation[1, 0], rotation[0, 0])


def yaw_enu_to_ned(yaw):
    return math.atan2(math.cos(yaw), math.sin(yaw))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def depth_points(depth_m, fx, fy, cx, cy, tile=8, min_depth=0.25, max_depth=6.0):
    """Optical-frame points from a metric depth image, keeping the nearest
    valid return of each ``tile`` x ``tile`` block (so a thin close obstacle
    between sample pixels is never skipped). Returns (N, 3)."""
    h, w = depth_m.shape
    h2, w2 = h - h % tile, w - w % tile
    d = np.where((depth_m >= min_depth) & (depth_m <= max_depth), depth_m, np.inf)[:h2, :w2]
    blocks = d.reshape(h2 // tile, tile, w2 // tile, tile).transpose(0, 2, 1, 3).reshape(h2 // tile, w2 // tile, -1)
    pick = blocks.argmin(axis=2)
    z = np.take_along_axis(blocks, pick[..., None], axis=2)[..., 0]
    rows, cols = np.divmod(pick, tile)
    v = np.arange(h2 // tile)[:, None] * tile + rows
    u = np.arange(w2 // tile)[None, :] * tile + cols
    ok = np.isfinite(z) & _consistent(z)
    z, u, v = z[ok], u[ok], v[ok]
    return np.column_stack([(u - cx) * z / fx, (v - cy) * z / fy, z])


def _consistent(z, tolerance=0.12, support=3):
    """Speckle filter: a tile's depth must agree with the median of its 3x3
    neighbours (and have ``support`` of them). Stereo's gross outliers are
    isolated; surfaces are not."""
    padded = np.pad(z, 1, constant_values=np.inf)
    rows, cols = z.shape
    stack = np.stack([padded[i:i + rows, j:j + cols] for i in range(3) for j in range(3) if (i, j) != (1, 1)])
    valid = np.isfinite(stack)
    with np.errstate(invalid='ignore'):
        median = np.nanmedian(np.where(valid, stack, np.nan), axis=0)
    return (valid.sum(axis=0) >= support) & (np.abs(z - median) <= tolerance * np.nan_to_num(median, nan=np.inf))


def optical_to_body(camera_pitch=0.):
    """Optical -> body FLU rotation for a camera tilted down by ``camera_pitch``."""
    c, s = math.cos(camera_pitch), math.sin(camera_pitch)
    return np.array([[c, 0., s], [0., 1., 0.], [-s, 0., c]]) @ OPTICAL_TO_FLU


def camera_to_odom(points_optical, position, rotation, camera_offset, camera_pitch=0.):
    """Optical-frame points to the odometry frame. ``camera_pitch`` tilts the
    camera down (radians) about body y."""
    c, s = math.cos(camera_pitch), math.sin(camera_pitch)
    pitch = np.array([[c, 0., s], [0., 1., 0.], [-s, 0., c]])
    body = points_optical @ (pitch @ OPTICAL_TO_FLU).T + np.asarray(camera_offset, float)
    origin = np.asarray(position, float) + rotation @ np.asarray(camera_offset, float)
    return body @ rotation.T + np.asarray(position, float), origin
