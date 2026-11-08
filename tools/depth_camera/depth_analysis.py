"""depth_analysis: numpy-only checks for the OAK-D's on-device depth and for VIO regressions.

Pure functions, testable anywhere (test_depth_analysis.py), run the same on the drone and a laptop.

  decode_depth(...)        sensor_msgs/Image depth payload (16UC1 mm or 32FC1 m) -> metres, NaN = no return
  valid_fraction(d)        share of pixels with a depth
  backproject(d, K)        valid pixels -> 3-D points in the camera optical frame (x right, y down, z forward)
  floor_from_depth(d, K)   dominant plane -> camera pitch below horizontal and height above that plane
  compare_depth(a, b)      relative error of a against reference b where both are valid
  best_shift(a, b)         horizontal pixel offset that best aligns a with b (detects a wrong reference frame)
  stamp_stats(stamps, hz)  frames missing from a stream by header stamp (what cuVSLAM sees)
  pair_fraction(l, r)      share of left frames with a right frame at the same instant
  camera_info_diff(a, b)   which CameraInfo fields differ
  tf_diff(a, b)            translation and rotation between two transforms
"""
import math

import numpy as np

_DTYPES = {'16UC1': ('u2', 2), 'mono16': ('u2', 2), '32FC1': ('f4', 4)}


def decode_depth(data, height, width, encoding, step=None, is_bigendian=0):
    if encoding not in _DTYPES:
        raise ValueError(f'not a depth encoding: {encoding!r}')
    kind, bpp = _DTYPES[encoding]
    row = width * bpp
    step = step or row
    if step < row or len(data) < height * step:
        raise ValueError(f'depth data too short: {len(data)} bytes for {height} rows of {step}')
    raw = np.frombuffer(bytes(data[:height * step]), np.uint8).reshape(height, step)[:, :row]
    arr = np.ascontiguousarray(raw).view(('>' if is_bigendian else '<') + kind).reshape(height, width)
    if kind == 'u2':
        out = arr.astype(np.float32) / 1000.0
        out[arr == 0] = np.nan
    else:
        out = arr.astype(np.float32)
        out[~np.isfinite(out) | (out <= 0)] = np.nan
    return out


def _valid(d):
    return np.isfinite(d) & (d > 0)


def valid_fraction(d):
    return float(np.mean(_valid(np.asarray(d, float))))


def backproject(d, k):
    fx, fy, cx, cy = k
    d = np.asarray(d, float)
    v, u = np.nonzero(_valid(d))
    z = d[v, u]
    return np.column_stack([(u - cx) * z / fx, (v - cy) * z / fy, z])


def _refine(p, n, d0, thresh):
    for _ in range(2):
        inl = p[np.abs(p @ n + d0) < thresh]
        if len(inl) < 3:
            break
        c = inl.mean(axis=0)
        n = np.linalg.svd(inl - c, full_matrices=False)[2][-1]
        d0 = -float(n @ c)
    return n, d0


def floor_from_depth(d, k, thresh=0.02, iters=300, max_points=20000, min_points=500, seed=0):
    """Largest plane (RANSAC, then least squares on its inliers). Returns pitch of the camera below that plane's
    horizontal (deg), the camera's height above it (m), inlier share and RMS, or None with too few points.
    Optical frame: the plane's normal pointing at the camera is (0, -cos(pitch), -sin(pitch))."""
    p = backproject(d, k)
    if len(p) < min_points:
        return None
    rng = np.random.default_rng(seed)
    if len(p) > max_points:
        p = p[rng.choice(len(p), max_points, replace=False)]
    best, best_count = None, -1
    for _ in range(iters):
        a, b, c = p[rng.choice(len(p), 3, replace=False)]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-12:
            continue
        n = n / norm
        d0 = -float(n @ a)
        count = int(np.sum(np.abs(p @ n + d0) < thresh))
        if count > best_count:
            best, best_count = (n, d0), count
    if best is None:
        return None
    n, d0 = _refine(p, *best, thresh)
    if d0 < 0:                                             # orient the normal towards the camera (origin)
        n, d0 = -n, -d0
    resid = p @ n + d0
    inl = np.abs(resid) < thresh
    return {'pitch_deg': math.degrees(math.atan2(-n[2], -n[1])), 'height_m': d0,
            'normal': [float(x) for x in n], 'inlier_frac': float(inl.mean()),
            'rms_m': float(np.sqrt(np.mean(resid[inl] ** 2))) if inl.any() else None}


def compare_depth(a, b, bad=0.10, min_n=1000):
    """Relative error (a - b) / b over pixels valid in both. median_rel is the median |error|, bias_rel the
    median signed error, frac_bad the share with |error| > `bad`. None values when fewer than min_n overlap."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    m = _valid(a) & _valid(b)
    n = int(m.sum())
    if n < min_n:
        return {'n': n, 'median_rel': None, 'bias_rel': None, 'p90_rel': None, 'frac_bad': None}
    rel = (a[m] - b[m]) / b[m]
    ab = np.abs(rel)
    return {'n': n, 'median_rel': float(np.median(ab)), 'bias_rel': float(np.median(rel)),
            'p90_rel': float(np.percentile(ab, 90)), 'frac_bad': float(np.mean(ab > bad))}


def best_shift(a, b, max_shift=8, cap=0.5):
    """The s that best matches a[:, x] with b[:, x + s], scored by the mean of min(|relative error|, cap) so that
    depth edges decide it (a median is flat across small shifts of piecewise-flat depth). Returns (s, score)."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    w = a.shape[1]
    best = None
    for s in range(-max_shift, max_shift + 1):
        if abs(s) >= w:
            continue
        pa, pb = (a[:, :w - s], b[:, s:]) if s >= 0 else (a[:, -s:], b[:, :w + s])
        m = _valid(pa) & _valid(pb)
        if not m.any():
            continue
        score = float(np.mean(np.minimum(np.abs((pa[m] - pb[m]) / pb[m]), cap)))
        if best is None or score < best[1] - 1e-12 or (abs(score - best[1]) <= 1e-12 and abs(s) < abs(best[0])):
            best = (s, score)
    return best if best is not None else (None, None)


def stamp_stats(stamps, nominal_hz):
    """Frames missing from a stream, judged by header stamps against the nominal period (a gap of k periods
    means k - 1 frames never arrived). Duplicates are counted and dropped. None with fewer than 2 frames."""
    t = np.sort(np.asarray(stamps, float))
    if len(t) < 2:
        return None
    keep = np.concatenate([[True], np.diff(t) > 1e-6])
    dup = int((~keep).sum())
    t = t[keep]
    if len(t) < 2:
        return None
    period = 1.0 / nominal_hz
    gaps = np.diff(t)
    k = np.maximum(np.rint(gaps / period).astype(int), 1)
    missing = int(np.sum(k - 1))
    expected = len(t) + missing
    return {'n': int(len(t)), 'missing': missing, 'missing_pct': 100.0 * missing / expected,
            'worst_gap_ms': float(gaps.max() * 1000.0), 'rate_hz': float((len(t) - 1) / (t[-1] - t[0])),
            'duplicates': dup}


def pair_fraction(left, right, tol_s=0.002):
    """Share of left stamps that have a right stamp within tol_s."""
    lt, rt = np.asarray(left, float), np.sort(np.asarray(right, float))
    if not len(lt):
        return 0.0
    if not len(rt):
        return 0.0
    i = np.clip(np.searchsorted(rt, lt), 1, len(rt) - 1) if len(rt) > 1 else np.zeros(len(lt), int)
    near = np.minimum(np.abs(rt[i] - lt), np.abs(rt[np.maximum(i - 1, 0)] - lt))
    return float(np.mean(near <= tol_s + 1e-12))


_SCALARS = ('height', 'width', 'distortion_model', 'frame_id')
_ARRAYS = ('d', 'k', 'r', 'p')


def camera_info_diff(a, b, tol=1e-6):
    out = [f for f in _SCALARS if a.get(f) != b.get(f)]
    for f in _ARRAYS:
        x, y = np.asarray(a.get(f, []), float), np.asarray(b.get(f, []), float)
        if x.shape != y.shape or not np.allclose(x, y, atol=tol, rtol=0):
            out.append(f)
    return out


def tf_diff(a, b):
    """(translation distance in m, rotation angle in deg) between transforms given as ((x,y,z), (qx,qy,qz,qw))."""
    ta, qa = np.asarray(a[0], float), np.asarray(a[1], float)
    tb, qb = np.asarray(b[0], float), np.asarray(b[1], float)
    qa, qb = qa / np.linalg.norm(qa), qb / np.linalg.norm(qb)
    dot = min(1.0, abs(float(qa @ qb)))
    return float(np.linalg.norm(ta - tb)), math.degrees(2.0 * math.acos(dot))
