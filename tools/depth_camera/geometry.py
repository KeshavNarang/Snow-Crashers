"""geometry: decide which camera frame a depth image is really in, against an independent host stereo match.

  scale_k(k, from_size, to_size)          intrinsics for a resized image (pixel-centre convention)
  reproject_depth(d, k_src, R, t, k_dst)  move a depth image into another camera (z-buffered, nearest surface wins)
  identify_frame(device, candidates)      which candidate frame the device depth matches (median |error| <= 3 %)
  host_depth(left, right, fx, baseline)   OpenCV SGBM on a rectified pair -> depth in the left AND the right frame

Frames: optical (x right, y down, z forward). R, t map a point from the source frame into the destination frame:
X_dst = R @ X_src + t (the right camera of a rectified pair is t = (-baseline, 0, 0) from the left one).
"""
import numpy as np

import depth_analysis as da

try:
    import cv2
    HAVE_CV2 = True
except ImportError:
    cv2, HAVE_CV2 = None, False


def scale_k(k, from_size, to_size):
    fx, fy, cx, cy = k
    sx, sy = to_size[0] / from_size[0], to_size[1] / from_size[1]
    return (round(fx * sx, 6), round(fy * sy, 6), round((cx + 0.5) * sx - 0.5, 6), round((cy + 0.5) * sy - 0.5, 6))


def reproject_depth(depth, k_src, r, t, k_dst, size_dst):
    """Depth image of the same surfaces seen from another camera. Pixels nothing projects to are NaN."""
    p = da.backproject(depth, k_src)
    w, h = size_dst
    out = np.full(h * w, np.inf)
    if len(p):
        q = p @ np.asarray(r, float).T + np.asarray(t, float)
        z = q[:, 2]
        ok = z > 1e-6
        q, z = q[ok], z[ok]
        fx, fy, cx, cy = k_dst
        u = np.rint(fx * q[:, 0] / z + cx).astype(np.int64)
        v = np.rint(fy * q[:, 1] / z + cy).astype(np.int64)
        inside = (u >= 0) & (u < w) & (v >= 0) & (v < h)
        np.minimum.at(out, v[inside] * w + u[inside], z[inside])
    out[~np.isfinite(out)] = np.nan
    return out.reshape(h, w)


def identify_frame(device, candidates, max_rel=0.03, min_n=1000):
    scores = {name: da.compare_depth(device, cand, min_n=min_n) for name, cand in candidates.items()}
    ok = [(s['median_rel'], name) for name, s in scores.items() if s['median_rel'] is not None and s['median_rel'] <= max_rel]
    return {'best': min(ok)[1] if ok else None, 'scores': scores}


def host_depth(left, right, fx, baseline, num_disp=96, block=7):
    """SGBM on a rectified mono8 pair. Returns (depth in the left frame, depth in the right frame), metres, NaN = no match.
    The right-frame map comes from matching the mirrored pair, so it is an independent match, not a reprojection."""
    if not HAVE_CV2:
        raise RuntimeError('OpenCV is required for host_depth')
    sgbm = cv2.StereoSGBM_create(minDisparity=0, numDisparities=num_disp, blockSize=block, P1=8 * block * block,
                                 P2=32 * block * block, uniquenessRatio=10, disp12MaxDiff=1,
                                 mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)

    def depth_from(a, b):
        disp = sgbm.compute(a, b).astype(np.float32) / 16.0
        d = np.full(disp.shape, np.nan, np.float32)
        good = disp > 0.5
        d[good] = fx * baseline / disp[good]
        return d

    dl = depth_from(left, right)
    dr = np.fliplr(depth_from(np.ascontiguousarray(np.fliplr(right)), np.ascontiguousarray(np.fliplr(left))))
    return dl, dr
