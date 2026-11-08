"""3D points from the OAK-D-S2's two mono cameras, in a level frame (no ROS: tested on its own).

The skybeta stack runs the camera without its on-board depth processor (i_pipeline_type:
stereo has no StereoDepth node), so the drone publishes no depth topic. This does the same
job on the Jetson from what it does publish:

    /oak/left/image_mono, /oak/right/image_mono   mono8 1024x640 @ 30 Hz, raw (not rectified)
    /oak/{left,right}/camera_info                  K (D reported as zeros, R identity)
    TF oak_left_camera_optical_frame -> oak_right   the stereo extrinsics cuVSLAM uses
                                                     (7.5 cm baseline, ~0.3/0.7 deg rotation)

1. Rectify both images with those numbers (the principal points differ by ~12 px vertically),
   at half resolution to save CPU.
2. Semi-global block matching (OpenCV SGBM) over most of the image.
3. Every matched pixel becomes a 3D point (Z = f * B / disparity) in the left camera frame.
4. ``level()`` rotates them into a level body frame (F forward, L left, U up, metres from the
   camera) using the camera's mount pitch. **The camera on skybeta-03 is pitched 27 deg down**
   (the stack's TF says ~0): it sees from 0.5 to 53.5 deg below horizontal, mostly floor,
   never anything above its own height.
5. ``fit_floor()`` finds the floor plane (RANSAC) and from it the camera's real pitch and
   height: a check on the mount angle.
"""
import math

import cv2
import numpy as np


def _matcher(num_disparities, block):
    create = getattr(cv2, 'StereoSGBM_create', None) or cv2.StereoSGBM.create
    return create(minDisparity=0, numDisparities=num_disparities, blockSize=block,
                  P1=8 * block * block, P2=32 * block * block, disp12MaxDiff=1,
                  uniquenessRatio=10, speckleWindowSize=50, speckleRange=2,
                  mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)


def quaternion_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def extrinsics_from_tf(translation, quaternion_xyzw):
    """TF gives the pose of the right optical frame in the left one (lookup_transform(left,
    right)): X_left = R_lr X_right + t_lr. OpenCV wants X_right = R X_left + T."""
    r_lr = quaternion_to_matrix(*quaternion_xyzw)
    t_lr = np.asarray(translation, dtype=float).reshape(3, 1)
    return r_lr.T, -r_lr.T @ t_lr


def mount_matrix(pitch_deg, roll_deg=0.0):
    """Columns: the camera optical axes (x right, y down, z forward) in the level frame
    (F, L, U) for a camera pitched ``pitch_deg`` DOWN (and rolled ``roll_deg``, right side down)."""
    p, r = math.radians(pitch_deg), math.radians(roll_deg)
    x_c = np.array([0.0, -1.0, 0.0])
    y_c = np.array([-math.sin(p), 0.0, -math.cos(p)])
    z_c = np.array([math.cos(p), 0.0, -math.sin(p)])
    if r:
        cr, sr = math.cos(r), math.sin(r)
        x_c, y_c = cr * x_c + sr * y_c, -sr * x_c + cr * y_c
    return np.stack([x_c, y_c, z_c], axis=1)


def level(points_cam, pitch_deg, roll_deg=0.0):
    """N x 3 camera-frame points -> N x 3 (F forward, L left, U up), metres from the camera."""
    return np.asarray(points_cam, dtype=float) @ mount_matrix(pitch_deg, roll_deg).T


def fit_floor(points_cam, iterations=80, tolerance=0.02, seed=0, min_inliers=0.2):
    """RANSAC floor plane in the camera frame -> dict(pitch_deg, roll_deg, height_m, inliers)
    for the camera relative to that plane, or None. Pitch is positive DOWN, roll positive
    right side down; height is the camera's distance above the plane."""
    pts = np.asarray(points_cam, dtype=float)
    if len(pts) < 50:
        return None
    rng = np.random.default_rng(seed)
    if len(pts) > 3000:
        pts = pts[rng.choice(len(pts), 3000, replace=False)]
    best, best_count = None, 0
    for _ in range(iterations):
        a, b, c = pts[rng.choice(len(pts), 3, replace=False)]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        count = np.count_nonzero(np.abs((pts - a) @ n) < tolerance)
        if count > best_count:
            best, best_count = (n, a), count
    if best is None or best_count < min_inliers * len(pts):
        return None
    n, a = best
    inliers = pts[np.abs((pts - a) @ n) < tolerance]
    centre = inliers.mean(axis=0)
    n = np.linalg.svd(inliers - centre)[2][2]
    d = -float(n @ centre)                      # plane: n . X + d = 0
    if d < 0:                                    # orient n from the floor up towards the camera
        n, d = -n, -d
    # Level up direction in camera coordinates is (0, -cos p, -sin p) for pitch p down.
    pitch = math.degrees(math.atan2(-n[2], -n[1]))
    roll = math.degrees(math.atan2(-n[0], math.hypot(n[1], n[2])))
    return {'pitch_deg': round(pitch, 2), 'roll_deg': round(roll, 2), 'height_m': round(d, 3),
            'inliers': round(len(inliers) / float(len(pts)), 2)}


class StereoRig:
    """Rectification maps + matcher for one calibration. ``points(left, right)`` -> reading."""

    def __init__(self, k_left, k_right, size, r, t, scale=0.5, d_left=None, d_right=None,
                 num_disparities=96, block=7, row_span=(0.0, 0.9), step=2, max_range_m=6.0,
                 texture_min=10.0, saturation_px=3):
        self.num_disparities, self.block = num_disparities, block
        self.texture_min, self.saturation_px = float(texture_min), saturation_px
        w, h = int(round(size[0] * scale)), int(round(size[1] * scale))
        self.size, self.scale = (w, h), scale
        s = np.diag([scale, scale, 1.0])
        k1 = s @ np.asarray(k_left, dtype=float).reshape(3, 3)
        k2 = s @ np.asarray(k_right, dtype=float).reshape(3, 3)
        d1 = np.zeros(5) if d_left is None else np.asarray(d_left, dtype=float)
        d2 = np.zeros(5) if d_right is None else np.asarray(d_right, dtype=float)
        r = np.asarray(r, dtype=float).reshape(3, 3)
        t = np.asarray(t, dtype=float).reshape(3, 1)
        r1, r2, p1, p2, _q, _a, _b = cv2.stereoRectify(k1, d1, k2, d2, (w, h), r, t,
                                                       flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
        self.maps = (cv2.initUndistortRectifyMap(k1, d1, r1, p1, (w, h), cv2.CV_16SC2),
                     cv2.initUndistortRectifyMap(k2, d2, r2, p2, (w, h), cv2.CV_16SC2))
        self.r1 = r1                              # X_rectified = R1 X_left
        self.f = float(p1[0, 0])
        self.cx, self.cy = float(p1[0, 2]), float(p1[1, 2])
        self.baseline = abs(float(p2[0, 3]) / float(p2[0, 0]))
        self.rows = (int(row_span[0] * h), int(row_span[1] * h))
        self.cols = (num_disparities, w)          # no match possible left of this
        self.step = step
        self.matcher = _matcher(num_disparities, block)
        self.min_range = self.f * self.baseline / num_disparities
        self.max_range = max_range_m
        vs = np.arange(self.rows[0], self.rows[1], step)
        us = np.arange(self.cols[0], self.cols[1], step)
        self.grid_u, self.grid_v = np.meshgrid(us.astype(np.float32), vs.astype(np.float32))

    def rectify(self, left, right):
        out = []
        for image, (m1, m2) in zip((left, right), self.maps):
            if image.shape[1] != self.size[0]:
                image = cv2.resize(image, self.size, interpolation=cv2.INTER_AREA)
            out.append(cv2.remap(image, m1, m2, cv2.INTER_LINEAR))
        return out

    def top_elevation_deg(self, pitch_deg):
        """Elevation (deg, negative = below horizontal) of the highest ray the points cover."""
        ray = np.array([0.0, (self.rows[0] - self.cy) / self.f, 1.0]) @ self.r1   # rectified -> left
        f, _l, u = level(ray[None, :], pitch_deg)[0]
        return math.degrees(math.atan2(u, f))

    def points(self, left, right):
        """Two mono8 frames -> (N x 3 left-camera-frame points in metres, matched fraction).

        Two rejections that matter on real images (found on skybeta-03 looking at a glossy
        table): on low-texture or glare patches SGBM piles matches up at its largest disparity,
        a fake surface square-on to the camera at the minimum range. So pixels with too little
        horizontal texture (``texture_min``, mean |d/dx| over the block) and disparities within
        ``saturation_px`` of the search limit are dropped."""
        lr, rr = self.rectify(left, right)
        r0, r1 = self.rows
        band = lr[r0:r1]
        disparity = self.matcher.compute(band, rr[r0:r1]).astype(np.float32) / 16.0
        if self.texture_min > 0:
            gx = np.abs(cv2.Sobel(band, cv2.CV_32F, 1, 0, ksize=3))
            disparity[cv2.blur(gx, (self.block, self.block)) < self.texture_min] = 0.0
        disparity[disparity >= self.num_disparities - self.saturation_px] = 0.0
        d = disparity[::self.step, self.cols[0]::self.step][:self.grid_u.shape[0], :self.grid_u.shape[1]]
        good = d > 0.5
        z = np.zeros_like(d)
        z[good] = self.f * self.baseline / d[good]
        good &= (z > 0.05) & (z < self.max_range)
        zg = z[good]
        x = (self.grid_u[good] - self.cx) * zg / self.f
        y = (self.grid_v[good] - self.cy) * zg / self.f
        rect = np.stack([x, y, zg], axis=1)
        return rect @ self.r1, float(good.mean()) if good.size else 0.0
