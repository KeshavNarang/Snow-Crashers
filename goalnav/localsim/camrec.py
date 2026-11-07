"""Keep what the OAK-D saw, for the replay viewer: the RGB frame, the depth
frame and what the pad detector made of the RGB.

Each kept frame writes three small PNGs into ``<run>/cam/`` and one entry in
the trace: the time, the detector's blobs with their verdicts, and two
points projected into the RGB image from the true pose: the H's real centre
and where the drone was aiming to land.
"""
import math
from pathlib import Path
import shutil

import numpy as np

from localsim.png import write_png

# Depth colours, near to far (0.3 to 5 m): a perceptual ramp, black for no return.
RAMP = np.array([[250, 230, 60], [240, 140, 40], [200, 60, 80], [120, 40, 140], [40, 40, 110]], float)
NEAR, FAR = 0.3, 5.0


class CameraRecorder:
    def __init__(self, out_dir, period_s=0.4):
        self.dir = Path(out_dir) / 'cam'
        shutil.rmtree(self.dir, ignore_errors=True)
        self.dir.mkdir(parents=True)
        self.period = period_s
        self.next = -math.inf
        self.frames = []

    def add(self, t, rgb_camera, colour, depth_camera, depth_mm, pose, detection, marks):
        """``detection``: the pad detector's record for this frame (or None if it
        did not look); ``marks``: {name: course xyz} drawn into the RGB view."""
        if detection is None and t < self.next:
            return                          # Every frame the detector looks at; others now and then.
        self.next = t + self.period
        k = len(self.frames)
        write_png(self.dir / f'{k:04d}_rgb.png', colour)
        write_png(self.dir / f'{k:04d}_depth.png', depth_image(depth_camera, depth_mm))
        entry = {'t': round(t, 2), 'k': k}
        if detection is not None:
            write_png(self.dir / f'{k:04d}_mask.png', mask_image(detection))
            entry['blobs'] = [[*b['box'], b['why']] for b in detection['blobs'][:8]]
        entry['marks'] = {name: uv for name, xyz in marks.items()
                          if (uv := project(rgb_camera, pose, xyz)) is not None}
        self.frames.append(entry)

    def summary(self, rgb_camera, depth_camera):
        s = depth_camera.stride
        return {'frames': self.frames, 'rgb': [rgb_camera.width, rgb_camera.height],
                'depth': [depth_camera.width // s, depth_camera.height // s], 'near_far': [NEAR, FAR]}


def depth_image(camera, depth_mm, scale=2):
    """One pixel per cast ray (the sim repeats each over its stride), coloured."""
    s = camera.stride
    z = depth_mm[s // 2::s, s // 2::s].astype(float) / 1000.
    f = np.clip((z - NEAR) / (FAR - NEAR), 0., 1.) * (len(RAMP) - 1)
    low = np.minimum(f.astype(int), len(RAMP) - 2)
    w = (f - low)[..., None]
    rgb = RAMP[low] * (1 - w) + RAMP[low + 1] * w
    rgb[z <= 0] = 0.
    rgb = np.repeat(np.repeat(rgb, scale, axis=0), scale, axis=1)
    return rgb.astype(np.uint8)


def mask_image(detection):
    """Colour-model hits: dim for every blob, bright green for the one taken as the pad."""
    mask = detection['mask']
    rgb = np.zeros((*mask.shape, 3), np.uint8)
    rgb[mask] = (40, 110, 160)
    pick = detection.get('pick')
    if pick is not None:
        rgb[pick[:, 0], pick[:, 1]] = (60, 255, 120)
    return rgb


def project(camera, pose, xyz):
    """Course point -> RGB pixel (u, v) from the true pose, or None if behind or off the image."""
    position, rotation = pose
    origin = position + rotation @ np.asarray(camera.offset)
    optical = (rotation @ camera.body_from_optical()).T @ (np.asarray(xyz, float) - origin)
    if optical[2] < 0.05:
        return None
    u = camera.fx * optical[0] / optical[2] + camera.width / 2
    v = camera.fx * optical[1] / optical[2] + camera.height / 2
    if not (-0.5 * camera.width < u < 1.5 * camera.width and -0.5 * camera.height < v < 1.5 * camera.height):
        return None
    return [round(float(u), 1), round(float(v), 1)]
