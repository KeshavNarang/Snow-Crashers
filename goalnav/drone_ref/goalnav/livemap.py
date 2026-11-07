"""A live height-banded occupancy map, built from depth during one run.

The map lives in the odometry frame (ENU, metres) and starts empty at every
start: nothing is stored between runs (rule 6.4). Each XY cell keeps a
log-odds value per 10 cm height band above the floor. Depth returns add
evidence of occupancy; the space a ray crossed before its return adds
evidence of free space; each cell gets at most one of either per frame. It
takes hits in three frames to call a cell occupied: a real surface is seen
in every frame, while the OAK-D's confident wrong-depth patches (repeating
prints, mesh) move with the viewpoint and seldom land on a cell three times.

Two fences keep the invisible net from corrupting the map: returns beyond
``point_fence`` (the expected net plus a margin) are the room behind the net,
so they add no obstacle, but the ray to them still clears the space it
crossed inside the net; rays only clear space inside ``carve_fence``, so the few
returns the net strands give are never erased by the many rays that pass
through them.
"""
from dataclasses import dataclass

import numpy as np

HIT, MISS = 0.7, -0.4         # Balanced: a phantom seen now and then fades; a surface seen every frame stays.
LOW, HIGH = -2.0, 3.5
OCCUPIED = 2.0                 # Hits in three frames: a wall seen for 0.2 s, not a passing wrong-depth patch.
FLOOR_BAND = 0.06              # Returns this close to the floor are the floor (and its battens).


@dataclass
class Fence:
    """An axis-aligned XY box in the odometry frame, possibly rotated by yaw
    about ``centre`` (the course's estimated heading)."""
    centre: tuple
    half: tuple
    yaw: float = 0.

    def contains(self, xy, margin=0.):
        c, s = np.cos(self.yaw), np.sin(self.yaw)
        d = np.asarray(xy, float) - self.centre
        u = c * d[..., 0] + s * d[..., 1]
        v = -s * d[..., 0] + c * d[..., 1]
        return (np.abs(u) <= self.half[0] + margin) & (np.abs(v) <= self.half[1] + margin)


class LiveMap:
    def __init__(self, centre_xy, floor_z, size_m=9.6, resolution=0.05, band=0.1, top=2.4):
        self.res, self.band = resolution, band
        self.n = int(round(size_m / resolution))
        self.origin = np.asarray(centre_xy, float) - size_m / 2
        self.bands = int(round(top / band))
        self.floor_z = float(floor_z)
        self.logodds = np.zeros((self.bands, self.n, self.n), np.float32)
        self.seen = np.zeros((self.n, self.n), bool)
        self.point_fence = None
        self.carve_fence = None
        self.frames = 0

    # -- indexing ---------------------------------------------------------------
    def cell(self, xy):
        return np.floor((np.asarray(xy, float) - self.origin) / self.res).astype(int)

    def centre_of(self, ij):
        return self.origin + (np.asarray(ij, float) + 0.5) * self.res

    def inside(self, ij):
        return np.all((ij >= 0) & (ij < self.n), axis=-1)

    def band_of(self, z):
        return np.floor((np.asarray(z, float) - self.floor_z) / self.band).astype(int)

    # -- updates ------------------------------------------------------------------
    def insert(self, origin, points, step=None):
        """Add one depth frame: ``points`` (N, 3) odometry-frame returns seen
        from the camera centre ``origin``."""
        step = step or self.res
        points = np.asarray(points, float).reshape(-1, 3)
        hits = points[points[:, 2] > self.floor_z + FLOOR_BAND] if len(points) else points
        if self.point_fence is not None and len(hits):
            hits = hits[self.point_fence.contains(hits[:, :2])]
        origin = np.asarray(origin, float)
        size = self.bands * self.n * self.n
        update = np.zeros(size, np.float32)
        if len(points):
            ray = points - origin
            length = np.linalg.norm(ray, axis=1)
            count = np.maximum(((length - 0.1) / step).astype(int), 0)
            total = int(count.sum())
            if total:
                owner = np.repeat(np.arange(len(points)), count)
                offsets = np.arange(total) - np.repeat(np.cumsum(count) - count, count)
                along = (offsets + 0.5) * step / np.maximum(length[owner], 1e-6)
                samples = origin + ray[owner] * along[:, None]
                if self.carve_fence is not None:
                    samples = samples[self.carve_fence.contains(samples[:, :2], margin=-0.05)]
                flat = self._flat(samples)
                update[np.unique(flat[flat >= 0])] += MISS           # Once per frame per cell.
            flat = self._flat(hits)
            hit_cells = np.unique(flat[flat >= 0])
            update[hit_cells] = HIT                               # A hit outweighs this frame's misses.
        changed = update != 0
        grid = self.logodds.reshape(-1)
        grid[changed] = np.clip(grid[changed] + update[changed], LOW, HIGH)
        self.seen |= changed.reshape(self.bands, self.n, self.n).any(axis=0)
        self.frames += 1

    def mark(self, xy, z_top, weight=HIT):
        """Evidence of something solid under the drone from the floor up to
        ``z_top`` (the rangefinder saw a boulder)."""
        ij = self.cell(xy)
        if not self.inside(ij):
            return
        top = max(0, min(self.bands, int(self.band_of(z_top)) + 1))
        column = self.logodds[:top, ij[0], ij[1]]
        column[:] = np.clip(column + weight, LOW, HIGH)

    def _flat(self, points):
        ij = self.cell(points[:, :2])
        k = self.band_of(points[:, 2])
        ok = self.inside(ij) & (k >= 0) & (k < self.bands)
        flat = np.full(len(points), -1, int)
        flat[ok] = (k[ok] * self.n + ij[ok, 0]) * self.n + ij[ok, 1]
        return flat

    # -- queries ------------------------------------------------------------------------
    def bands_between(self, z_low, z_high):
        low = max(0, int(self.band_of(z_low)))
        high = min(self.bands, int(self.band_of(z_high)) + 1)
        return low, max(low + 1, high)

    def blocked(self, z_low, z_high):
        """XY cells occupied anywhere between the two odometry heights."""
        low, high = self.bands_between(z_low, z_high)
        return (self.logodds[low:high] > OCCUPIED).any(axis=0)

    def free_interval(self, ij, z_low, z_high):
        """(bottom, top) odometry heights of the free gap around the middle of
        [z_low, z_high] in one column, from occupied bands below and above."""
        low, high = self.bands_between(z_low, z_high)
        column = self.logodds[:, ij[0], ij[1]] > OCCUPIED
        middle = (low + high) // 2
        below = [k for k in range(low, middle + 1) if column[k]]
        above = [k for k in range(middle, high) if column[k]]
        bottom = (below[-1] + 1) if below else low
        top = above[0] if above else high
        return self.floor_z + bottom * self.band, self.floor_z + top * self.band

    def occupied_cells(self, z_low, z_high):
        """(N, 2) XY centres of occupied cells in the height range, for display."""
        return self.centre_of(np.argwhere(self.blocked(z_low, z_high)))

    def snapshot(self, every=1):
        """Occupied voxels (i, j, band) for display."""
        return np.argwhere(self.logodds[:, ::every, ::every] > OCCUPIED)
