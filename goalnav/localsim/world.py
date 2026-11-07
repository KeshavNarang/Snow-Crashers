"""Course geometry from the published CAD: voxels, depth rendering, collisions."""
from dataclasses import dataclass
import hashlib
import math
from pathlib import Path

import numpy as np

from drone_ref.course import read_obj
from localsim.exact_depth import ExactFaces, upsample

# Flat floor markings: visible to the camera as floor, never obstacles.
DECALS = ('Guideline', 'Landing pad', 'Launch mark', 'Approach ring')
# Rendered (the camera sees them) but not wall contacts.
NON_CONTACT = ('Floor', 'Net roof')


@dataclass(frozen=True)
class Camera:
    """Rectified pinhole camera, optical frame x right, y down, z forward."""
    width: int = 640
    height: int = 400
    hfov_deg: float = 80.
    min_depth: float = 0.2
    max_depth: float = 10.
    # base_link <- optical: camera 0.10 m ahead of the body origin.
    offset: tuple = (0.10, 0., 0.)
    stride: int = 8          # ray grid step in pixels; repeated to full resolution
    pitch_deg: float = 0.    # tilted down about body y (the team's mount: 27)

    def body_from_optical(self):
        """Optical (x right, y down, z forward) -> body FLU rotation, with the tilt."""
        t = math.radians(self.pitch_deg)
        pitch = np.array([[math.cos(t), 0., math.sin(t)], [0., 1., 0.], [-math.sin(t), 0., math.cos(t)]])
        return pitch @ np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)

    @property
    def fx(self):
        return (self.width / 2) / math.tan(math.radians(self.hfov_deg) / 2)

    @property
    def cx(self):
        return self.width / 2 - 0.5

    @property
    def cy(self):
        return self.height / 2 - 0.5


class World:
    def __init__(self, obj_path, voxel=0.025, cache_dir=None, dark_tunnel=False):
        self.voxel = voxel
        self.dark_tunnel = dark_tunnel
        mesh = read_obj(obj_path)
        self.mesh, self.exact = mesh, None
        key = hashlib.sha256(f'{mesh.sha256}:{voxel}'.encode()).hexdigest()[:16]
        cache = Path(cache_dir) / f'voxels-{key}.npz' if cache_dir else None
        if cache and cache.exists():
            data = np.load(cache)
            self.solid, self.contact, self.origin = data['solid'], data['contact'], data['origin']
        else:
            self.solid, self.contact, self.origin = self._voxelize(mesh)
            if cache:
                cache.parent.mkdir(parents=True, exist_ok=True)
                np.savez_compressed(cache, solid=self.solid, contact=self.contact, origin=self.origin)
        self.tunnel_box = self._bounds(mesh, 'Tube oval')
        self.shape = np.array(self.solid.shape)
        self._build_skip_grid()

    def _build_skip_grid(self, factor=4):
        """Coarse occupancy dilated by one cell: empty there means nothing solid
        within one coarse cell, so a ray may jump that far safely."""
        f = factor
        padded = np.zeros(tuple(-(-n // f) * f for n in self.solid.shape), dtype=bool)
        padded[:self.solid.shape[0], :self.solid.shape[1], :self.solid.shape[2]] = self.solid
        c = padded.reshape(padded.shape[0] // f, f, padded.shape[1] // f, f, padded.shape[2] // f, f).any(axis=(1, 3, 5))
        d = c.copy()
        for axis in range(3):
            d |= np.roll(c, 1, axis) | np.roll(c, -1, axis)
        grown = d.copy()
        for axis in range(3):          # also edges and corners
            grown |= np.roll(d, 1, axis) | np.roll(d, -1, axis)
        self.skip, self.skip_factor = grown, f

    @staticmethod
    def _bounds(mesh, prefix):
        index = [i for name, tris in mesh.groups.items() if name.startswith(prefix) for t in tris for i in t]
        if not index:
            return None
        points = np.asarray(mesh.vertices)[index]
        return points.min(axis=0), points.max(axis=0)

    def _voxelize(self, mesh):
        vertices = np.asarray(mesh.vertices, dtype=float)
        all_points = vertices
        origin = all_points.min(axis=0) - self.voxel
        shape = np.ceil((all_points.max(axis=0) - origin) / self.voxel).astype(int) + 2
        solid = np.zeros(shape, dtype=bool)
        contact = np.zeros(shape, dtype=bool)
        for name, tris in mesh.groups.items():
            if not tris or name.startswith(DECALS):
                continue
            cells = self._surface_cells(vertices[np.asarray(tris)], origin, shape)
            solid[tuple(cells.T)] = True
            if not name.startswith(NON_CONTACT):
                contact[tuple(cells.T)] = True
        return solid, contact, origin

    def _surface_cells(self, tris, origin, shape):
        cells = []
        a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
        longest = np.maximum.reduce([np.linalg.norm(b - a, axis=1), np.linalg.norm(c - a, axis=1),
                                     np.linalg.norm(c - b, axis=1)])
        steps = np.maximum(1, np.ceil(longest / (self.voxel * 0.5))).astype(int)
        for n in np.unique(steps):
            pick = steps == n
            u, v = np.meshgrid(np.arange(n + 1), np.arange(n + 1))
            keep = (u + v) <= n
            u, v = u[keep] / n, v[keep] / n
            pa, pb, pc = a[pick], b[pick], c[pick]
            points = (pa[:, None] + u[None, :, None] * (pb - pa)[:, None]
                      + v[None, :, None] * (pc - pa)[:, None]).reshape(-1, 3)
            index = np.floor((points - origin) / self.voxel).astype(int)
            index = index[np.all((index >= 0) & (index < shape), axis=1)]
            cells.append(np.unique(index, axis=0))
        return np.unique(np.concatenate(cells), axis=0)

    def in_tunnel(self, position):
        if self.tunnel_box is None:
            return False
        low, high = self.tunnel_box
        return bool(np.all(position >= low) and np.all(position <= high))

    def render_depth(self, camera, position, rotation, exact=False):
        """Depth image in millimetres (uint16, 0 = no return) at full resolution.

        Rays are cast at 1/stride resolution and repeated, matching the vision
        node's nearest-per-tile sampling at that stride. ``exact`` re-intersects
        planar boxes exactly and interpolates (``localsim.exact_depth``; slower).
        """
        s = camera.stride
        u = np.arange(s / 2, camera.width, s)
        v = np.arange(s / 2, camera.height, s)
        uu, vv = np.meshgrid(u, v)
        rays_cam = np.stack([(uu - camera.cx) / camera.fx, (vv - camera.cy) / camera.fx,
                             np.ones_like(uu)], axis=-1).reshape(-1, 3)
        world_rays = rays_cam @ (rotation @ camera.body_from_optical()).T
        origin = position + rotation @ np.asarray(camera.offset)
        norms = np.linalg.norm(world_rays, axis=1)
        directions = world_rays / norms[:, None]
        hit = self.cast(origin, directions, camera.min_depth * 0.5, camera.max_depth * 1.6)
        if exact:
            if self.exact is None:
                self.exact = ExactFaces(self.mesh, skip=DECALS)
            hit = self.exact.refine(origin, directions, hit, 2 * self.voxel)
        depth = self.observe(origin, directions, hit, camera) / norms   # Optical z depth, not ray length.
        valid = np.isfinite(depth) & (depth >= camera.min_depth) & (depth <= camera.max_depth)
        if self.dark_tunnel and self.in_tunnel(origin):
            valid[:] = False  # Passive stereo has no texture in the dark.
        millimetres = np.where(valid, np.round(depth * 1000), 0).astype(np.uint16)
        image = millimetres.reshape(len(v), len(u))
        if exact:
            return upsample(image, s, camera.width, camera.height)
        return np.repeat(np.repeat(image, s, axis=0), s, axis=1)[:camera.height, :camera.width]

    def observe(self, origin, directions, hit, camera):
        """Ray lengths the sensor reports for true hits; a hook for sensor models."""
        return hit

    def cast(self, origin, directions, start, limit):
        """Distance along each unit ray to the first solid voxel (inf if none)."""
        hit = np.full(len(directions), np.inf)
        active = np.arange(len(directions))
        fine, coarse = self.voxel * 0.8, self.voxel * self.skip_factor
        t = np.full(len(directions), start)
        while active.size:
            points = origin + directions[active] * t[active, None]
            index = np.floor((points - self.origin) / self.voxel).astype(int)
            inside = np.all((index >= 0) & (index < self.shape), axis=1) & (t[active] < limit)
            occupied = np.zeros(active.size, dtype=bool)
            occupied[inside] = self.solid[tuple(index[inside].T)]
            hit[active[occupied]] = t[active[occupied]]
            near = np.ones(active.size, dtype=bool)
            near[inside] = self.skip[tuple((index[inside] // self.skip_factor).T)]
            keep = ~occupied & inside
            t[active] += np.where(near, fine, coarse)
            active = active[keep]
        return hit

    def contacts(self, position, radius, half_height):
        """Number of wall/obstacle voxels inside the body cylinder."""
        low = np.floor((position - [radius, radius, half_height] - self.origin) / self.voxel).astype(int)
        high = np.floor((position + [radius, radius, half_height] - self.origin) / self.voxel).astype(int) + 1
        low, high = np.maximum(low, 0), np.minimum(high, self.shape)
        block = self.contact[low[0]:high[0], low[1]:high[1], low[2]:high[2]]
        if not block.any():
            return 0
        ix, iy = np.nonzero(block.any(axis=2))
        centres_x = self.origin[0] + (low[0] + ix + .5) * self.voxel - position[0]
        centres_y = self.origin[1] + (low[1] + iy + .5) * self.voxel - position[1]
        return int(np.count_nonzero(np.hypot(centres_x, centres_y) <= radius))
