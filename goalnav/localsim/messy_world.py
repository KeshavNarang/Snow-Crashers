"""Sensor models for the irregular course (``localsim.messy_course``).

On top of the voxel ray caster (``localsim.world.World``):

- tarps move: every surface above the floor ripples along the ray by up to
  about 2 cm, a few travelling waves of 0.5-1.5 Hz;
- the net is almost invisible: a ray leaving the net box returns from the
  net cords only with a probability that falls with distance (2 % at 0.4 m
  or closer, 0.3 % at 1 m), else it continues to the cluttered room behind
  (people walking, tables, the hall walls, floor);
- stereo depth: the measured OAK-D error model (``localsim.oakd_model``):
  disparity noise, outliers, holes by surface, edge bands, the left strip;
- a downward VL53L1X: the nearest return in its 27 degree cone, so boulders
  and tarps under the drone shorten it;
- materials (from photos of the course): the tunnel is a wire-mesh tube, so
  only about a third of its mesh returns depth in any frame (it is still
  solid to touch).
"""
import math

import numpy as np

import hashlib
from pathlib import Path

from localsim import oakd_model
from localsim.world import World

FLUTTER_M = 0.02                 # vinyl stretched over plywood: it ripples, it does not billow
NET_RETURN = (0.003, 0.02)       # p = min(cap, k / r^2): thin black cord, almost never matched
RANGE_MAX_M = 4.0
RANGE_HALF_ANGLE = math.radians(13.5)
MESH_RETURN = 0.35               # wire tunnel: share of mesh voxels seen per frame
SOLID, MESH, TARP = 1, 2, 3
GLASS_RETURN = 0.04              # the hall's windows: a reflection now and then, else daylight (no depth)
MULLION_PITCH, MULLION_HALF = 1.5, 0.04
GLASS_SILL, GLASS_HEAD = 0.45, 2.7


def glass_hit(glass_x, origin, directions):
    """(distance to the glass wall's plane, frame or not) for each ray; inf where it is not met."""
    with np.errstate(divide='ignore', invalid='ignore'):
        t = np.where(directions[:, 0] < 0, (glass_x - origin[0]) / directions[:, 0], np.inf)
    t = np.where(t > 0, t, np.inf)
    p = origin + directions * np.where(np.isfinite(t), t, 0.)[:, None]
    y = np.mod(p[:, 1], MULLION_PITCH)
    frame = (np.minimum(y, MULLION_PITCH - y) < MULLION_HALF) | (p[:, 2] < GLASS_SILL) | (p[:, 2] > GLASS_HEAD)
    return t, frame & np.isfinite(t)


class MessyWorld(World):
    def __init__(self, obj_path, meta, seed=0, cache_dir=None, **kwargs):
        super().__init__(obj_path, cache_dir=cache_dir, **kwargs)
        self.meta = meta
        self.label = self._labels(cache_dir)
        self.frame = 0
        self.rng = np.random.default_rng(seed + 7919)
        self.now = 0.
        n = meta['nets']
        self.net_low = np.array([-n['west'], -n['south'], -0.01])
        self.net_high = np.array([n['east'], n['north'], 3.048])
        k = 6
        directions = self.rng.normal(size=(k, 3))
        directions /= np.linalg.norm(directions, axis=1)[:, None]
        self.waves = {'k': directions * self.rng.uniform(2., 6., (k, 1)),
                      'omega': self.rng.uniform(3., 9., k), 'phase': self.rng.uniform(0, 2 * math.pi, k),
                      'amp': self.rng.dirichlet(np.ones(k)) * FLUTTER_M}

    def _labels(self, cache_dir):
        """Material of each solid voxel: wire mesh, vinyl tarp or solid."""
        key = hashlib.sha256(f'labels:{self.mesh.sha256}:{self.voxel}'.encode()).hexdigest()[:16]
        cache = Path(cache_dir) / f'labels-{key}.npy' if cache_dir else None
        if cache and cache.exists():
            return np.load(cache)
        label = np.zeros(self.solid.shape, np.int8)
        vertices = np.asarray(self.mesh.vertices, float)
        order = [(MESH, ('Tube oval',)), (TARP, ('Tarp', 'Tunnel tarp', 'Tunnel drape')), (SOLID, None)]
        for value, prefixes in order:
            for name, tris in self.mesh.groups.items():
                if not tris or name.startswith(('Floor',)) and value != SOLID:
                    continue
                kind = MESH if name.startswith('Tube oval') else TARP if name.startswith(
                    ('Tarp', 'Tunnel tarp', 'Tunnel drape')) else SOLID
                if kind != value:
                    continue
                cells = self._surface_cells(vertices[np.asarray(tris)], self.origin, self.shape)
                label[tuple(cells.T)] = value
        if cache:
            cache.parent.mkdir(parents=True, exist_ok=True)
            np.save(cache, label)
        return label

    def cast(self, origin, directions, start, limit):
        """The voxel march, with the wire mesh only partly visible this frame."""
        hit = np.full(len(directions), np.inf)
        active = np.arange(len(directions))
        fine, coarse = self.voxel * 0.8, self.voxel * self.skip_factor
        t = np.full(len(directions), start)
        salt = (self.frame * 2654435761) & 0xffffffff
        while active.size:
            points = origin + directions[active] * t[active, None]
            index = np.floor((points - self.origin) / self.voxel).astype(int)
            inside = np.all((index >= 0) & (index < self.shape), axis=1) & (t[active] < limit)
            occupied = np.zeros(active.size, dtype=bool)
            cells = tuple(index[inside].T)
            solid = self.solid[cells]
            mesh = self.label[cells] == MESH
            if mesh.any():
                ix, iy, iz = (c[mesh].astype(np.int64) for c in cells)
                h = ((ix * 73856093) ^ (iy * 19349663) ^ (iz * 83492791) ^ salt) % 1000
                seen = solid.copy()
                seen[np.nonzero(mesh)[0][h >= MESH_RETURN * 1000]] = False
                solid = seen
            occupied[inside] = solid
            hit[active[occupied]] = t[active[occupied]]
            near = np.ones(active.size, dtype=bool)
            near[inside] = self.skip[tuple((index[inside] // self.skip_factor).T)]
            keep = ~occupied & inside
            t[active] += np.where(near, fine, coarse)
            active = active[keep]
        return hit

    # -- moving tarps ------------------------------------------------------
    def flutter(self, points):
        w = self.waves
        phase = points @ w['k'].T - w['omega'] * self.now + w['phase']
        return np.sin(phase) @ w['amp']

    # -- what lies outside the net ----------------------------------------
    def clutter_boxes(self):
        """Axis-aligned (low, high) boxes at the current time."""
        boxes = []
        for b in self.meta['clutter']:
            along = b['along'] + b['amp'] * math.sin(b['omega'] * self.now + b['phase'])
            sx, sy, sz = b['size']
            side = b['side']
            if side in ('north', 'south'):
                y = (self.net_high[1] + b['out']) if side == 'north' else (self.net_low[1] - b['out'])
                centre, half = (along, y), (sx / 2, sy / 2)
            else:
                x = (self.net_high[0] + b['out']) if side == 'east' else (self.net_low[0] - b['out'])
                centre, half = (x, along), (sy / 2, sx / 2)
            boxes.append((np.array([centre[0] - half[0], centre[1] - half[1], -0.01]),
                          np.array([centre[0] + half[0], centre[1] + half[1], sz])))
        return boxes

    @staticmethod
    def _box_entry(origin, directions, low, high):
        with np.errstate(divide='ignore', invalid='ignore'):
            a = (low - origin) / directions
            b = (high - origin) / directions
        near = np.nanmax(np.minimum(a, b), axis=1)
        far = np.nanmin(np.maximum(a, b), axis=1)
        return np.where((far >= np.maximum(near, 0.)) & (far > 0), np.maximum(near, 0.), np.inf)

    def _net_exit(self, origin, directions):
        with np.errstate(divide='ignore', invalid='ignore'):
            a = (self.net_low - origin) / directions
            b = (self.net_high - origin) / directions
        return np.nanmin(np.where(np.maximum(a, b) > 0, np.maximum(a, b), np.inf), axis=1)

    def _beyond(self, origin, directions):
        """First return outside the net: clutter, the room's floor or ceiling."""
        best = np.full(len(directions), np.inf)
        for low, high in self.clutter_boxes():
            best = np.minimum(best, self._box_entry(origin, directions, low, high))
        with np.errstate(divide='ignore', invalid='ignore'):
            floor = np.where(directions[:, 2] < 0, -origin[2] / directions[:, 2], np.inf)
            ceiling = np.where(directions[:, 2] > 0, (6. - origin[2]) / directions[:, 2], np.inf)
        best = np.minimum(best, np.minimum(floor, ceiling))
        hall = self.meta.get('hall')
        if hall:
            t, frame = glass_hit(hall['glass_x'], origin, directions)
            reflect = self.rng.random(len(t)) < GLASS_RETURN
            glass = t < best
            best[glass & (frame | reflect)] = t[glass & (frame | reflect)]
            best[glass & ~frame & ~reflect] = np.inf           # Through the glass: outdoors, out of range.
        return best

    # -- the stereo depth model ------------------------------------------------
    def observe(self, origin, directions, hit, camera):
        """Ray lengths the OAK-D reports: tarps ripple, the net is nearly
        invisible, then the measured stereo error model (``localsim.oakd_model``)."""
        self.frame += 1
        out = hit.copy()
        kinds = np.full(len(out), oakd_model.KINDS.index('beyond'))
        finite = np.isfinite(out)
        if finite.any():
            points = origin + directions[finite] * out[finite, None]
            index = np.clip(np.floor((points - self.origin) / self.voxel).astype(int), 0, self.shape - 1)
            label = self.label[tuple(index.T)]
            kind = np.where(label == TARP, oakd_model.KINDS.index('tarp'),
                            np.where(label == MESH, oakd_model.KINDS.index('mesh'), oakd_model.KINDS.index('plywood')))
            kind[points[:, 2] < 0.03] = oakd_model.KINDS.index('floor')
            kinds[finite] = kind
            above = points[:, 2] > 0.03
            moved = out[finite]
            moved[above] -= self.flutter(points[above])
            out[finite] = moved
        net = self._net_exit(origin, directions)
        through = net < out
        strand = self.rng.random(len(out)) < np.minimum(NET_RETURN[1], NET_RETURN[0] / np.maximum(net, 0.05) ** 2)
        caught = through & strand
        out[caught] = net[caught]
        kinds[caught] = oakd_model.KINDS.index('net')
        passed = through & ~strand
        if passed.any():
            out[passed] = np.minimum(out[passed], self._beyond(origin, directions[passed]))
            kinds[passed] = oakd_model.KINDS.index('beyond')
        grid = (len(np.arange(camera.stride / 2, camera.height, camera.stride)),
                len(np.arange(camera.stride / 2, camera.width, camera.stride)))
        return oakd_model.apply(out, kinds, grid, camera, self.rng)

    # -- the downward rangefinder -----------------------------------------------
    def range_down(self, position, rotation):
        """VL53L1X distance (m) from the body origin along body -z, nearest
        in its cone; None beyond range."""
        ring = [(0., 0.)] + [(RANGE_HALF_ANGLE, a) for a in np.linspace(0, 2 * math.pi, 8, endpoint=False)]
        body = np.array([[math.sin(t) * math.cos(a), math.sin(t) * math.sin(a), -math.cos(t)] for t, a in ring])
        directions = body @ rotation.T
        hit = self.cast(position, directions, 0.0, RANGE_MAX_M + 0.5)
        hit = hit * np.abs(directions[:, 2])          # Report the vertical component, as the axis ranges.
        best = float(np.min(hit))
        if not math.isfinite(best) or best > RANGE_MAX_M:
            return None
        return best + float(self.rng.normal(0., 0.01))
