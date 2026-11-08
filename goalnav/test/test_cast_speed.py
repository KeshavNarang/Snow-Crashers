"""MessyWorld.cast speed-up: same answers, measurably faster.

``reference_cast`` below is a FROZEN copy of the ray march before the speed-up. The profile that
motivated it (one 200 s goalrun, laptop): 86 s wall, of which ``MessyWorld.cast`` is 37.5 s
self-time (43 s with callees), called by the RGB/mono render (``messy_rgb.render``, 40 s) and the
rangefinder (``range_down``). The new cast must (1) give bit-identical hits, including the
wire-mesh tunnel's per-frame hash and rays that leave the grid, (2) be faster in the same process.
"""
import math
import sys
import tempfile
import time
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from localsim import messy_course                          # noqa: E402
from localsim.messy_world import MESH, MESH_RETURN, MessyWorld   # noqa: E402


# ---- frozen reference (do not edit: it is the behaviour being preserved) --------------------------
def reference_cast(self, origin, directions, start, limit):
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
# ----------------------------------------------------------------------------------------------------


def _rays(n_side, yaw, pitch_down):
    """A camera-like fan of unit rays about a heading (yaw from +x, pitch down)."""
    u = np.linspace(-0.6, 0.6, n_side)
    uu, vv = np.meshgrid(u, u * 0.62)
    forward = np.array([math.cos(yaw) * math.cos(pitch_down), math.sin(yaw) * math.cos(pitch_down),
                        -math.sin(pitch_down)])
    left = np.array([-math.sin(yaw), math.cos(yaw), 0.])
    up = np.cross(forward, left)
    d = forward + uu.reshape(-1, 1) * left + vv.reshape(-1, 1) * up
    return d / np.linalg.norm(d, axis=1)[:, None]


class CastTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        course = messy_course.generate(3, 0.7)
        obj, meta_path = messy_course.save(course, Path(cls.tmp.name) / 'course')
        import json
        cls.meta = json.loads(meta_path.read_text())
        cls.world = MessyWorld(obj, cls.meta, seed=3, cache_dir=ROOT / 'localsim/.cache')
        tube = messy_course.TUBE
        # Poses over the course: launch, the maze, facing into the tunnel (wire mesh), the window
        # room, the pad, and one by the net looking out (rays leave the grid).
        cls.cases = [((-2.5, 0.9, 1.1), math.pi / 2, 0.48, 0.05, 12.),
                     ((-1.3, 1.6, 1.15), math.pi / 2, 0.48, 0.05, 12.),
                     ((tube['cx'] - 0.1, 2.0, 0.85), -math.pi / 2, 0.30, 0.05, 12.),     # into the tunnel
                     ((tube['cx'] - 0.1, 1.0, 0.85), -math.pi / 2, 0.30, 0.05, 12.),     # inside it
                     ((0.5, -2.4, 1.18), math.radians(160), 0.48, 0.05, 12.),
                     ((-1.6, -2.0, 1.5), math.pi, 1.2, 0.0, 4.5),                       # rangefinder-like
                     ((-2.9, 2.5, 1.2), math.pi, 0.1, 0.05, 12.)]                         # out over the net

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def _compare(self, frame):
        w = self.world
        w.frame = frame
        for position, yaw, pitch, start, limit in self.cases:
            d = _rays(60, yaw, pitch)
            origin = np.array(position, float)
            want = reference_cast(w, origin, d, start, limit)
            got = w.cast(origin, d, start, limit)
            np.testing.assert_array_equal(got, want, err_msg=f'{position} frame {frame}')
        return want

    def test_identical_hits_over_the_course(self):
        for frame in (0, 1, 7, 12345):          # the mesh hash changes every frame
            self._compare(frame)

    def test_cases_exercise_the_mesh_misses_and_the_grid_edge(self):
        # So the identity above cannot pass by not covering what changed.
        w = self.world
        w.frame = 1
        meshed, missed = 0, 0
        for position, yaw, pitch, start, limit in self.cases:
            hit = reference_cast(w, np.array(position, float), _rays(60, yaw, pitch), start, limit)
            missed += np.count_nonzero(~np.isfinite(hit))
            p = np.array(position) + _rays(60, yaw, pitch) * np.where(np.isfinite(hit), hit, 0)[:, None]
            idx = np.floor((p[np.isfinite(hit)] - w.origin) / w.voxel).astype(int)
            idx = np.clip(idx, 0, w.shape - 1)
            meshed += np.count_nonzero(w.label[tuple(idx.T)] == MESH)
        self.assertGreater(meshed, 50, 'no ray stopped on the wire tunnel')
        self.assertGreater(missed, 50, 'no ray left the grid or ran out of range')

    def test_faster_in_the_same_process(self):
        w = self.world
        w.frame = 3
        jobs = [(np.array(p, float), _rays(120, yaw, pitch), s, l) for p, yaw, pitch, s, l in self.cases[:5]]

        def best_of(fn, n=3):
            times = []
            for _ in range(n):
                t0 = time.perf_counter()
                for origin, d, s, l in jobs:
                    fn(w, origin, d, s, l)
                times.append(time.perf_counter() - t0)
            return min(times)

        old = best_of(reference_cast)
        new = best_of(MessyWorld.cast)
        print(f'\ncast: reference {old * 1e3:.0f} ms, new {new * 1e3:.0f} ms, x{old / new:.2f}')
        self.assertLess(new, old / 1.3)


if __name__ == '__main__':
    unittest.main()
