"""Where to touch down on the return, from walls seen on the way back.

By the return the odometry has drifted (up to ~0.2 m in the simulator), so
the launch position it remembers is that far off, and the launch corridor
leaves only ~0.25 m a side (M1 to the nearly invisible west net is one short
panel). Instead the drone measures the walls by the mark (goals.yaml
``return_walls``: M1's corridor face and the barrier's north face) from depth
seen on the way back, in the frame it is flying in now, and touches down a
set distance from each. The map is no use for this: its cells were placed
when the drone took off, before the drift.
"""
import numpy as np

from drone_ref.goalnav.goals import near_face


class ReturnSpot:
    def __init__(self, route, target, walls, max_shift=0.3, window=0.45, min_points=40, keep=4000):
        self.route, self.base = route, np.asarray(target, float)
        self.walls = walls                # [{wall: [[x, y], [x, y]] (CAD), offset: m}]
        self.max_shift, self.window, self.min_points, self.keep = max_shift, window, min_points, keep
        self.lines = None
        self.points = [np.zeros((0, 2)) for _ in walls]

    @staticmethod
    def _normal(line):
        a, b = line
        d = (b - a) / np.linalg.norm(b - a)
        return np.array([-d[1], d[0]])

    def observe(self, origin, points, floor_z, low=0.6, high=1.65):
        """Depth points (odometry frame) seen from ``origin``. A wall's points
        count only when seen from the mark's side of it (M1's corridor face is
        in view over the barrier, before the drone is past it). Only the band
        ``low``-``high`` over the floor: lower down, the tarps bulge and pool."""
        if not self.walls:
            return
        if self.lines is None:            # The course heading is learned by now.
            self.lines = [tuple(self.route.cad_point(p) for p in w['wall']) for w in self.walls]
        z = points[:, 2] - floor_z
        xy = points[(z > low) & (z < high), :2]
        for k, line in enumerate(self.lines):
            normal, a = self._normal(line), line[0]
            if float((np.asarray(origin[:2]) - a) @ normal) * float((self.base - a) @ normal) <= 0:
                continue
            near = np.abs((xy - a) @ normal) < self.window
            self.points[k] = np.vstack([self.points[k], xy[near]])[-self.keep:]

    def _face(self, k):
        a, b = self.lines[k]
        return near_face(self.points[k], a, b, self.window, self.min_points)

    def target(self):
        """The touchdown spot: each measured wall's offset from it, the move
        from the remembered launch capped at ``max_shift`` (the 2 ft mark)."""
        spot = self.base.copy()
        for k, wall in enumerate(self.walls if self.lines else []):
            face = self._face(k)
            if face is None:
                continue
            normal, a = self._normal(self.lines[k]), self.lines[k][0]
            off = float((spot - a) @ normal) - face        # the spot from the face, signed
            spot = spot + normal * (np.sign(off) * float(wall['offset']) - off)
        shift = spot - self.base
        norm = float(np.linalg.norm(shift))
        return self.base + shift * min(1., self.max_shift / norm) if norm > 0 else spot

    def measured(self):
        return self.lines is not None and all(self._face(k) is not None for k in range(len(self.walls)))
