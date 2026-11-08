"""A waypoint whose estimate lands on an obstacle is aimed at the nearest clear point near it.

Observation (localsim, seed 705, severity 0.7, near ladder, no lapses, 2026-11-08): the
window_lookout estimate (placed from the previous goals' estimates plus the CAD offset) sat on a
mapped obstacle (clearance 0); the planner found no path for 25 s and aborted 'no progress toward
window_front', then the descent hit a wall.
"""
import unittest

import numpy as np

from drone_ref.goalnav.livemap import LiveMap
from drone_ref.goalnav.planner import clearance
from drone_ref.goalnav.mission import free_target


class FreeTarget(unittest.TestCase):
    def setUp(self):
        self.map = LiveMap((0., 0.), 0.)
        blocked = np.zeros((self.map.n, self.map.n), bool)
        i, j = self.map.cell(np.array([0., 0.]))
        blocked[i - 6:i + 7, j - 6:j + 7] = True               # a 0.65 m block on the target
        self.clear = clearance(blocked, self.map.res)

    def test_a_target_on_an_obstacle_moves_to_clear_space_close_by(self):
        t = free_target(self.map, self.clear, np.array([0., 0.]), need=0.33, search=0.9)
        self.assertIsNotNone(t)
        ij = self.map.cell(t)
        self.assertGreaterEqual(self.clear[ij[0], ij[1]], 0.33)
        self.assertLess(float(np.linalg.norm(t)), 0.9)

    def test_a_clear_target_stays_where_it_is(self):
        q = np.array([1.5, 1.5])
        self.assertTrue(np.allclose(free_target(self.map, self.clear, q, need=0.33, search=0.9), q))


if __name__ == '__main__':
    unittest.main()
