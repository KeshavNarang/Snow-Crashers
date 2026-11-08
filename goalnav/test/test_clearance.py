"""The clearance metric (localsim.clearance) on a hand-built voxel world.

    cd goalnav && python -m unittest discover -s test
"""
import sys
import types
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from localsim import clearance              # noqa: E402
from localsim.goalrun import drone_contacts  # noqa: E402
from localsim.world import World            # noqa: E402

VOXEL = 0.025


def wall_world(wall_x=1.0, z_range=(0.0, 2.0)):
    """A 4 x 4 x 2.5 m box of free space with one plane of contact voxels at x = wall_x."""
    shape = (160, 160, 100)
    contact = np.zeros(shape, bool)
    i = int(wall_x / VOXEL)
    k0, k1 = int(z_range[0] / VOXEL), int(z_range[1] / VOXEL)
    contact[i, :, k0:k1] = True
    w = types.SimpleNamespace(origin=np.zeros(3), voxel=VOXEL, shape=np.array(shape), contact=contact)
    w.contacts = types.MethodType(World.contacts, w)     # the judge's own contact count
    return w, (i + 0.5) * VOXEL                          # the wall's voxel centres


class ClearanceTest(unittest.TestCase):
    def test_gap_from_the_rotor_rim(self):
        w, face = wall_world()
        p = (0.5, 2.0125, 1.0)     # on a voxel centre in y
        self.assertAlmostEqual(clearance.body_clearance(w, p), face - 0.5 - clearance.BODY_RADIUS, places=6)

    def test_negative_exactly_when_the_judge_counts_contact(self):
        # The metric must agree with goalrun's contact judge: below zero only when it sees a touch.
        w, face = wall_world()
        for x in np.linspace(face - 0.40, face - 0.20, 41):
            p = (x, 2.0, 1.0)
            touching = drone_contacts(w, np.array(p), clearance.BODY_RADIUS) > 0
            self.assertEqual(clearance.body_clearance(w, p) <= 0, touching, msg=f'x={x:.3f}')

    def test_far_wall_reports_the_search_cap(self):
        w, _ = wall_world(wall_x=3.5)
        self.assertEqual(clearance.body_clearance(w, (0.5, 2.0, 1.0)), clearance.SEARCH_M)

    def test_wall_out_of_the_body_band_does_not_count(self):
        # A batten under the feet (z 0-0.10) is not beside a body flying at 1 m.
        w, _ = wall_world(z_range=(0.0, 0.10))
        self.assertEqual(clearance.body_clearance(w, (0.8, 2.0, 1.0)), clearance.SEARCH_M)

    def test_net(self):
        nets = {'west': 3.0, 'east': 2.0, 'south': 2.5, 'north': 1.5}
        self.assertAlmostEqual(clearance.net_clearance(nets, (1.5, 0.0, 1.0)), 0.5 - clearance.BODY_RADIUS)
        self.assertAlmostEqual(clearance.net_clearance(nets, (0.0, 1.4, 1.0)), 0.1 - clearance.BODY_RADIUS)

    def test_target_is_a_quarter_of_the_tunnel_radius(self):
        self.assertAlmostEqual(clearance.TARGET_M, 0.7366 / 4)


if __name__ == '__main__':
    unittest.main()
