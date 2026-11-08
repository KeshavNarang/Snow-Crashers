"""drone_ref.goalnav.homeward: the return's margins, touchdown spot and descent gate.

    cd goalnav && python -m unittest discover -s test
"""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from drone_ref.goalnav import homeward      # noqa: E402
from drone_ref.goalnav.homeward import DescentGate, HomewardConfig   # noqa: E402


class MarginTest(unittest.TestCase):
    def test_margins_come_from_the_clearance_target(self):
        cfg = HomewardConfig()
        self.assertAlmostEqual(cfg.clear_target, 0.7366 / 4)             # tunnel radius / 4
        self.assertAlmostEqual(cfg.reach, 0.29 + 0.7366 / 4)            # centre: rotor tips + target
        self.assertGreaterEqual(cfg.vertical_margin, cfg.clear_target)  # feet over the wall tops

    def test_old_margins_were_under_the_target(self):
        # The bug: mission's return_reach 0.45 (centre) left 0.16 m to the rotor tips, return_margin 0.15.
        self.assertLess(0.45 - 0.29, HomewardConfig().clear_target)
        self.assertLess(0.15, HomewardConfig().clear_target)

    def test_face_offset_never_under_the_reach(self):
        cfg = HomewardConfig()
        self.assertAlmostEqual(homeward.face_offset(0.42, cfg), cfg.reach)   # the barrier's 0.42 -> 0.474
        self.assertAlmostEqual(homeward.face_offset(0.50, cfg), 0.50)        # M1's already clears it
        off = HomewardConfig(clear_target=0.0)
        self.assertAlmostEqual(homeward.face_offset(0.42, off), 0.42)        # no target: as configured

    def test_cruise_height_is_capped(self):
        cfg = HomewardConfig()
        # Unseen cells are 1.85 m tall: feet over them by the vertical margin, origin 7 cm over the feet.
        need = homeward.cruise_height(1.85, cfg)
        self.assertAlmostEqual(need, 1.85 + cfg.vertical_margin + cfg.feet_below_origin)
        self.assertLessEqual(need, cfg.max_height)


class SpotFreezeTest(unittest.TestCase):
    def test_spot_follows_measurements_above_the_walls_and_freezes_beside_them(self):
        cfg = HomewardConfig()
        self.assertTrue(homeward.spot_may_move(cfg.freeze_below + 0.1, cfg))
        self.assertFalse(homeward.spot_may_move(cfg.freeze_below - 0.1, cfg))

    def test_freeze_is_at_the_wall_tops_plus_the_body(self):
        cfg = HomewardConfig()
        self.assertGreaterEqual(cfg.freeze_below, cfg.wall_top + cfg.feet_below_origin)


class DescentGateTest(unittest.TestCase):
    def test_descends_when_on_the_spot_and_still(self):
        g = DescentGate(HomewardConfig())
        self.assertAlmostEqual(g.rate(h=1.5, off=0.01, speed=0.02), g.cfg.fast_rate)
        self.assertAlmostEqual(g.rate(h=0.4, off=0.01, speed=0.02), g.cfg.slow_rate)

    def test_holds_when_off_the_spot_beside_the_walls(self):
        # s15 (drone scenario): drifting sideways while coming down past the barrier.
        g = DescentGate(HomewardConfig())
        self.assertEqual(g.rate(h=1.2, off=0.10, speed=0.02), 0.)
        self.assertEqual(g.rate(h=1.2, off=0.01, speed=0.20), 0.)

    def test_hysteresis(self):
        cfg = HomewardConfig()
        g = DescentGate(cfg)
        self.assertEqual(g.rate(h=1.2, off=cfg.hold_off + 0.01, speed=0.), 0.)
        between = (cfg.hold_off + cfg.resume_off) / 2
        self.assertEqual(g.rate(h=1.2, off=between, speed=0.), 0.)          # not back yet
        self.assertGreater(g.rate(h=1.2, off=cfg.resume_off - 0.005, speed=0.), 0.)
        self.assertGreater(g.rate(h=1.2, off=between, speed=0.), 0.)        # and stays going

    def test_above_the_walls_it_never_holds(self):
        g = DescentGate(HomewardConfig())
        self.assertGreater(g.rate(h=2.0, off=0.3, speed=0.3), 0.)

    def test_does_not_hold_forever(self):
        # A hold must not become a hover until the battery goes: after hold_max_s it carries on, slowly.
        cfg = HomewardConfig()
        g = DescentGate(cfg)
        t = 0.
        while t < cfg.hold_max_s + 0.5:
            r = g.rate(h=1.0, off=0.2, speed=0., dt=0.1)
            t += 0.1
        self.assertAlmostEqual(r, cfg.slow_rate)

    def test_on_the_ground_it_presses_down(self):
        g = DescentGate(HomewardConfig())
        self.assertGreater(g.rate(h=0.05, off=0.2, speed=0.2), 0.)          # never hold once landing


if __name__ == '__main__':
    unittest.main()
