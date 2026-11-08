"""goalnav holds still while PX4 has no vision, and re-measures after the estimate jumps.

Observation (localsim, seed 704, severity 0.9, near ladder, 6 fusion lapses a minute modelled on
skybeta-03 2026-11-08 where PX4 fused vision 60 % of a 30 s sample): a lapse (PX4 dead reckoning,
up to 2 s) ended just before the window settle; the estimate reset, and the drone flew into the
window's frame at (-0.43, -2.31): 'in contact with a wall for 1 s'.
"""
import unittest

from drone_ref.goalnav.mission import VisionGuard


class HoldWithoutVision(unittest.TestCase):
    def test_holds_while_dead_reckoning_and_for_a_second_after(self):
        g = VisionGuard(hold_s=1.0)
        self.assertFalse(g.update(10.0, dead_reckoning=False, resets=0).hold)
        self.assertTrue(g.update(10.1, dead_reckoning=True, resets=0).hold)
        self.assertTrue(g.update(11.5, dead_reckoning=True, resets=0).hold)
        self.assertTrue(g.update(11.6, dead_reckoning=False, resets=1).hold)     # vision back, estimate reset
        self.assertTrue(g.update(12.4, dead_reckoning=False, resets=1).hold)     # not steady for 1 s yet
        self.assertFalse(g.update(12.7, dead_reckoning=False, resets=1).hold)

    def test_a_reset_is_reported_once_so_the_window_is_measured_again(self):
        g = VisionGuard(hold_s=1.0)
        g.update(10.0, dead_reckoning=False, resets=3)
        self.assertTrue(g.update(10.1, dead_reckoning=False, resets=4).jumped)
        self.assertFalse(g.update(10.2, dead_reckoning=False, resets=4).jumped)

    def test_steady_vision_never_holds(self):
        g = VisionGuard(hold_s=1.0)
        self.assertFalse(any(g.update(10 + k * 0.02, dead_reckoning=False, resets=0).hold for k in range(500)))

    def test_tight_passages_wait_for_steady_vision(self):
        """Seed 704 with the hold (2026-11-08): touches in the tunnel and the window right after
        lapses: PX4 holds a drifting estimate, so the body wanders ~0.2 m where margins are 0.1-0.2 m."""
        g = VisionGuard(hold_s=1.0, steady_s=3.0)
        g.update(10.0, dead_reckoning=True, resets=0)
        g.update(11.0, dead_reckoning=False, resets=1)
        self.assertFalse(g.update(12.5, dead_reckoning=False, resets=1).hold)
        self.assertFalse(g.steady(12.5))           # open flight may go on, a tight passage waits
        self.assertTrue(g.steady(14.1))
        self.assertTrue(VisionGuard().steady(5.0))  # never lost: steady


if __name__ == '__main__':
    unittest.main()
