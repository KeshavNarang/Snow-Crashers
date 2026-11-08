"""Eval for ps5_remote.message (no controller, no drone needed)."""
import unittest

from ps5_remote import message


class Ps5MessageTest(unittest.TestCase):
    def test_r2_l2_are_up_down_without_l1(self):
        # Asked 2026-11-08: R2/L2 up/down, the controller flies the whole course (no dead-man).
        base = {'l1': False, 'lx': 0.0, 'ly': -1.0, 'l2': 0.0, 'r2': 1.0}
        self.assertEqual(message(base, 1, [], 0.0)['sticks'], [1.0, 0.0, 1.0])
        self.assertEqual(message(dict(base, ly=0.0, r2=0.0, l2=0.6), 1, [], 0.0)['sticks'], [0.0, 0.0, -0.6])


if __name__ == '__main__':
    unittest.main()
