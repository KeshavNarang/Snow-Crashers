"""Touchdown on the pad, and the return spot's distance from the barrier's stands.

Observations (localsim, seed 405, severity 0.8, 2026-11-08):
- on the floor at 64.0 s, ON_PAD only at 67.4 s: ON_PAD waited for PX4's land detector, so the
  drone sat ~3.9 s on the pad for a 0.5 s rest. PX4 auto-disarms 2 s after its own 'landed'.
- RETURN_DESCEND by the launch mark: the rotor discs over the barrier stand's floor battens
  (viewer screenshot): the spot kept 0.42 m from the barrier's centreline, the stand's feet and
  pavers reach ~0.29 m out (localsim messy_course PAVER_OUT) and the rotor tips 0.29 m.
"""
import math
from pathlib import Path
import unittest

import yaml

from drone_ref.goalnav.homeward import HomewardConfig
from drone_ref.goalnav.mission import Touchdown

GOALS = Path(__file__).resolve().parents[1] / 'config' / 'goals.yaml'
STAND_OUT = 0.29        # stand feet / pavers from a panel's centreline at floor level


class TouchdownOnOurOwnSensing(unittest.TestCase):
    def test_declared_from_height_and_speed_before_px4_says_landed(self):
        td = Touchdown(base_height=0.07)
        t, hit = 64.0, None
        while t < 65.0 and hit is None:
            if td.update(t, height=0.07, vz=0.0, landed=False):
                hit = t
            t += 0.02
        self.assertIsNotNone(hit, 'never declared touchdown without PX4 landed')
        self.assertLess(hit - 64.0, 0.3)

    def test_not_while_still_descending(self):
        td = Touchdown(base_height=0.07)
        self.assertFalse(any(td.update(63.0 + k * 0.02, height=0.30 - k * 0.005, vz=-0.3, landed=False)
                             for k in range(20)))

    def test_px4_landed_counts_at_once(self):
        self.assertTrue(Touchdown(base_height=0.07).update(70.0, height=0.2, vz=-0.1, landed=True))


class ReturnSpotClearsTheStands(unittest.TestCase):
    def test_rotor_tips_clear_the_barrier_stand_by_10_cm(self):
        walls = yaml.safe_load(GOALS.read_text())['return_walls']
        barrier = [w for w in walls if 'barrier' in w.get('name', '')]
        self.assertTrue(barrier, 'no return wall named barrier in goals.yaml')
        gap = barrier[0]['offset'] - HomewardConfig().body_radius - STAND_OUT
        self.assertGreaterEqual(gap, 0.10)


class ReturnSpotMidwayNetAndM1(unittest.TestCase):
    """v8 (seed 405): with M1's offset at 0.805 m the spot sat 0.41 m from the west net (rotor
    tips 0.12 m off it). The team: split the difference between the net and M1."""

    def test_spot_halfway_between_the_west_net_and_m1_for_its_whole_swing(self):
        walls = {w.get('name'): w for w in yaml.safe_load(GOALS.read_text())['return_walls']}
        offset, (a, _) = walls['M1']['offset'], walls['M1']['wall']
        net_x, y = -3.048, 0.36 + walls['barrier']['offset']         # spot's row, off the barrier
        for swing in (30., 45., 60.):                                  # M1 east of north (walls-rebuilt)
            n = (-math.cos(math.radians(swing)), math.sin(math.radians(swing)))
            dist_m1 = lambda x: (x - a[0]) * n[0] + (y - a[1]) * n[1]
            lo, hi = net_x, a[0]
            for _ in range(60):                                        # bisect: net gap == M1 gap
                mid = (lo + hi) / 2
                lo, hi = (mid, hi) if mid - net_x < dist_m1(mid) else (lo, mid)
            self.assertAlmostEqual(offset, mid - net_x, delta=0.05, msg=f'M1 swung {swing} deg')


class ReturnStaysOverTheWallsNearThem(unittest.TestCase):
    """localsim seed 602 (severity 0.7, near ladder, 2026-11-08): on the way down the body overlapped
    the barrier stand by 9.5 cm at (-2.47, 0.59, 0.81), 0.23 m past the barrier's centreline: the
    return's height came from the live map, built at takeoff and drifted since. Near a return wall
    (known lines: barrier, M1) the feet must stay over the wall top; at the touchdown spot (0.70-0.725 m
    off them) it may come down."""

    BARRIER = ((-2.90, 0.324), (-2.15, 0.402))

    def test_0_23_m_past_the_barrier_holds_over_the_wall_top(self):
        from drone_ref.goalnav.homeward import wall_floor
        floor = wall_floor([(-2.47, 0.59)], [self.BARRIER], feet_below_origin=0.07)
        self.assertGreaterEqual(floor, 1.73 + 0.07 + 0.15)

    def test_at_the_touchdown_spot_it_may_come_down(self):
        from drone_ref.goalnav.homeward import wall_floor
        spot = (-2.55, 0.36 + 0.725)
        self.assertEqual(wall_floor([spot], [self.BARRIER], feet_below_origin=0.07), 0.)



if __name__ == '__main__':
    unittest.main()
