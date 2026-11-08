"""Evals for geometry_check.match_triplets(): python3 -m unittest -v test_triplets

The live geometry check compares each depth image with a host stereo match of the left/right images from the
SAME instant. Matching by header stamp must pair exact instants, skip depth frames without a partner, and never
reuse a left/right image for two depth frames.
"""
import unittest

import geometry_check as gc


def items(stamps, tag):
    return [(s, f'{tag}{i}') for i, s in enumerate(stamps)]


class Match(unittest.TestCase):
    def test_exact_instants_are_paired(self):
        t = [100.0 + i / 30 for i in range(10)]
        out = gc.match_triplets(items(t, 'L'), items(t, 'R'), items(t[2:5], 'D'), tol_s=0.003)
        self.assertEqual([(d, l, r) for (_, l, r, d) in out], [('D0', 'L2', 'R2'), ('D1', 'L3', 'R3'), ('D2', 'L4', 'R4')])

    def test_depth_without_partners_is_skipped(self):
        t = [100.0 + i / 30 for i in range(10)]
        right = [x for i, x in enumerate(t) if i != 3]
        out = gc.match_triplets(items(t, 'L'), items(right, 'R'), items(t[2:5], 'D'), tol_s=0.003)
        self.assertEqual([d for (_, _, _, d) in out], ['D0', 'D2'])

    def test_off_by_half_a_frame_is_not_a_match(self):
        t = [100.0 + i / 30 for i in range(10)]
        out = gc.match_triplets(items(t, 'L'), items(t, 'R'), items([x + 0.016 for x in t], 'D'), tol_s=0.003)
        self.assertEqual(out, [])

    def test_no_image_is_used_twice(self):
        t = [100.0, 100.0333]
        out = gc.match_triplets(items(t, 'L'), items(t, 'R'), items([100.0, 100.001], 'D'), tol_s=0.003)
        self.assertEqual(len(out), 1)


if __name__ == '__main__':
    unittest.main()
